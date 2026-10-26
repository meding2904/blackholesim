#version 330

// ===========================================================================
//  Schwarzschild black hole: per-pixel backward ray tracing of null geodesics
//
//  Units: G = c = 1, distances in gravitational radii r_g = GM/c^2 (M = 1).
//      event horizon  r = 2      photon sphere  r = 3      ISCO  r = 6
//      critical impact parameter b = 3*sqrt(3) ~ 5.196
//
//  Each camera ray lies in a plane through the hole. In that plane, with
//  u = r_g / r and phi measured from the camera's radial direction, photons obey
//
//      d^2u/dphi^2 + u = 3 u^2                                        (Binet)
//
//  which is integrated here with classical RK4 at a fixed dphi (bounded by
//  uMaxSteps). Initial conditions come from the camera ray's direction cosine
//  relative to the outward radial:
//
//      b    = r0 sin(psi) / sqrt(1 - 2 r_g/r0)      (impact parameter)
//      du   = -cos(psi) / b                          (radial derivative)
//
//  The first integral du^2 = 1/b^2 - u^2 + 2u^3 holds by construction.
//  Captured rays (u > 1/2) render black; escaping rays sample the procedural
//  sky along their asymptotic direction. The accretion disk is sampled as a
//  finite-thickness emissive/absorbing medium each time the ray passes through
//  the equatorial slab, so multiple lensed images accumulate naturally.
// ===========================================================================

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform vec2  uResolution;
uniform vec3  uCamPos;
uniform vec3  uCamRight;
uniform vec3  uCamUp;
uniform vec3  uCamForward;
uniform float uTanHalfFov;
uniform float uTime;          // disk animation time (seconds)
uniform float uDiskVisible;
uniform float uDiskInner;
uniform float uDiskOuter;
uniform float uRotationSpeed; // animation rate multiplier
uniform float uTempScale;     // Kelvin at the temperature profile peak
uniform float uDiskGain;      // emission normalisation
uniform float uDiskDensity;   // optical depth scale
uniform float uStepSize;      // dphi per RK4 step (radians)
uniform int   uMaxSteps;
uniform int   uAASamples;
uniform float uStarGain;
uniform sampler2D uSky;       // equirectangular procedural star field
uniform sampler2D uNoise;     // tileable procedural turbulence

const float PI = 3.14159265359;
const float ESCAPE_RADIUS = 900.0;
const vec3  DISK_AXIS = vec3(0.0, 1.0, 0.0);

// ---------------------------------------------------------------------------
//  Utility
// ---------------------------------------------------------------------------
float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

// Blackbody chromaticity (Tanner Helland fit, 1000 K .. 40000 K), linear RGB.
vec3 blackbodyRGB(float kelvin) {
    float t = clamp(kelvin, 1000.0, 40000.0) * 0.01;
    float r, g, b;

    if (t <= 66.0) {
        r = 255.0;
        g = clamp(99.4708025861 * log(t) - 161.1195681661, 0.0, 255.0);
    } else {
        r = clamp(329.698727446 * pow(t - 60.0, -0.1332047592), 0.0, 255.0);
        g = clamp(288.1221695283 * pow(t - 60.0, -0.0755148492), 0.0, 255.0);
    }

    if (t >= 66.0) b = 255.0;
    else if (t <= 19.0) b = 0.0;
    else b = clamp(138.5177312231 * log(t - 10.0) - 305.0447927307, 0.0, 255.0);

    return vec3(r, g, b) / 255.0;
}

vec3 sampleSky(vec3 dir) {
    float u = atan(dir.z, dir.x) * (0.5 / PI) + 0.5;
    float v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / PI);
    // Explicit LOD: the illumination loop has data-dependent exits, where
    // implicit derivatives (and therefore texture() LOD selection) are undefined.
    return textureLod(uSky, vec2(u, v), 0.0).rgb * uStarGain;
}

// ---------------------------------------------------------------------------
//  Accretion disk model
// ---------------------------------------------------------------------------

// Flaring thin slab, in r_g. The vertical Gaussian profile is what gives the
// disk its volumetric, self-occluding look near edge-on inclinations.
float diskHalfThickness(float r) { return 0.012 * r + 0.055; }

// Shakura-Sunyaev effective temperature for a thin disk:
//     T(r) ~ [ (1/r^3) (1 - sqrt(r_in/r)) ]^(1/4)
// normalised so the peak (at r = 49/36 r_in) is 1.
float temperatureNorm(float r) {
    float x = uDiskInner / max(r, 1e-3);
    float f = max(x * x * x * (1.0 - sqrt(clamp(x, 0.0, 1.0))), 0.0);
    return pow(f, 0.25) * (1.0 / 0.48773);
}

// Differential rotation: the pattern is advected by the local Keplerian rate, so
// inner rings lap outer ones. Three tileable noise scales, sheared into
// trailing filaments.
// footprint: world-space size of one pixel at this point, in r_g. The three
// noise layers use explicit LODs derived from it (the ray loop has divergent
// exits, so implicit derivatives would be undefined).
float diskTurbulence(float r, float theta, float footprint) {
    // Sample in the material's rotating frame: every radius is advected at its
    // own Keplerian rate, so the pattern shears into trailing filaments on its
    // own. Cartesian (cos, sin) coordinates keep the field seamless at the
    // +/-pi wrap and isotropic in physical space.
    float omega = pow(max(r, 1.0), -1.5) * uRotationSpeed;
    float a = theta - omega * uTime;
    vec2 p = vec2(cos(a), sin(a)) * r;

    // The turbulence texture is 512^2, so each layer has
    // 512 * scale texels per r_g.
    const float size = 512.0;
    float lod1 = max(log2(max(footprint * 0.090 * size, 1.0)), 0.0);
    float lod2 = max(log2(max(footprint * 0.240 * size, 1.0)), 0.0);
    float lod3 = max(log2(max(footprint * 0.620 * size, 1.0)), 0.0);

    float n1 = textureLod(uNoise, p * 0.090 + vec2(0.11, 0.53), lod1).r;
    float n2 = textureLod(uNoise, p * 0.240 + vec2(0.37, 0.23), lod2).g;
    float n3 = textureLod(uNoise, p * 0.620 + vec2(0.71, 0.89), lod3).b;

    float n = n1 * 0.50 + n2 * 0.32 + n3 * 0.18;
    n = clamp((n - 0.5) * 1.40 + 0.5, 0.0, 1.0);      // push the contrast
    return n * n * (3.0 - 2.0 * n);
}

// Doppler + gravitational shift for emission from a circular orbit:
//     g = nu_obs / nu_emit = sqrt(1 - 3M/r) / (1 - Omega * L_phi/E)
// The numerator carries gravitational and transverse-Doppler redshift, the
// denominator the line-of-sight motion. bPhi is the conserved L_phi/E of the
// traced ray (the sign is unchanged for the physical, time-forward photon).
float frequencyShift(float r, float bPhi) {
    float u = 1.0 / max(r, 1e-3);
    float omega = pow(u, 1.5);                 // Keplerian, M = 1
    float numerator = sqrt(max(1.0 - 3.0 * u, 0.02));
    float denominator = max(1.0 - omega * bPhi, 0.10);
    return numerator / denominator;
}

// Analytic crossing of the equatorial plane. The disk is modelled as a thin
// optically thick slab: the crossing gives a crisp surface, while the slab
// thickness divided by the ray's incidence cosine sets the optical depth (so
// grazing rays see a much longer column and the disk becomes opaque edge-on).
void accumulateDisk(inout vec3 accum, inout float transmittance, vec3 p0, vec3 p1, float bPhi) {
    if ((p0.y > 0.0) == (p1.y > 0.0)) return;   // no equatorial crossing in this step

    vec3 hit = mix(p0, p1, p0.y / (p0.y - p1.y));
    float r = length(hit);
    if (r <= uDiskInner || r >= uDiskOuter) return;

    vec3 dir = normalize(p1 - p0);
    float incidence = max(abs(dir.y), 0.02);
    float column = clamp(2.0 * diskHalfThickness(r) / incidence, 0.0, 24.0);

    float edgeIn = smoothstep(uDiskInner, uDiskInner * 1.18, r);
    float edgeOut = 1.0 - smoothstep(uDiskOuter * 0.58, uDiskOuter, r);
    float theta = atan(hit.z, hit.x);
    // Screen footprint: distance x angular pixel size, stretched by the
    // incidence angle so grazing views do not alias.
    float footprint = length(hit - uCamPos) * (2.0 * uTanHalfFov / uResolution.y) / incidence;
    float turb = diskTurbulence(r, theta, footprint);

    float density = edgeIn * edgeOut * (0.14 + 1.05 * turb);
    float tau = uDiskDensity * density * column;
    float alpha = 1.0 - exp(-tau);

    // The Shakura-Sunyaev profile vanishes at the inner edge (zero torque);
    // soften it so the rim glows instead of cutting to black.
    float tNorm = mix(0.78, temperatureNorm(r), smoothstep(uDiskInner, uDiskInner * 1.45, r)) *
                  (0.90 + 0.20 * turb);
    float g = frequencyShift(r, bPhi);

    // blackbodyRGB returns display-referred sRGB; convert to linear so the
    // tone mapper does not wash the warm colours out.
    vec3 tint = pow(blackbodyRGB(tNorm * uTempScale * g), vec3(2.2));
    tint = mix(vec3(dot(tint, vec3(0.2126, 0.7152, 0.0722))), tint, 1.25);

    // Bolometric surface brightness: sigma T^4 falls off steeply, softened here
    // so the outer disk stays readable, plus the g^4 beaming factor.
    float brightness = uDiskGain * pow(tNorm, 3.4) * pow(g, 4.0);
    brightness *= 0.30 + 0.95 * turb;
    brightness *= 1.0 + 1.40 * exp(-(r - uDiskInner) * 0.50);   // hot inner rim

    accum += transmittance * tint * brightness * alpha;
    transmittance *= (1.0 - alpha);
}

// ---------------------------------------------------------------------------
//  Ray setup
// ---------------------------------------------------------------------------
vec2 aaOffset(int index, int count) {
    if (count >= 4) {
        return vec2(((index == 0 || index == 3) ? -0.25 : 0.25),
                    (index < 2) ? -0.25 : 0.25);
    }
    if (count == 2) {
        return (index == 0) ? vec2(-0.25, -0.25) : vec2(0.25, 0.25);
    }
    return vec2(0.0);
}

vec3 traceCameraRay(vec3 origin, vec3 dir) {
    float r0 = length(origin);
    vec3 e1 = origin / r0;                     // outward radial basis
    float u0 = 1.0 / r0;
    float cosPsi = dot(dir, e1);
    vec3 tangential = dir - e1 * cosPsi;
    float sinPsi = length(tangential);

    // Purely radial rays carry no angular momentum: straight in or straight out.
    if (sinPsi < 1.0e-5) {
        return (cosPsi < 0.0) ? vec3(0.0) : sampleSky(dir);
    }

    vec3 e2 = tangential / sinPsi;                       // +phi direction
    vec3 planeNormal = normalize(cross(e1, e2));         // orbital normal
    float bPhi = 0.0;                                     // filled below

    float lapse = sqrt(max(1.0 - 2.0 * u0, 1.0e-4));
    float b = r0 * sinPsi / lapse;
    bPhi = b * dot(planeNormal, DISK_AXIS);

    float u = u0;
    float du = -cosPsi / b;

    vec3 radial = e1;
    vec3 position = origin;

    // Rotation by one fixed phase step, about the orbital normal.
    float cs = cos(uStepSize);
    float sn = sin(uStepSize);

    vec3 accum = vec3(0.0);
    float transmittance = 1.0;
    vec3 escapeDir = dir;

    for (int i = 0; i < uMaxSteps; ++i) {
        // ---- RK4 on u'' = 3u^2 - u -------------------------------------
        float k1u = du;
        float k1d = 3.0 * u * u - u;
        float k2u = du + 0.5 * uStepSize * k1d;
        float u2 = u + 0.5 * uStepSize * k1u;
        float k2d = 3.0 * u2 * u2 - u2;
        float k3u = du + 0.5 * uStepSize * k2d;
        float u3 = u + 0.5 * uStepSize * k2u;
        float k3d = 3.0 * u3 * u3 - u3;
        float k4u = du + uStepSize * k3d;
        float u4 = u + uStepSize * k3u;
        float k4d = 3.0 * u4 * u4 - u4;

        float un = u + (uStepSize / 6.0) * (k1u + 2.0 * k2u + 2.0 * k3u + k4u);
        float dun = du + (uStepSize / 6.0) * (k1d + 2.0 * k2d + 2.0 * k3d + k4d);

        // ---- advance the 3D position ------------------------------------
        vec3 nextRadial = radial * cs + cross(planeNormal, radial) * sn;
        vec3 nextPosition = nextRadial * (1.0 / un);

        // ---- event horizon ----------------------------------------------
        if (un > 0.5) {                       // r < 2 r_g: captured
            transmittance = 0.0;
            break;
        }
        if (un <= 0.0) {                      // radial escape overshot infinity
            escapeDir = nextRadial;
            break;
        }

        // ---- accretion disk ---------------------------------------------
        if (uDiskVisible > 0.5 && transmittance > 0.002) {
            accumulateDisk(accum, transmittance, position, nextPosition, bPhi);
        }

        u = un;
        du = dun;
        radial = nextRadial;
        position = nextPosition;

        // ---- escape to the background -----------------------------------
        if (un <= 1.0 / ESCAPE_RADIUS && dun < 0.0) {
            // cos(psi) = -b du, sin(psi) = b u sqrt(1 - 2u); normalising the
            // result removes the coordinate-frame scaling.
            escapeDir = normalize(radial * (-b * du) + e2 * (b * u * sqrt(max(1.0 - 2.0 * u, 0.0))));
            break;
        }
    }

    vec3 color = accum;
    if (transmittance > 0.002) color += transmittance * sampleSky(escapeDir);
    return color;
}

// ---------------------------------------------------------------------------
//  Main
// ---------------------------------------------------------------------------
void main() {
    vec3 color = vec3(0.0);
    int count = max(uAASamples, 1);

    for (int s = 0; s < 4; ++s) {
        if (s >= count) break;
        vec2 pixel = gl_FragCoord.xy + aaOffset(s, count);
        vec2 ndc = (pixel - 0.5 * uResolution) / (0.5 * uResolution.y);
        vec3 dir = normalize(uCamForward + uCamRight * (ndc.x * uTanHalfFov) +
                             uCamUp * (ndc.y * uTanHalfFov));
        color += traceCameraRay(uCamPos, dir);
    }
    color /= float(count);

    finalColor = vec4(color, 1.0);
}
