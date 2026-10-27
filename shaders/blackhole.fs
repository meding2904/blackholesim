#version 330

// ===========================================================================
//  Kerr black hole: per-pixel backward ray tracing of null geodesics
//
//  Units: G = c = 1, distances in gravitational radii r_g = GM/c^2 (M = 1).
//      a = 0: horizon 2, photon sphere 3, ISCO 6, b_crit = 3 sqrt(3)
//
//  Geodesics are integrated from the Carter-separated first-order system in
//  Boyer-Lindquist coordinates (see the Kerr geometry section below), which
//  reduces exactly to the planar Binet equation d^2u/dphi^2 + u = 3u^2 as a -> 0.
//  Captured rays render black; escaping rays sample the star field along their
//  asymptotic direction. The accretion disk is sampled as a finite-thickness
//  emissive/absorbing slab each time the ray passes through the equatorial
//  plane, so multiple lensed images accumulate naturally.
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
uniform float uStepSize;      // relative affine step h: dl = h * r
uniform float uSpin;          // dimensionless spin a/M
uniform float uHorizon;       // r+ = 1 + sqrt(1 - a^2)
uniform int   uMaxSteps;
uniform int   uAASamples;
uniform float uStarGain;
uniform float uPhotonPro;     // prograde photon orbit r_ph(a): the capture threshold
uniform sampler2D uNoise;     // tileable procedural turbulence
uniform samplerCube uSky;     // baked deep field: faint stars, band, dust
uniform float uSkyTexel;      // radians subtended by one deep-field texel at mip 0

const float PI = 3.14159265359;
const float ESCAPE_RADIUS = 900.0;

// Far-field radial ruler. Past this radius, once a ray is moving outward and its
// radial potential is still positive at twice the radius (so no turning point is
// within reach), the radial ruler is allowed to grow: the residual bending out
// there is small enough that a six times longer step leaves the escape direction
// where a converged integration puts it (measured: 0.01 px median, 1.2 px p99
// against a fine reference) while removing about a quarter of all steps. The
// angular and pole rulers still cap the step, so a perigee is resolved as finely
// as it was.
const float FAR_RADIUS = 60.0;
const float FAR_GROWTH = 6.0;

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

// ---------------------------------------------------------------------------
//  Sky: a prefiltered deep field plus analytic bright stars
//
//  The background carries two kinds of signal and they want opposite treatment:
//
//    * Unresolved light -- the galactic band, its dust lanes, nebulosity and
//      the star haze (the faint population's mean light). A pixel that cannot
//      resolve detail should
//      receive the *average* radiance of what it covers, which is exactly what
//      sampling a mip level returns. So it lives in the baked cube map, sampled
//      at the LOD matching the pixel's footprint on the sky. This is also what
//      fixes the tangential caustic: there the lens folds a long strip of sky
//      into one pixel, and a point-sampled star field turns that strip into
//      confetti, where a footprint-matched mip returns the strip's mean.
//
//    * Bright stars -- point sources, which must keep a constant angular size at
//      any magnification. One screen pixel already covers far less sky than a
//      texel of any affordable cube map, so they stay analytic: Gaussians one
//      pixel wide, widened (and dimmed by the same factor) as the lens
//      compresses the image, and faded into the strip mean once the strip
//      holds more stars than the lattice search can honestly resolve.
// ---------------------------------------------------------------------------

// 0..1 from a lattice cell id (photometry: magnitude and colour).
float hashCell(vec3 p) {
    vec3 q = fract(p * vec3(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z);
}

vec4 hashCell4(vec3 p) {
    vec4 q = vec4(p, 1.0);
    q = fract(q * vec4(0.1031, 0.1030, 0.0973, 0.1099));
    q += dot(q, q.wzyx + 33.33);
    return fract((q.xxyz + q.yzzw) * q.zywx);
}

// How much wider than one pixel a sky feature has to be drawn. wind (the
// azimuth a ray swept) stands in for the Jacobian of the lens map: it grows as
// the ray approaches the critical curve, which is where the compression is.
float skySpread(float wind) { return 1.0 + 0.9 * max(wind - 2.0, 0.0); }

// Unresolved sky, prefiltered for one pixel: the footprint is the pixel's
// angular size stretched by the lens compression. LOD below 0 would magnify the
// bake (and shimmer), so it is clamped at the base level. The bake stores the
// square root of radiance (the band is faint enough that 8-bit linear would
// quantise it into patches), so square it back here.
vec3 sampleDeepField(vec3 dir, float footprint) {
    float lod = log2(max(footprint, uSkyTexel) / uSkyTexel);
    vec3 encoded = textureLod(uSky, dir, clamp(lod, 0.0, 8.0)).rgb;
    return encoded * encoded;
}

vec3 sampleBrightStars(vec3 dir, float pixelAngle, float spread) {
    // 0.018 rad cells: a few thousand cells where the unit sphere crosses the
    // lattice, of which a few percent hold a star.
    const float CELL = 0.018;
    // The eight cells searched below share the lattice corner nearest the ray
    // and cover half a cell around it. The profile must stay inside that
    // margin: widen sigma past it and stars just outside the search silently
    // drop out while the lattice cuts each profile off at the block edge --
    // the square "confetti" that used to plague folded sky. So the widening
    // stops at SIGMA_MAX, where 3*SIGMA_MAX is exactly half a cell.
    const float SIGMA_MAX = CELL / 6.0;
    float sigmaWanted = 0.70 * pixelAngle * spread;
    float sigma = min(sigmaWanted, SIGMA_MAX);
    float inv2s2 = 1.0 / (2.0 * sigma * sigma);
    float cutoff = 9.0 * sigma * sigma;
    float flux = 1.0 / (spread * spread);   // widening must not add light

    // Once the footprint grows past twice the cap, one pixel covers a strip of
    // sky holding several stars and no single profile is meaningful: the
    // honest answer is their average, which the deep field already carries
    // (its star haze, sampled at the same footprint). Cross-fade the discrete
    // layer into that mean -- the brightness stays correct and the folded sky
    // reads as smooth lensed glow instead of clipped blobs.
    float resolve = 1.0 - smoothstep(1.0, 2.0, sigmaWanted / SIGMA_MAX);

    // Only the cells that share the lattice corner nearest the ray can hold a
    // star within the cutoff (3 sigma is at most half a cell), so those eight
    // are the entire search. Testing the 3x3x3 neighbourhood of the corner
    // instead is 3.4x the work for the same image.
    vec3 cell = dir / CELL;
    vec3 base = floor(cell);
    vec3 corner = base + step(vec3(0.5), cell - base);
    vec3 sum = vec3(0.0);
    for (int i = 0; i < 8; ++i) {
        vec3 id = corner - vec3(float(i & 1), float((i >> 1) & 1), float((i >> 2) & 1));
        vec4 h = hashCell4(id);
        if (h.w > 0.04) continue;              // most cells are empty sky
        vec3 star = normalize(id + h.xyz);
        float sep2 = max(2.0 - 2.0 * dot(dir, star), 0.0);   // ~angle^2
        if (sep2 > cutoff) continue;

        float phot = hashCell(id);
        float mag = phot * phot * phot * phot * phot;   // a few bright, many faint
        float kelvin = 4200.0 + 12500.0 * pow(fract(phot * 37.0), 0.9);
        vec3 tint = blackbodyRGB(kelvin);
        tint /= max(max(tint.r, tint.g), max(tint.b, 0.001));
        sum += tint * (exp(-sep2 * inv2s2) * (0.010 + 0.55 * mag) * flux);
    }

    // Mean radiance of the lattice: 0.04/CELL^2 stars per steradian, each with
    // mean flux 2*pi*(0.7*pixelAngle)^2 * (0.010 + 0.55/6). The cross-fade
    // target, and what a folded strip converges to.
    float starMean = 0.04 * 6.2831853 * 0.49 * pixelAngle * pixelAngle *
                     (0.010 + 0.55 / 6.0) / (CELL * CELL);
    return mix(vec3(starMean), sum, resolve);
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
    // 512 * scale texels per r_g. A bias of one level is deliberate: a mip
    // chosen so the footprint is exactly one texel still leaves detail at the
    // Nyquist limit, which reads as per-pixel grain and seethes once the pattern
    // is advected. One extra level costs nothing visible and removes the churn.
    const float size = 512.0;
    const float lodBias = 1.0;
    float lod1 = max(log2(max(footprint * 0.090 * size, 1.0)) + lodBias, 0.0);
    float lod2 = max(log2(max(footprint * 0.240 * size, 1.0)) + lodBias, 0.0);
    float lod3 = max(log2(max(footprint * 0.620 * size, 1.0)) + lodBias, 0.0);

    float n1 = textureLod(uNoise, p * 0.090 + vec2(0.11, 0.53), lod1).r;
    float n2 = textureLod(uNoise, p * 0.240 + vec2(0.37, 0.23), lod2).g;
    float n3 = textureLod(uNoise, p * 0.620 + vec2(0.71, 0.89), lod3).b;

    float n = n1 * 0.50 + n2 * 0.32 + n3 * 0.18;
    n = clamp((n - 0.5) * 1.40 + 0.5, 0.0, 1.0);      // push the contrast
    return n * n * (3.0 - 2.0 * n);
}

// ---------------------------------------------------------------------------
//  Kerr geometry (M = 1, r_g = 1). Boyer-Lindquist with the spin axis on +Y.
//
//  Rays are not planar once a != 0, so instead of one Binet equation the motion
//  is integrated from the Carter-separated first-order system:
//      Sigma dr/dl   = sr sqrt(R(r))
//      Sigma dth/dl  = sc sqrt(Theta(theta))
//      Sigma dphi/dl = [2 a r E + L (Sigma - 2r)/sin^2(theta)] / Delta
//  with E, L and the Carter constant Q exact for each ray. The polar state is
//  the unit pair (cos theta, sin theta), advanced by the exact identities
//  d(cs, sn)/dl = dth/dl * (-sn, cs): no transcendentals in the loop, one
//  inverse square root per step to renormalise, and -- unlike a u = cos(theta)
//  state -- full relative precision in sin^2(theta) near the pole, where the
//  L/sin^2(theta) spike in dphi/dl carries the entire ~pi sweep of a near-axis
//  ray. In float32, u = cos(theta) cannot even resolve that region: sin^2 falls
//  below the ulp of u near 1 and the sweep integrates as noise (the torn seam
//  and shimmer along the projected spin axis).
// ---------------------------------------------------------------------------
float kerrSigma(float r, float cu, float a) { return r * r + a * a * cu * cu; }
float kerrDelta(float r, float a) { return r * r - 2.0 * r + a * a; }

// Ray constants of the two potentials. They depend only on (E, L, Q, a), so the
// inner loop evaluates them once per ray instead of once per RK4 stage.
struct KerrConst {
    float a2;   // a^2
    float ea;   // a*E
    float carter;  // (L - aE)^2 + Q
    float a2E2;    // a^2 E^2
    float l2;      // L^2
};

KerrConst kerrConstants(float E, float L, float Q, float a) {
    KerrConst c;
    c.a2 = a * a;
    c.ea = a * E;
    c.carter = (L - a * E) * (L - a * E) + Q;
    c.a2E2 = c.a2 * E * E;
    c.l2 = L * L;
    return c;
}

// Radial centrifugal barrier: R(r) >= 0 is required for the ray to exist.
float kerrRadial(KerrConst c, float E, float L, float r) {
    float tr = E * (r * r + c.a2) - c.ea;
    float dl = r * r - 2.0 * r + c.a2;
    return tr * tr - dl * c.carter;
}

// (Sigma dtheta/dl)^2 = Theta(theta) = Q + a^2 E^2 cos^2 - L^2 cos^2/sin^2.
// Negative inside the polar forbidden zone |theta| < thetaT, where the ray has
// reached its turning point and must bounce.
float kerrPolarTheta(KerrConst c, float Q, float cu, float sn) {
    float s2 = max(sn * sn, 1.0e-20);
    float u2 = cu * cu;
    return Q + c.a2E2 * u2 - c.l2 * u2 / s2;
}

// Polar turning point. A ray with L != 0 bounces at |theta| = thetaT and can
// never enter the zone below it. sin^2(thetaT) is the small positive root of
// Theta = 0 in a cancellation-free form that keeps full float32 precision for
// tiny thetaT:
//   sin^2(thetaT) = 2 L^2 / (a^2 E^2 + Q + L^2 + sqrt(B^2 + 4 a^2 E^2 Q)),
//   B = a^2 E^2 - Q - L^2.
// The mirror through the turning point is the double-angle reflection
// theta -> 2 thetaT - theta (north) or 2 (pi - thetaT) - theta (south),
// expressed on (cos, sin) with C2 = cos(2 thetaT), S2 = sin(2 thetaT).
void kerrPolarTurn(float E, float L, float Q, float a, out float snT, out float C2,
                   out float S2) {
    float A = a * a * E * E;
    float B = A - Q - L * L;
    float s = 2.0 * L * L /
              max(A + Q + L * L + sqrt(max(B * B + 4.0 * A * Q, 0.0)), 1.0e-20);
    s = clamp(s, 0.0, 1.0);
    snT = sqrt(s);
    C2 = 1.0 - 2.0 * s;
    S2 = 2.0 * snT * sqrt(max(1.0 - s, 0.0));
}

void kerrDeriv(float r, float cu, float sn, KerrConst kc, float E, float L, float Q, float a,
               float sr, float sc, out float dr, out float dth, out float dph) {
    float sigma = kerrSigma(r, cu, a);
    float delta = kerrDelta(r, a);
    float R = max(kerrRadial(kc, E, L, r), 0.0);
    float T = max(kerrPolarTheta(kc, Q, cu, sn), 0.0);
    float sin2 = max(sn * sn, 1.0e-20);
    dr = sr * sqrt(R) / sigma;
    dth = sc * sqrt(T) / sigma;
    dph = (2.0 * a * r * E + L * (sigma - 2.0 * r) / sin2) / (sigma * delta);
}

// Camera ray -> conserved (E, L, Q). The camera is a static observer: its
// orthonormal frame gives k = e_t + n_r e_r + n_theta e_theta + n_phi e_phi,
// which is the a = 0 construction generalised to Kerr.
void kerrRayInit(vec3 pos, vec3 dir, float a, out float r, out float cu, out float sn,
                 out float ph, out float E, out float L, out float Q, out float sr,
                 out float sc) {
    r = length(pos);
    cu = clamp(pos.y / max(r, 1.0e-4), -1.0, 1.0);
    float sinT = sqrt(max(1.0 - cu * cu, 1.0e-8));
    sn = sinT;
    ph = atan(pos.z, pos.x);
    float cp = cos(ph);
    float sp = sin(ph);
    vec3 rhat = vec3(sinT * cp, cu, sinT * sp);
    vec3 that = vec3(cu * cp, -sinT, cu * sp);
    vec3 phat = vec3(-sp, 0.0, cp);
    float nr = dot(dir, rhat);
    float nt = dot(dir, that);
    float np = dot(dir, phat);

    float sigma = r * r + a * a * cu * cu;
    float delta = r * r - 2.0 * r + a * a;
    float sin2 = max(sinT * sinT, 1.0e-6);
    float gtt = -(1.0 - 2.0 * r / sigma);
    float gtphi = -2.0 * a * r * sin2 / sigma;
    float gphiphi = (r * r + a * a + 2.0 * a * a * r * sin2 / sigma) * sin2;
    float beta = inversesqrt(max(gphiphi - gtphi * gtphi / gtt, 1.0e-9));
    float alphaT = -(gtphi / gtt) * beta;

    float kt = inversesqrt(max(-gtt, 1.0e-6)) + np * alphaT;
    float kr = nr * sqrt(max(delta / sigma, 0.0));
    float kth = nt * inversesqrt(max(sigma, 1.0e-6));
    float kphi = np * beta;

    E = -(gtt * kt + gtphi * kphi);
    L = gtphi * kt + gphiphi * kphi;
    float pth = sigma * kth;
    Q = pth * pth + cu * cu * (L * L / sin2 - a * a * E * E);
    sr = (kr >= 0.0) ? 1.0 : -1.0;
    sc = (kth >= 0.0) ? 1.0 : -1.0;      // sign of dtheta/dl
}

// Emission shift from a circular prograde equatorial orbit:
//     g = 1 / (u^t (1 - Omega L/E)),  Omega = 1/(r^{3/2} + a)
//     u^t = (r^{3/2} + a) / (r^{3/4} sqrt(r^{3/2} - 3 r^{1/2} + 2a))
// The square root carries the gravitational and transverse Doppler redshift of
// the orbit, the denominator the line-of-sight Doppler boost.
float kerrOrbitShift(float r, float a, float lOverE) {
    float sq = sqrt(max(r, 1.0e-3));
    float omega = 1.0 / (r * sq + a);
    float ut = (r * sq + a) /
               (pow(max(r, 1.0e-3), 0.75) * sqrt(max(r * sq - 3.0 * sq + 2.0 * a, 1.0e-4)));
    return 1.0 / (ut * max(1.0 - omega * lOverE, 0.05));
}

// Emissive shading for one hit on the disk: either the equatorial sheet (the
// normal case) or the inner rim wall, which faces the hole radially.
void shadeDiskPoint(inout vec3 accum, inout float transmittance, float r, float phi,
                    float incidence, float edgeInner, KerrConst kc, float E, float L, float Q,
                    float a) {
    // (diskHits is bumped by the caller: a pixel only needs multi-sampling when
    // it actually gathered compressed disk detail, see main())
    float column = clamp(2.0 * diskHalfThickness(r) / incidence, 0.0, 24.0);

    float edgeIn = edgeInner * smoothstep(uDiskInner, uDiskInner * 1.18, r);
    float edgeOut = 1.0 - smoothstep(uDiskOuter * 0.58, uDiskOuter, r);
    // Screen footprint: distance x angular pixel size, stretched by the
    // incidence angle so grazing views do not alias.
    float hitDistance = length(vec3(r * cos(phi), 0.0, r * sin(phi)) - uCamPos);
    float footprint = hitDistance * (2.0 * uTanHalfFov / uResolution.y) / incidence;
    float turb = diskTurbulence(r, phi, footprint);

    float density = edgeIn * edgeOut * (0.14 + 1.05 * turb);
    float alpha = 1.0 - exp(-uDiskDensity * density * column);

    float tNorm = mix(0.78, temperatureNorm(r), smoothstep(uDiskInner, uDiskInner * 1.45, r)) *
                  (0.90 + 0.20 * turb);
    // The shift is unbounded on paper for a ray emitted exactly along the
    // orbital motion; a thin, non-limb-darkened disk cannot resolve that, so the
    // displayed range is bounded (documented approximation).
    float g = clamp(kerrOrbitShift(r, a, -L / max(E, 1.0e-4)), 0.08, 5.0);

    vec3 tint = pow(blackbodyRGB(tNorm * uTempScale * g), vec3(2.2));
    tint = mix(vec3(dot(tint, vec3(0.2126, 0.7152, 0.0722))), tint, 1.25);

    float brightness = uDiskGain * pow(tNorm, 3.4) * pow(g, 4.0);
    brightness *= 0.30 + 0.95 * turb;
    brightness *= 1.0 + 1.40 * exp(-(r - uDiskInner) * 0.50);   // hot inner rim

    accum += transmittance * tint * brightness * alpha;
    transmittance *= (1.0 - alpha);
}

// Equatorial crossing, found with a cubic Hermite root of cos(theta) inside the
// step. A linear root is off by O(dl^2) with dl of a few r_g, which walks the hit
// radius by a visible fraction of r_g from pixel to pixel.
void accumulateDisk(inout vec3 accum, inout float transmittance, float r0, float cu0,
                    float sn0, float ph0, float r1, float cu1, float sn1, float ph1, float dl,
                    KerrConst kc, float E, float L, float Q, float a, inout int diskHits) {
    float drA, dthA, dpA, drB, dthB, dpB;
    kerrDeriv(r0, cu0, sn0, kc, E, L, Q, a, 1.0, 1.0, drA, dthA, dpA);
    kerrDeriv(r1, cu1, sn1, kc, E, L, Q, a, 1.0, 1.0, drB, dthB, dpB);
    float mu0 = -sn0 * dthA * dl, mu1 = -sn1 * dthB * dl;   // d(cos)/dl * dl
    float mr0 = drA * dl, mr1 = drB * dl;
    float s = clamp(cu0 / (cu0 - cu1), 0.0, 1.0);
    for (int it = 0; it < 3; ++it) {
        float s2 = s * s, s3 = s2 * s;
        float cuS = (2.0 * s3 - 3.0 * s2 + 1.0) * cu0 + (s3 - 2.0 * s2 + s) * mu0 +
                    (-2.0 * s3 + 3.0 * s2) * cu1 + (s3 - s2) * mu1;
        float dcuS = (6.0 * s2 - 6.0 * s) * cu0 + (3.0 * s2 - 4.0 * s + 1.0) * mu0 +
                     (-6.0 * s2 + 6.0 * s) * cu1 + (3.0 * s2 - 2.0 * s) * mu1;
        if (abs(dcuS) < 1.0e-9) break;
        s = clamp(s - cuS / dcuS, 0.0, 1.0);
    }
    float s2 = s * s, s3 = s2 * s;
    float r = (2.0 * s3 - 3.0 * s2 + 1.0) * r0 + (s3 - 2.0 * s2 + s) * mr0 +
              (-2.0 * s3 + 3.0 * s2) * r1 + (s3 - s2) * mr1;
    if (r <= uDiskInner || r >= uDiskOuter) return;

    float dphi = mod(ph1 - ph0 + PI, 2.0 * PI) - PI;   // shortest way round
    float phi = ph0 + dphi * s;

    // Incidence: the disk normal is +Y = cos(theta) rhat - sin(theta) thetahat,
    // so on the equator |cos i| = |v_theta| / |v|.
    float dr, dth, dph;
    kerrDeriv(r, 0.0, 1.0, kc, E, L, Q, a, 1.0, 1.0, dr, dth, dph);
    float vTheta = r * dth;                      // sin(theta) = 1 here
    float vPhi = r * dph;
    float speed = sqrt(dr * dr + vTheta * vTheta + vPhi * vPhi);
    float incidence = clamp(abs(vTheta) / max(speed, 1.0e-6), 0.02, 1.0);

    shadeDiskPoint(accum, transmittance, r, phi, incidence, 1.0, kc, E, L, Q, a);
    ++diskHits;
}

// Inner rim wall. The disk is a thin slab with a vertical edge at the ISCO, so a
// sightline that grazes the equatorial plane *inside* that radius meets the wall
// rather than passing through a hole. Without this the edge-on view shows a dark
// filament wherever the first crossing falls inside the disk.
void accumulateRim(inout vec3 accum, inout float transmittance, float r0, float ph0, float r1,
                   float ph1, float cu, KerrConst kc, float E, float L, float Q, float a,
                   inout int diskHits) {
    float slab = diskHalfThickness(uDiskInner) / max(uDiskInner, 1.0);
    if (abs(cu) > 3.0 * slab) return;   // above the rim height: it passes overhead

    float dphi = mod(ph1 - ph0 + PI, 2.0 * PI) - PI;
    float frac = clamp((uDiskInner - r0) / max(r1 - r0, 1.0e-6), 0.0, 1.0);
    float phi = ph0 + dphi * frac;

    float dr, dth, dph;
    kerrDeriv(uDiskInner, 0.0, 1.0, kc, E, L, Q, a, 1.0, 1.0, dr, dth, dph);
    float speed = sqrt(dr * dr + (uDiskInner * dph) * (uDiskInner * dph));
    // The wall faces radially, so its incidence is the radial velocity share.
    float incidence = clamp(abs(dr) / max(speed, 1.0e-6), 0.08, 1.0);
    shadeDiskPoint(accum, transmittance, uDiskInner * 1.12, phi, incidence, 1.0, kc, E, L, Q, a);
    ++diskHits;
}

// ---------------------------------------------------------------------------
//  Ray setup
// ---------------------------------------------------------------------------
// Sample offsets relative to the pixel centre. Nested, so that refining a pixel
// from one sample to four keeps the first sample where it was.
vec2 aaOffset(int index, int count) {
    if (count >= 4) {
        return vec2(((index == 0 || index == 3) ? -0.25 : 0.25),
                    (index < 2) ? -0.25 : 0.25);
    }
    if (count == 2) {
        return (index == 0) ? vec2(-0.25, -0.25) : vec2(0.25, 0.25);
    }
    return vec2(-0.25);
}

vec3 rayDirection(vec2 pixel) {
    vec2 ndc = (pixel - 0.5 * uResolution) / (0.5 * uResolution.y);
    return normalize(uCamForward + uCamRight * (ndc.x * uTanHalfFov) +
                     uCamUp * (ndc.y * uTanHalfFov));
}

// wind: total azimuth swept, which is small for a ray that goes straight to the
// background or to the near side of the disk, and large once it wraps the hole.
vec3 traceCameraRay(vec3 origin, vec3 dir, out float wind, out int diskHits) {
    diskHits = 0;
    float r, cu, sn, phi, E, L, Q, sr, sc;
    kerrRayInit(origin, dir, uSpin, r, cu, sn, phi, E, L, Q, sr, sc);
    float phiStart = phi;
    // Polar turning point, a ray constant. snT = 0 exactly for L = 0, whose
    // geodesic crosses the axis instead of bouncing; that case must skip the
    // pole-proximity ruler below or it would stall at the axis.
    float snT, C2, S2;
    kerrPolarTurn(E, L, Q, uSpin, snT, C2, S2);
    bool polarGuard = snT > 1.0e-6;
    KerrConst kc = kerrConstants(E, L, Q, uSpin);

    vec3 accum = vec3(0.0);
    float transmittance = 1.0;
    vec3 escapeDir = dir;
    bool escaped = false;

    for (int i = 0; i < uMaxSteps; ++i) {
        // Two step rulers, whichever is stricter: geometric (dl = h r, which
        // strides through the nearly straight far field and stays finite at a
        // turning point) and angular (bounded sweep per step, which is what
        // actually resolves the bending near the hole).
        // Stage 1 doubles as the step ruler: same point, same derivative.
        float k1r, k1th, k1p, k2r, k2th, k2p, k3r, k3th, k3p, k4r, k4th, k4p;
        kerrDeriv(r, cu, sn, kc, E, L, Q, uSpin, sr, sc, k1r, k1th, k1p);
        // Radial ruler, with the far-field growth folded in before the caps below
        // so that the angular and pole rulers keep their authority: the growth
        // only ever lengthens a step that the geometry already allowed.
        float radial = uStepSize * max(r, 1.0);
        if (sr > 0.0 && r > FAR_RADIUS && kerrRadial(kc, E, L, 2.0 * r) > 0.0) {
            radial = max(radial, FAR_GROWTH * uStepSize * r);
        }
        float rate = max(abs(k1p), abs(k1th));
        float dl = radial;
        if (rate > 1.0e-9) dl = min(dl, uStepSize * 0.6 / rate);
        // Pole-proximity ruler: keep the theta travel a quarter of the distance
        // to the pole, so the L/sin^2(theta) spike in dphi cannot change by
        // orders of magnitude inside one RK4 step. That spike carries the whole
        // ~pi sweep of a near-axis ray; a step that leaps into it samples it in
        // one stage only and integrates it as garbage (star confetti along the
        // seam). Inside the spike the angular ruler above takes over.
        if (polarGuard && abs(k1th) > 1.0e-9) dl = min(dl, 0.25 * sn / abs(k1th));

        // RK4 on (r, cos theta, sin theta, phi). The polar pair advances by the
        // exact identity d(cs, sn)/dl = dth/dl * (-sn, cs).
        float k1cs = -sn * k1th, k1sn = cu * k1th;
        float cs2 = cu + 0.5 * dl * k1cs, sn2 = sn + 0.5 * dl * k1sn;
        kerrDeriv(r + 0.5 * dl * k1r, cs2, sn2, kc, E, L, Q, uSpin, sr, sc, k2r, k2th, k2p);
        float k2cs = -sn2 * k2th, k2sn = cs2 * k2th;
        float cs3 = cu + 0.5 * dl * k2cs, sn3 = sn + 0.5 * dl * k2sn;
        kerrDeriv(r + 0.5 * dl * k2r, cs3, sn3, kc, E, L, Q, uSpin, sr, sc, k3r, k3th, k3p);
        float k3cs = -sn3 * k3th, k3sn = cs3 * k3th;
        float cs4 = cu + dl * k3cs, sn4 = sn + dl * k3sn;
        kerrDeriv(r + dl * k3r, cs4, sn4, kc, E, L, Q, uSpin, sr, sc, k4r, k4th, k4p);
        float k4cs = -sn4 * k4th, k4sn = cs4 * k4th;

        float rn = r + (dl / 6.0) * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
        float cn = cu + (dl / 6.0) * (k1cs + 2.0 * k2cs + 2.0 * k3cs + k4cs);
        float snn = sn + (dl / 6.0) * (k1sn + 2.0 * k2sn + 2.0 * k3sn + k4sn);
        float pn = phi + (dl / 6.0) * (k1p + 2.0 * k2p + 2.0 * k3p + k4p);

        // Disk crossing, tested on the raw step end before any mirror: a
        // grazing ray that overshoots the lower polar turning point would
        // otherwise be reflected back to the positive side and lose its hit.
        bool crossed = (cu > 0.0) != (cn > 0.0);
        if (uDiskVisible > 0.5 && transmittance > 0.002) {
            if (crossed) {
                accumulateDisk(accum, transmittance, r, cu, sn, phi, rn, cn, snn, pn, dl, kc,
                               E, L, Q, uSpin, diskHits);
            } else if ((r < uDiskInner) != (rn < uDiskInner)) {
                accumulateRim(accum, transmittance, r, phi, rn, pn, cu, kc, E, L, Q, uSpin,
                              diskHits);
            }
            // An opaque disk ends the march: no later image of the disk and no
            // piece of sky can contribute through it any more.
            if (transmittance < 0.002) break;
        }

        // Turning points: mirror through the root and flip the branch. R depends
        // only on r and Theta only on cos(theta), so each test is independent.
        if (kerrRadial(kc, E, L, rn) < 0.0) {
            // Newton on R(r) = 0 so the mirror is about the turning radius itself.
            float rt = 0.5 * (r + rn);
            float slope = 4.0 * E * rt * (E * (rt * rt + kc.a2) - kc.ea) -
                          (2.0 * rt - 2.0) * kc.carter;
            if (abs(slope) > 1.0e-9) {
                rt = clamp(rt - kerrRadial(kc, E, L, rt) / slope, min(r, rn), max(r, rn));
            }
            rn = 2.0 * rt - rn;
            sr = -sr;
        }
        if (kerrPolarTheta(kc, Q, cn, snn) < 0.0) {
            // Mirror through the polar turning point (the double-angle
            // reflection on the unit pair) and flip the branch. L = 0 keeps
            // Theta > 0 and never enters this branch: that ray crosses the axis
            // freely, which is its correct continuation.
            float cn0 = cn;
            if (cn >= 0.0) {
                cn = C2 * cn + S2 * snn;
                snn = S2 * cn0 - C2 * snn;
            } else {
                cn = C2 * cn - S2 * snn;
                snn = -S2 * cn0 - C2 * snn;
            }
            sc = -sc;
        }
        float nrm = inversesqrt(max(cn * cn + snn * snn, 1.0e-12));
        cn *= nrm;
        snn *= nrm;

        r = rn;
        cu = cn;
        sn = snn;
        phi = pn;

        // Capture theorem: a ray inside the prograde photon orbit and moving
        // inward has its perigee below the smallest radius a photon can turn at,
        // so it is captured -- and provably captured *here*, not eventually.
        // Without this the dphi/dL spike as Delta -> 0 shrinks the angular ruler
        // towards zero and a plunging ray creeps until its budget runs out: that
        // used to cost a black shadow pixel its full 1200 steps.
        if (sr < 0.0 && r <= uPhotonPro) {
            transmittance = 0.0;
            break;
        }

        // Event horizon: Delta < 0 is exactly the region between the horizons.
        if (r <= uHorizon || kerrDelta(r, uSpin) <= 0.0) {
            transmittance = 0.0;
            break;
        }

        // Escape to the background: far away and moving outward.
        if (r >= ESCAPE_RADIUS) {
            float dr, dth, dph;
            kerrDeriv(r, cu, sn, kc, E, L, Q, uSpin, sr, sc, dr, dth, dph);
            if (dr > 0.0) {
                float cp = cos(phi);
                float sp = sin(phi);
                vec3 rhat = vec3(sn * cp, cu, sn * sp);
                vec3 that = vec3(cu * cp, -sn, cu * sp);
                vec3 phat = vec3(-sp, 0.0, cp);
                escapeDir = normalize(rhat * dr + that * (r * dth) + phat * (r * sn * dph));
                escaped = true;
                break;
            }
        }
    }

    wind = abs(phi - phiStart);

    // A ray that burns its whole step budget is still winding near the critical
    // curve: treat it as captured rather than sampling the sky from a direction
    // it never reached. Without this the shadow edge speckles with stars.
    if (!escaped) transmittance = 0.0;

    vec3 color = accum;
    if (transmittance > 0.002) {
        float pixelAngle = 2.0 * uTanHalfFov / uResolution.y;
        float spread = skySpread(wind);
        color += transmittance * uStarGain *
                 (sampleDeepField(escapeDir, pixelAngle * spread) +
                  sampleBrightStars(escapeDir, pixelAngle, spread));
    }
    return color;
}

// ---------------------------------------------------------------------------
//  Main
// ---------------------------------------------------------------------------
void main() {
    int count = clamp(uAASamples, 1, 4);
    vec2 base = gl_FragCoord.xy;

    // Sub-pixel dither. Rays lying exactly in the plane through the hole, the
    // camera and the spin axis (the centre column) are a symmetry set: with no
    // angular momentum about the axis they plunge where their neighbours wind,
    // which leaves a one-pixel seam through the disk. A per-pixel offset breaks
    // the alignment, so the seam is averaged away instead of being coherent.
    vec2 dither = vec2(hash12(base), hash12(base + vec2(19.19, 7.77))) - 0.5;
    vec2 origin = base + dither * 0.7;

    float wind = 0.0;
    int diskHits = 0;
    vec3 color =
        traceCameraRay(uCamPos, rayDirection(origin + aaOffset(0, count)), wind, diskHits);

    // A ray that has wound half a turn or more returns through a strongly
    // compressed image: one pixel then spans many r_g of disk, temperature and
    // Doppler factor included, so a single sample picks one of them and the
    // result speckles. Refine those pixels only, and only when the compressed
    // image is the *disk*: folded sky is handled by the deep-field LOD and the
    // widened star profiles, and a captured ray is black whichever way it is
    // sampled. Gating on diskHits keeps the four rays where they buy something:
    // at Balanced the old rule refined 42% of pixels, this one 16%.
    int total = (wind > 3.0 && diskHits > 0 && count < 4) ? 4 : count;
    for (int s = 1; s < 4; ++s) {
        if (s >= total) break;
        color += traceCameraRay(uCamPos, rayDirection(origin + aaOffset(s, total)), wind, diskHits);
    }
    color /= float(total);

    finalColor = vec4(color, 1.0);
}
