#version 330

// Final composite: HDR scene + bloom pyramid, ACES tone mapping, gamma and a
// subtle vignette / dither so gradients do not band.
//
// UVs come from gl_FragCoord because raylib's DrawRectangle() supplies texture
// atlas coordinates rather than the 0..1 quad coordinates.
in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D uScene;
uniform sampler2D uBloomNear;
uniform sampler2D uBloomFar;
uniform vec2  uResolution;
uniform float uExposure;
uniform float uBloomStrength;

vec3 acesToneMap(vec3 x) {
    // Narkowicz ACES fit.
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

void main() {
    vec2 uv = gl_FragCoord.xy / uResolution;

    vec3 hdr = texture(uScene, uv).rgb;
    vec3 bloom = texture(uBloomNear, uv).rgb * 0.62 +
                 texture(uBloomFar, uv).rgb * 0.40;
    hdr += bloom * uBloomStrength;

    vec3 color = acesToneMap(hdr * uExposure);

    // Gentle vignette keeps the eye on the shadow.
    vec2 q = uv - 0.5;
    float vignette = 1.0 - 0.42 * dot(q, q) * 1.6;
    color *= clamp(vignette, 0.0, 1.0);

    color = pow(max(color, 0.0), vec3(1.0 / 2.2));

    // ~1/255 ordered dither to break up 8-bit banding in the dark sky.
    color += (hash12(gl_FragCoord.xy) - 0.5) / 255.0;

    finalColor = vec4(color, 1.0);
}
