#version 330

// Separable 9-tap Gaussian used by the bloom pyramid. The first pass also
// applies the bright-pass threshold, so only the hottest pixels glow.
//
// NOTE: raylib's DrawRectangle() provides atlas UVs from its shapes texture, not
// 0..1 quad UVs, so every pass derives its UVs from gl_FragCoord instead.
in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D uTex;
uniform vec2  uUVScale;    // 1 / target resolution
uniform vec2  uTexel;      // 1 / source resolution
uniform vec2  uDirection;  // (1,0) horizontal, (0,1) vertical
uniform float uRadius;     // spread in source texels
uniform float uThreshold;  // < 0 disables the bright pass

float luminance(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

vec3 fetch(vec2 uv) {
    vec3 c = texture(uTex, uv).rgb;
    if (uThreshold >= 0.0) {
        c *= smoothstep(uThreshold, uThreshold + 0.55, luminance(c));
    }
    return c;
}

void main() {
    vec2 uv = gl_FragCoord.xy * uUVScale;
    vec2 stepUV = uTexel * uDirection * uRadius;

    const float w[5] = float[](0.227027, 0.1945946, 0.1216216, 0.054054, 0.0162162);
    vec3 sum = fetch(uv) * w[0];
    for (int i = 1; i < 5; ++i) {
        float offset = float(i);
        sum += (fetch(uv - stepUV * offset) + fetch(uv + stepUV * offset)) * w[i];
    }

    finalColor = vec4(sum, 1.0);
}
