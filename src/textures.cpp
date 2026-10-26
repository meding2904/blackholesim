#include "textures.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace bh {
namespace {

constexpr int kNoiseSize = 512;

uint32_t hash32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

float hash2i(int x, int y, uint32_t seed) {
    const uint32_t h = hash32(static_cast<uint32_t>(x) * 73856093u ^
                              static_cast<uint32_t>(y) * 19349663u ^ seed);
    return static_cast<float>(h & 0xFFFFFFu) * (1.0f / 16777216.0f);
}

float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

// Tileable value noise in [0,1); period is the lattice size in cells.
float valueNoiseTile(float x, float y, int period, uint32_t seed) {
    const int xi = static_cast<int>(std::floor(x));
    const int yi = static_cast<int>(std::floor(y));
    const float fx = smoothstep01(x - static_cast<float>(xi));
    const float fy = smoothstep01(y - static_cast<float>(yi));

    const auto wrap = [period](int v) { return ((v % period) + period) % period; };
    const float a = hash2i(wrap(xi), wrap(yi), seed);
    const float b = hash2i(wrap(xi + 1), wrap(yi), seed);
    const float c = hash2i(wrap(xi), wrap(yi + 1), seed);
    const float d = hash2i(wrap(xi + 1), wrap(yi + 1), seed);

    const float top = a + (b - a) * fx;
    const float bottom = c + (d - c) * fx;
    return top + (bottom - top) * fy;
}

// uv in [0,1); tiles seamlessly on both axes.
float fbmTile(float u, float v, int basePeriod, int octaves, uint32_t seed) {
    float sum = 0.0f;
    float amplitude = 0.5f;
    float total = 0.0f;
    int period = basePeriod;
    for (int o = 0; o < octaves; ++o) {
        sum += amplitude * valueNoiseTile(u * static_cast<float>(period),
                                          v * static_cast<float>(period), period,
                                          seed + static_cast<uint32_t>(o) * 131u);
        total += amplitude;
        amplitude *= 0.5f;
        period *= 2;
    }
    return sum / std::max(total, 1e-6f);
}

}  // namespace

Texture2D createDiskNoiseTexture() {
    std::vector<unsigned char> pixels(static_cast<size_t>(kNoiseSize) * kNoiseSize * 4, 0);

    // Four decorrelated fractal channels at different base frequencies.
    const int periods[4] = {6, 10, 18, 4};
    const int octaves[4] = {4, 4, 4, 3};
    const uint32_t seeds[4] = {0x1A2B3Cu, 0x4D5E6Fu, 0x718293u, 0xA4B5C6u};
    // Contrast stretch per channel so the disk pattern has usable dynamic range.
    const float gains[4] = {1.7f, 1.9f, 2.1f, 1.5f};
    const float biases[4] = {-0.32f, -0.42f, -0.52f, -0.22f};

    for (int y = 0; y < kNoiseSize; ++y) {
        const float v = (y + 0.5f) / kNoiseSize;
        for (int x = 0; x < kNoiseSize; ++x) {
            const float u = (x + 0.5f) / kNoiseSize;
            const size_t idx = (static_cast<size_t>(y) * kNoiseSize + x) * 4;
            for (int c = 0; c < 4; ++c) {
                const float n = fbmTile(u, v, periods[c], octaves[c], seeds[c]);
                const float value = std::clamp(n * gains[c] + biases[c], 0.0f, 1.0f);
                pixels[idx + c] = static_cast<unsigned char>(value * 255.0f + 0.5f);
            }
        }
    }

    Image image = {};
    image.data = pixels.data();
    image.width = kNoiseSize;
    image.height = kNoiseSize;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

    Texture2D texture = LoadTextureFromImage(image);
    SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    GenTextureMipmaps(&texture);
    // Trilinear: the shader picks an explicit LOD per sample, and blending the
    // two nearest levels keeps the pattern from stepping between mips as the
    // footprint changes across the disk (which reads as flat bands that pop
    // while the disk rotates).
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
    return texture;
}

}  // namespace bh
