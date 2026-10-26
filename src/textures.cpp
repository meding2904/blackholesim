#include "textures.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace bh {
namespace {

constexpr int kSkyWidth = 4096;
constexpr int kSkyHeight = 2048;
constexpr int kNoiseSize = 512;

struct Rng {
    uint64_t state;
    explicit Rng(uint64_t seed) : state(seed) {}
    double next() {  // splitmix64 -> [0,1)
        uint64_t z = (state += 0x9e3779b97f4a7c15ull);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * (1.0 / 9007199254740992.0);
    }
};

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


// Same fit the shader uses so the star colours and the disk colours agree.
void blackbodyRGB(float kelvin, float& r, float& g, float& b) {
    const float t = std::clamp(kelvin, 1000.0f, 40000.0f) * 0.01f;
    if (t <= 66.0f) {
        r = 1.0f;
        g = std::clamp(0.9947080258f * std::log(t) - 1.6111956817f, 0.0f, 1.0f);
    } else {
        r = std::clamp(3.2969872745f * std::pow(t - 60.0f, -0.1332047592f), 0.0f, 1.0f);
        g = std::clamp(2.8812216953f * std::pow(t - 60.0f, -0.0755148492f), 0.0f, 1.0f);
    }
    if (t >= 66.0f) b = 1.0f;
    else if (t <= 19.0f) b = 0.0f;
    else b = std::clamp(1.3851773122f * std::log(t - 10.0f) - 3.0504479273f, 0.0f, 1.0f);
}

}  // namespace

Texture2D createSkyTexture() {
    std::vector<unsigned char> pixels(static_cast<size_t>(kSkyWidth) * kSkyHeight * 4, 0);

    // The sky is a pure star field: any faint smooth nebula gradient bands badly
    // once quantised to 8 bits, because the display curve lifts near-black values
    // ~9x. Black space plus stars keeps the background clean at any zoom.
    // ---- stars --------------------------------------------------------------
    Rng rng(0x5EEDB1ACull);
    constexpr int kStarCount = 22000;

    const auto addPixel = [&](int x, int y, float r, float g, float b, float a) {
        if (y < 0 || y >= kSkyHeight) return;
        x = ((x % kSkyWidth) + kSkyWidth) % kSkyWidth;
        const size_t idx = (static_cast<size_t>(y) * kSkyWidth + x) * 4;
        const auto add = [&](int channel, float value) {
            const float out = std::min(255.0f, pixels[idx + channel] + value);
            pixels[idx + channel] = static_cast<unsigned char>(out);
        };
        add(0, r * 255.0f);
        add(1, g * 255.0f);
        add(2, b * 255.0f);
        (void)a;
    };

    const auto splat = [&](float cx, float cy, float sigma, float r, float g, float b,
                           float amplitude) {
        const float sigmaU = std::max(sigma * std::sin(cy / kSkyHeight * 3.14159265f), 0.35f);
        const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0f)));
        const int px = static_cast<int>(std::lround(cx));
        const int py = static_cast<int>(std::lround(cy));
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                const float du = static_cast<float>(dx) / sigmaU;
                const float dv = static_cast<float>(dy) / sigma;
                const float weight = std::exp(-0.5f * (du * du + dv * dv));
                if (weight < 0.002f) continue;
                const float value = amplitude * weight;
                addPixel(px + dx, py + dy, r * value, g * value, b * value, value);
            }
        }
    };

    for (int s = 0; s < kStarCount; ++s) {
        const double v = (s + 0.5) / kStarCount;  // stratified in cos(theta)
        const double cosTheta = 1.0 - 2.0 * v;
        const double u = rng.next();
        const double brightnessSeed = rng.next();
        const double colorSeed = rng.next();

        const float cy = static_cast<float>(std::acos(std::clamp(cosTheta, -1.0, 1.0)) /
                                            3.14159265358979 * kSkyHeight);
        const float cx = static_cast<float>(u * kSkyWidth);

        // Power-law brightness: a few bright stars, many faint ones.
        const float magnitude = static_cast<float>(std::pow(brightnessSeed, 5.0));
        const float amplitude = 0.05f + 0.9f * magnitude;
        const float sigma = 0.28f + 0.35f * magnitude;

        const float kelvin = 2800.0f + 13500.0f * static_cast<float>(std::pow(colorSeed, 2.4));
        float r, g, b;
        blackbodyRGB(kelvin, r, g, b);
        const float maxChannel = std::max({r, g, b, 0.001f});
        r /= maxChannel;
        g /= maxChannel;
        b /= maxChannel;

        splat(cx, cy, sigma, r, g, b, amplitude);
        if (magnitude > 0.62f) {
            // Soft halo for the handful of brightest stars.
            splat(cx, cy, sigma * 3.2f, r, g, b, amplitude * 0.04f);
        }
    }

    Image image = {};
    image.data = pixels.data();
    image.width = kSkyWidth;
    image.height = kSkyHeight;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

    if (std::getenv("BH_DEBUG") != nullptr) {
        std::printf("sky gen sample: %d,%d,%d  %d,%d,%d  %d,%d,%d\n", pixels[0], pixels[1],
                    pixels[2], pixels[(static_cast<size_t>(kSkyHeight / 2) * kSkyWidth +
                                       kSkyWidth / 2) * 4],
                    pixels[(static_cast<size_t>(kSkyHeight / 2) * kSkyWidth + kSkyWidth / 2) * 4 + 1],
                    pixels[(static_cast<size_t>(kSkyHeight / 2) * kSkyWidth + kSkyWidth / 2) * 4 + 2],
                    pixels[4], pixels[5], pixels[6]);
        ExportImage(image, "build/sky_gen.png");
    }

    Texture2D texture = LoadTextureFromImage(image);
    SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);

    return texture;
}

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
    SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
    return texture;
}

}  // namespace bh
