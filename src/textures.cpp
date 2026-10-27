#include "textures.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "rlgl.h"

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

// ---------------------------------------------------------------------------
//  Deep-field sky bake
//
//  Everything unresolved lives here: the galactic band, its dust lanes, two
//  emission nebulae and the star haze (the mean light of the faint star
//  population). No discrete stars: a texel-scale blob becomes a visible block
//  the moment the lens magnifies it, so every resolved star is analytic in the
//  shader and only their smooth average is baked. The point of baking instead
//  of evaluating per pixel is prefiltering: the mip chain is a low-pass of the
//  sky, so sampling it at the level that matches a pixel's footprint (which
//  near the critical curve is a large strip of sky) returns the average
//  radiance of that strip rather than one arbitrary feature out of it.
// ---------------------------------------------------------------------------

struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

Vec3f normalize3(Vec3f v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (!(len > 1e-12f)) return Vec3f{0.0f, 1.0f, 0.0f};
    const float inv = 1.0f / len;
    return Vec3f{v.x * inv, v.y * inv, v.z * inv};
}

float dot3(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3f mix3(Vec3f a, Vec3f b, float t) {
    return Vec3f{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

Vec3f scale3(Vec3f a, float s) { return Vec3f{a.x * s, a.y * s, a.z * s}; }

Vec3f add3(Vec3f a, Vec3f b) { return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z}; }

// Deterministic sequence: the bake must be reproducible byte for byte, so this
// is a hash chain rather than any library generator.


float hash3i(int x, int y, int z, uint32_t seed) {
    const uint32_t h = hash32(static_cast<uint32_t>(x) * 73856093u ^
                              static_cast<uint32_t>(y) * 19349663u ^
                              static_cast<uint32_t>(z) * 83492791u ^ seed);
    return static_cast<float>(h & 0xFFFFFFu) * (1.0f / 16777216.0f);
}

// Trilinear value noise on a 3D lattice. The sky field is a function of the
// direction rather than of 2D texture coordinates: an equirectangular field
// would pinch at the poles and a per-face field would seam at the cube edges.
float valueNoise3(float x, float y, float z, uint32_t seed) {
    const float fx0 = std::floor(x);
    const float fy0 = std::floor(y);
    const float fz0 = std::floor(z);
    const int xi = static_cast<int>(fx0);
    const int yi = static_cast<int>(fy0);
    const int zi = static_cast<int>(fz0);
    const float fx = smoothstep01(x - fx0);
    const float fy = smoothstep01(y - fy0);
    const float fz = smoothstep01(z - fz0);

    const auto corner = [&](int dx, int dy, int dz) {
        return hash3i(xi + dx, yi + dy, zi + dz, seed);
    };
    const float c000 = corner(0, 0, 0), c100 = corner(1, 0, 0);
    const float c010 = corner(0, 1, 0), c110 = corner(1, 1, 0);
    const float c001 = corner(0, 0, 1), c101 = corner(1, 0, 1);
    const float c011 = corner(0, 1, 1), c111 = corner(1, 1, 1);

    const float x00 = c000 + (c100 - c000) * fx;
    const float x10 = c010 + (c110 - c010) * fx;
    const float x01 = c001 + (c101 - c001) * fx;
    const float x11 = c011 + (c111 - c011) * fx;
    const float y0 = x00 + (x10 - x00) * fy;
    const float y1 = x01 + (x11 - x01) * fy;
    return y0 + (y1 - y0) * fz;
}

// Fractal noise in [0,1] over the sphere.
float fbm3(Vec3f d, float frequency, int octaves, uint32_t seed) {
    float sum = 0.0f;
    float amplitude = 0.5f;
    float total = 0.0f;
    float f = frequency;
    for (int o = 0; o < octaves; ++o) {
        sum += amplitude * valueNoise3(d.x * f, d.y * f, d.z * f,
                                       seed + static_cast<uint32_t>(o) * 131u);
        total += amplitude;
        amplitude *= 0.5f;
        f *= 2.04f;   // slightly off 2 so octave lattices do not line up
    }
    return sum / std::max(total, 1e-6f);
}

// Filaments (dust lanes): the ridged transform of the same noise, sharpened.
float ridge3(Vec3f d, float frequency, int octaves, uint32_t seed) {
    float sum = 0.0f;
    float amplitude = 0.5f;
    float total = 0.0f;
    float f = frequency;
    for (int o = 0; o < octaves; ++o) {
        const float n = valueNoise3(d.x * f, d.y * f, d.z * f,
                                    seed + static_cast<uint32_t>(o) * 977u);
        sum += amplitude * (1.0f - std::fabs(2.0f * n - 1.0f));
        total += amplitude;
        amplitude *= 0.5f;
        f *= 2.04f;
    }
    return sum / std::max(total, 1e-6f);
}

// OpenGL cube-map texel -> direction. Face order is
// +X, -X, +Y, -Y, +Z, -Z, u and v in [-1,1] over the face, v increasing with the
// row index (row 0 is t = 0, which is what glTexImage2D expects).
Vec3f faceDirection(int face, float u, float v) {
    switch (face) {
        case 0: return normalize3(Vec3f{1.0f, -v, -u});
        case 1: return normalize3(Vec3f{-1.0f, -v, u});
        case 2: return normalize3(Vec3f{u, 1.0f, v});
        case 3: return normalize3(Vec3f{u, -1.0f, -v});
        case 4: return normalize3(Vec3f{u, -v, 1.0f});
        default: return normalize3(Vec3f{-u, -v, -1.0f});
    }
}

// Galactic band: the "pole" of the plane, an arbitrary but fixed orientation, so
// the band sweeps through the frame as the camera orbits.
const Vec3f kBandPole = normalize3(Vec3f{0.31f, 0.90f, -0.30f});

struct SkySample {
    Vec3f rgb;    // linear radiance (the band core sits near 0.01)
    float dust;   // 0..1 lane depth
};

SkySample skyField(Vec3f d) {
    // Band profile: wide outer glow plus a narrow bright core.
    const float sinLat = dot3(d, kBandPole);
    // 0.17 rad (about 10 degrees) wide with a 3 degree core: a 15 degree band
    // fills a 50 degree frame edge to edge and stops reading as a band.
    const float bandWide = std::exp(-(sinLat * sinLat) / (2.0f * 0.17f * 0.17f));
    const float bandCore = std::exp(-(sinLat * sinLat) / (2.0f * 0.055f * 0.055f));

    // Two octaves each, and low frequencies: the diffuse field has to read as
    // clouds tens of degrees across. Higher octaves here mottle the whole frame
    // at 5-7 degrees, which looks like noise rather than sky, and the star
    // splatter supplies the fine texture anyway.
    const float clouds = fbm3(d, 2.2f, 2, 0x51C3A7u);   // large-scale cloud structure
    const float fine = fbm3(d, 4.5f, 2, 0x2A9F31u);     // unresolved star haze
    const float filaments = ridge3(d, 4.0f, 2, 0x77E1B5u);

    const float dust = smoothstep01(std::clamp((filaments - 0.40f) / 0.36f, 0.0f, 1.0f));
    const float lane = 1.0f - 0.80f * dust * bandWide;

    // Radiometric scale. These are linear radiances in the same units the shader
    // multiplies by uStarGain, on the way to ACES and a 2.2 gamma, so they are
    // much smaller than they look: 0.015 of band radiance already displays as
    // 30/255 (measured by inverting the composite curve). The band therefore has
    // to sit around 0.01 or it turns the whole frame grey instead of reading as a
    // faint milky glow on black.
    const float glow = 0.013f * bandWide * (0.45f + 0.75f * clouds) +
                       0.010f * bandCore * (0.25f + 1.25f * fine);
    // Faint isotropic floor of unresolved light away from the band.
    const float ambient = 0.0006f + 0.0018f * fine * fine;
    // Star haze: the mean light of the faint star population the shader draws
    // discretely. It lives here as a *smooth* field because folded pixels (the
    // lens has stretched one pixel across a strip of sky) cannot resolve
    // individual stars at all -- they must receive the strip's average. Keeping
    // that average in the prefiltered layer is also what lets the shader fade
    // its discrete stars out there without the fold going empty.
    const float haze = 0.0055f * (0.20f + 0.80f * bandWide) * (1.0f - 0.80f * dust);

    // Two emission nebulae, soft-edged and modulated so they read as clouds
    // rather than as discs. Subtraction is done on the cosine so it stays cheap.
    const auto blob = [](Vec3f p, Vec3f centre, float radius, float sharp) {
        const float x = (1.0f - std::clamp(dot3(p, centre), -1.0f, 1.0f)) / radius;
        return std::exp(-x * x * sharp);
    };
    const float nebulaA =
        blob(d, normalize3(Vec3f{-0.42f, -0.30f, 0.86f}), 0.55f, 2.2f) * (0.35f + 0.80f * clouds);
    const float nebulaB =
        blob(d, normalize3(Vec3f{0.63f, 0.26f, 0.73f}), 0.40f, 3.0f) * (0.30f + 0.90f * fine);

    const Vec3f warm{0.95f, 0.84f, 0.70f};
    const Vec3f cool{0.80f, 0.88f, 1.00f};
    const Vec3f bandColour = mix3(warm, cool, fine * 0.75f);
    const Vec3f emissionPink{0.72f, 0.32f, 0.46f};
    const Vec3f emissionTeal{0.30f, 0.50f, 0.58f};

    Vec3f rgb = add3(scale3(bandColour, glow * lane),
                     scale3(Vec3f{0.90f, 0.93f, 1.00f}, ambient));
    rgb = add3(rgb, scale3(Vec3f{0.95f, 0.95f, 1.00f}, haze));
    rgb = add3(rgb, scale3(emissionPink, 0.009f * nebulaA));
    rgb = add3(rgb, scale3(emissionTeal, 0.007f * nebulaB));
    return SkySample{rgb, dust};
}

// Storage encoding. The deep field's radiances are small (the band sits near
// 0.01), so storing them linearly in 8 bits leaves a couple of levels for the
// whole band and quantises it into visible patches. Storing the square root
// instead is nearly free (one multiply in the shader squares it back) and gives
// the faint end a relative precision instead of an absolute one: 0.0006 and 0.06
// both land with ~1% of themselves.
float skyEncode(float linear) {
    return std::sqrt(std::clamp(linear, 0.0f, 1.0f));
}

unsigned char skyByte(float linear) {
    return static_cast<unsigned char>(skyEncode(linear) * 255.0f + 0.5f);
}

// Box filter of one face into the next mip level. The mip chain is built here
// rather than by the GPU because prefiltering is the whole point of the bake:
// wide kernels (blocky box averages) are exactly the right filter for star light
// and are cheap enough to do once. Filtering happens on linear radiance and the
// encoding is applied afterwards, so a level is the mean of what a pixel would
// have covered.
void downsampleFace(const std::vector<float>& src, int srcSize, std::vector<float>& dst,
                    int dstSize) {
    dst.assign(static_cast<size_t>(dstSize) * dstSize * 3, 0.0f);
    const float inv = 1.0f / (2.0f * 2.0f);
    for (int y = 0; y < dstSize; ++y) {
        for (int x = 0; x < dstSize; ++x) {
            float r = 0.0f, g = 0.0f, b = 0.0f;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    const int sx = std::min(2 * x + dx, srcSize - 1);
                    const int sy = std::min(2 * y + dy, srcSize - 1);
                    const size_t si = (static_cast<size_t>(sy) * srcSize + sx) * 3;
                    r += src[si];
                    g += src[si + 1];
                    b += src[si + 2];
                }
            }
            const size_t di = (static_cast<size_t>(y) * dstSize + x) * 3;
            dst[di] = r * inv;
            dst[di + 1] = g * inv;
            dst[di + 2] = b * inv;
        }
    }
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

namespace {

size_t skyFaceOffset(int faceSize, int level, int faceIndex) {
    size_t offset = 0;
    int size = faceSize;
    for (int l = 0; l < level; ++l) {
        offset += static_cast<size_t>(6) * size * size * 4;
        size = std::max(1, size / 2);
    }
    return offset + static_cast<size_t>(faceIndex) * size * size * 4;
}

}  // namespace

const unsigned char* SkyBake::faceData(int level, int faceIndex) const {
    if (level < 0 || level >= mipCount || faceIndex < 0 || faceIndex >= 6) return nullptr;
    return pixels.data() + skyFaceOffset(faceSize, level, faceIndex);
}

unsigned char* SkyBake::faceData(int level, int faceIndex) {
    if (level < 0 || level >= mipCount || faceIndex < 0 || faceIndex >= 6) return nullptr;
    return pixels.data() + skyFaceOffset(faceSize, level, faceIndex);
}

SkyBake bakeSkyCubemap(int faceSize) {
    SkyBake bake;
    if (faceSize < 8) return bake;
    bake.faceSize = faceSize;
    bake.mipCount = 1;
    while ((faceSize >> bake.mipCount) > 0) ++bake.mipCount;

    size_t total = 0;
    for (int level = 0, size = faceSize; level < bake.mipCount; ++level, size /= 2) {
        total += static_cast<size_t>(6) * std::max(1, size) * std::max(1, size) * 4;
    }
    bake.pixels.assign(total, 0);

    const int cloudSize = std::max(16, faceSize / 4);
    std::vector<float> cloud(static_cast<size_t>(cloudSize) * cloudSize * 4, 0.0f);
    std::vector<float> face(static_cast<size_t>(faceSize) * faceSize * 3, 0.0f);
    std::vector<float> mipB;

    for (int f = 0; f < 6; ++f) {
        // 1. Diffuse sky at low resolution (it is smooth by construction: the
        //    smallest cloud feature is ~15 texels across at mip 0).
        for (int y = 0; y < cloudSize; ++y) {
            for (int x = 0; x < cloudSize; ++x) {
                const float u = 2.0f * (x + 0.5f) / cloudSize - 1.0f;
                const float v = 2.0f * (y + 0.5f) / cloudSize - 1.0f;
                const SkySample s = skyField(faceDirection(f, u, v));
                const size_t i = (static_cast<size_t>(y) * cloudSize + x) * 4;
                cloud[i] = s.rgb.x;
                cloud[i + 1] = s.rgb.y;
                cloud[i + 2] = s.rgb.z;
                cloud[i + 3] = s.dust;
            }
        }

        // 2. Bilinear upsample of the diffuse sky to the face resolution.
        for (int y = 0; y < faceSize; ++y) {
            const float fy = (y + 0.5f) / faceSize * cloudSize - 0.5f;
            const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0, cloudSize - 1);
            const int y1 = std::min(y0 + 1, cloudSize - 1);
            const float ty = std::clamp(fy - static_cast<float>(y0), 0.0f, 1.0f);
            for (int x = 0; x < faceSize; ++x) {
                const float fx = (x + 0.5f) / faceSize * cloudSize - 0.5f;
                const int x0 = std::clamp(static_cast<int>(std::floor(fx)), 0, cloudSize - 1);
                const int x1 = std::min(x0 + 1, cloudSize - 1);
                const float tx = std::clamp(fx - static_cast<float>(x0), 0.0f, 1.0f);
                const auto texel = [&](int cx, int cy, int c) {
                    return cloud[(static_cast<size_t>(cy) * cloudSize + cx) * 4 + c];
                };
                const size_t di = (static_cast<size_t>(y) * faceSize + x) * 3;
                for (int c = 0; c < 3; ++c) {
                    const float a = texel(x0, y0, c) + (texel(x1, y0, c) - texel(x0, y0, c)) * tx;
                    const float b = texel(x0, y1, c) + (texel(x1, y1, c) - texel(x0, y1, c)) * tx;
                    face[di + c] = a + (b - a) * ty;
                }
            }
        }

        // 3. Quantise mip 0 and build the rest of the chain from the float
        //    buffer, which is what keeps the total flux of every star.
        unsigned char* out = bake.faceData(0, f);
        for (size_t i = 0; i < static_cast<size_t>(faceSize) * faceSize; ++i) {
            out[i * 4 + 0] = skyByte(face[i * 3 + 0]);
            out[i * 4 + 1] = skyByte(face[i * 3 + 1]);
            out[i * 4 + 2] = skyByte(face[i * 3 + 2]);
            out[i * 4 + 3] = 255;
        }

        std::vector<float> src = std::move(face);
        int srcSize = faceSize;
        for (int level = 1; level < bake.mipCount; ++level) {
            const int dstSize = std::max(1, srcSize / 2);
            downsampleFace(src, srcSize, mipB, dstSize);
            unsigned char* dstOut = bake.faceData(level, f);
            for (size_t i = 0; i < static_cast<size_t>(dstSize) * dstSize; ++i) {
                dstOut[i * 4 + 0] = skyByte(mipB[i * 3 + 0]);
                dstOut[i * 4 + 1] = skyByte(mipB[i * 3 + 1]);
                dstOut[i * 4 + 2] = skyByte(mipB[i * 3 + 2]);
                dstOut[i * 4 + 3] = 255;
            }
            src = mipB;
            srcSize = dstSize;
        }
        face.assign(static_cast<size_t>(faceSize) * faceSize * 3, 0.0f);
    }

    return bake;
}

TextureCubemap uploadSkyCubemap(const SkyBake& bake) {
    TextureCubemap texture = {};
    if (bake.faceSize < 8 || bake.pixels.empty()) return texture;

    const unsigned int id = rlLoadTextureCubemap(bake.pixels.data(), bake.faceSize,
                                                RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
                                                bake.mipCount);
    if (id == 0) return texture;

    texture.id = id;
    texture.width = bake.faceSize;
    texture.height = bake.faceSize;
    texture.mipmaps = bake.mipCount;
    texture.format = RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

    // Trilinear + clamp: the shader picks the LOD itself, and blending the two
    // nearest levels is what makes the prefiltering continuous as the footprint
    // changes across the frame.
    rlEnableTextureCubemap(id);
    rlCubemapParameters(id, RL_TEXTURE_MIN_FILTER, RL_TEXTURE_FILTER_MIP_LINEAR);
    rlCubemapParameters(id, RL_TEXTURE_MAG_FILTER, RL_TEXTURE_FILTER_LINEAR);
    rlCubemapParameters(id, RL_TEXTURE_WRAP_S, RL_TEXTURE_WRAP_CLAMP);
    rlCubemapParameters(id, RL_TEXTURE_WRAP_T, RL_TEXTURE_WRAP_CLAMP);
    rlDisableTextureCubemap();
    return texture;
}

}  // namespace bh
