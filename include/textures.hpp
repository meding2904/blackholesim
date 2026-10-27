#pragma once

#include <vector>

#include "raylib.h"

namespace bh {

// Tileable RGBA fractal noise used for the accretion-disk turbulence.
// Four independent channels at different base frequencies. 512x512, mipmapped
// and trilinear-filtered (the shader samples it with explicit LODs).
Texture2D createDiskNoiseTexture();

// ---------------------------------------------------------------------------
//  Deep-field sky bake
// ---------------------------------------------------------------------------
// The background is rendered in two layers:
//
//   * this bake, a prefiltered cube map of everything *unresolved* -- the dense
//     faint-star population, the galactic band, its dust lanes and nebulosity.
//     Unresolved light is exactly what should be prefiltered: sampling it at the
//     mip level that matches the pixel's footprint on the sky is the correct
//     filter, and that is what stops the folded sky near the critical curve from
//     aliasing into confetti (a point-sampled star lattice cannot do that);
//   * the bright stars, which stay analytic in the shader so they keep their
//     one-pixel profile at any magnification (see sampleSky in blackhole.fs).
//
// The mip chain is filtered on the CPU (box filter per face) rather than with
// GenTextureMipmaps, which binds GL_TEXTURE_2D and therefore cannot generate
// cube-map mips through raylib.
struct SkyBake {
    int faceSize = 0;   // pixels per face at mip 0
    int mipCount = 0;   // log2(faceSize) + 1
    // [mip level][face 0..5][row][col], RGBA8: exactly the layout
    // rlLoadTextureCubemap() consumes.
    std::vector<unsigned char> pixels;

    const unsigned char* faceData(int level, int faceIndex) const;
    unsigned char* faceData(int level, int faceIndex);
};

// Deterministic: the same faceSize always bakes identical bytes.
SkyBake bakeSkyCubemap(int faceSize);

// Uploads a bake as a trilinear, clamp-to-edge cube map (no GL state outside the
// call is modified).
TextureCubemap uploadSkyCubemap(const SkyBake& bake);

}  // namespace bh
