#pragma once

#include "raylib.h"

namespace bh {

// Procedural equirectangular star field with a faint galactic band.
// 4096x2048, RGBA8, mipmapped. Deterministic (fixed seed).
Texture2D createSkyTexture();

// Tileable RGBA fractal noise used for the accretion-disk turbulence.
// Four independent channels at different base frequencies. 512x512.
Texture2D createDiskNoiseTexture();

}  // namespace bh
