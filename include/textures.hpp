#pragma once

#include "raylib.h"

namespace bh {

// Tileable RGBA fractal noise used for the accretion-disk turbulence.
// Four independent channels at different base frequencies. 512x512, mipmapped
// and trilinear-filtered (the shader samples it with explicit LODs).
Texture2D createDiskNoiseTexture();

}  // namespace bh
