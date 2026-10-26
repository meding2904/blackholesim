#pragma once

#include "geodesic.hpp"

namespace bh {

// Geometry is dimensionless: all distances are measured in gravitational radii,
// so changing the mass never changes the image. It only changes the physical
// scale readout (kilometres, seconds), which is what gravitationalRadiusKm()
// and lightCrossingSeconds() report.
inline constexpr float kInnerDiskRadius = 6.0f;   // ISCO for Schwarzschild

enum class QualityPreset { Performance = 0, Balanced = 1, High = 2 };
inline constexpr int kQualityCount = 3;

struct QualitySettings {
    const char* label;
    float renderScale;  // internal resolution relative to the window
    float phiStep;      // RK4 step in orbital phase (radians)
    int maxSteps;       // bounded integration => no GPU hangs
    int aaSamples;      // rays per pixel
};

const QualitySettings& qualitySettings(QualityPreset preset);

struct SimulationParameters {
    bool diskVisible = true;
    float rotationSpeed = 1.0f;   // multiples of the Keplerian rate
    float temperature = 6800.0f;  // Kelvin at the peak of the thin-disk profile
    float outerRadius = 26.0f;    // r_g
    float exposure = 0.75f;
    float bloom = 0.35f;
    bool paused = false;
    bool showUi = true;
    QualityPreset quality = QualityPreset::Balanced;
    float stellarMass = 10.0f;    // solar masses, scale readout only

    void clamp();
};

// Physical scales (cgs/SI converted): r_g = GM/c^2 = 1.4766 km per solar mass.
inline double gravitationalRadiusKm(double solarMasses) {
    return 1.47662504 * solarMasses;
}

// Light-crossing time of one gravitational radius, in seconds.
inline double lightCrossingSeconds(double solarMasses) {
    return gravitationalRadiusKm(solarMasses) / 299792.458;
}

}  // namespace bh
