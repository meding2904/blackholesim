#include "simulation_parameters.hpp"

#include <algorithm>
#include <cmath>

namespace bh {

namespace {
// The step is relative (dl = h r), so it is directly comparable between spins.
constexpr QualitySettings kQuality[kQualityCount] = {
    {"Performance", 0.62f, 0.055f, 420, 1},
    {"Balanced", 0.82f, 0.035f, 1200, 1},
    {"High", 0.88f, 0.022f, 1400, 2},
};

// NaN compares false against everything, so std::clamp would pass it through.
float finiteOr(float value, float fallback) {
    return std::isfinite(value) ? value : fallback;
}
}  // namespace

const QualitySettings& qualitySettings(QualityPreset preset) {
    const int index = std::clamp(static_cast<int>(preset), 0, kQualityCount - 1);
    return kQuality[index];
}

void SimulationParameters::clamp() {
    spin = std::clamp(finiteOr(spin, 0.6f), 0.0f, kMaxSpin);
    rotationSpeed = std::clamp(finiteOr(rotationSpeed, 180.0f), 0.0f, 180.0f);
    temperature = std::clamp(finiteOr(temperature, 6800.0f), 3000.0f, 20000.0f);
    // The disk cannot reach inside its own ISCO, which shrinks with spin.
    outerRadius =
        std::clamp(finiteOr(outerRadius, 26.0f), iscoRadius() * 1.25f, 45.0f);
    exposure = std::clamp(finiteOr(exposure, 0.75f), 0.1f, 4.0f);
    bloom = std::clamp(finiteOr(bloom, 0.35f), 0.0f, 2.0f);
    stellarMass = std::clamp(finiteOr(stellarMass, 10.0f), 0.1f, 1.0e8f);
    if (static_cast<int>(quality) < 0 || static_cast<int>(quality) >= kQualityCount) {
        quality = QualityPreset::Balanced;
    }
}

}  // namespace bh
