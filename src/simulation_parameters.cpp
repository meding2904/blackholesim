#include "simulation_parameters.hpp"

#include <algorithm>

namespace bh {

namespace {
// The step is relative (dl = h r), so it is directly comparable between spins.
constexpr QualitySettings kQuality[kQualityCount] = {
    {"Performance", 0.62f, 0.055f, 420, 1},
    {"Balanced", 0.82f, 0.035f, 1200, 1},
    {"High", 0.88f, 0.022f, 1400, 2},
};
}  // namespace

const QualitySettings& qualitySettings(QualityPreset preset) {
    const int index = std::clamp(static_cast<int>(preset), 0, kQualityCount - 1);
    return kQuality[index];
}

void SimulationParameters::clamp() {
    spin = std::clamp(spin, 0.0f, kMaxSpin);
    rotationSpeed = std::clamp(rotationSpeed, 0.0f, 180.0f);
    temperature = std::clamp(temperature, 3000.0f, 20000.0f);
    // The disk cannot reach inside its own ISCO, which shrinks with spin.
    outerRadius = std::clamp(outerRadius, iscoRadius() * 1.25f, 45.0f);
    exposure = std::clamp(exposure, 0.1f, 4.0f);
    bloom = std::clamp(bloom, 0.0f, 2.0f);
    stellarMass = std::clamp(stellarMass, 0.1f, 1.0e8f);
    if (static_cast<int>(quality) < 0 || static_cast<int>(quality) >= kQualityCount) {
        quality = QualityPreset::Balanced;
    }
}

}  // namespace bh
