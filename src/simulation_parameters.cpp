#include "simulation_parameters.hpp"

#include <algorithm>

namespace bh {

namespace {
constexpr QualitySettings kQuality[kQualityCount] = {
    {"Performance", 0.62f, 0.018f, 200, 1},
    {"Balanced", 0.82f, 0.012f, 320, 1},
    {"High", 0.88f, 0.008f, 460, 2},
};
}  // namespace

const QualitySettings& qualitySettings(QualityPreset preset) {
    const int index = std::clamp(static_cast<int>(preset), 0, kQualityCount - 1);
    return kQuality[index];
}

void SimulationParameters::clamp() {
    rotationSpeed = std::clamp(rotationSpeed, 0.0f, 3.0f);
    temperature = std::clamp(temperature, 3000.0f, 20000.0f);
    outerRadius = std::clamp(outerRadius, kInnerDiskRadius + 2.0f, 45.0f);
    exposure = std::clamp(exposure, 0.1f, 4.0f);
    bloom = std::clamp(bloom, 0.0f, 2.0f);
    stellarMass = std::clamp(stellarMass, 0.1f, 1.0e8f);
    if (static_cast<int>(quality) < 0 || static_cast<int>(quality) >= kQualityCount) {
        quality = QualityPreset::Balanced;
    }
}

}  // namespace bh
