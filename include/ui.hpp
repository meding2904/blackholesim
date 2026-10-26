#pragma once

#include "raylib.h"
#include "simulation_parameters.hpp"

namespace bh {

struct UiReadout {
    float cameraDistance = 26.0f;
    float inclinationDegrees = 80.0f;
    int fps = 0;
    int internalWidth = 0;
    int internalHeight = 0;
    bool hdr = true;
};

// Immediate-mode control panel drawn with raylib primitives (no GUI library).
// Every widget writes straight into SimulationParameters.
class Ui {
public:
    void draw(SimulationParameters& params, const UiReadout& readout);
    bool containsMouse() const;

private:
    unsigned int m_activeSlider = 0;
};

}  // namespace bh
