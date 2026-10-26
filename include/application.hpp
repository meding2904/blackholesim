#pragma once

#include <string>

#include "camera_controller.hpp"
#include "renderer.hpp"
#include "simulation_parameters.hpp"
#include "ui.hpp"

namespace bh {

struct RunOptions {
    int width = 1440;
    int height = 900;
    std::string capturePath;   // empty: interactive
    int captureFrames = 20;
    bool noUi = false;
    bool paused = false;
    float startTime = 0.0f;
    float yaw = 0.70f;
    float pitch = 0.16f;
    float distance = 45.0f;   // matches CameraController's default view
    int quality = -1;  // -1 keeps the default preset
};

class Application {
public:
    int run(const RunOptions& options);

private:
    void handleInput(float dt);

    CameraController m_camera;
    SimulationParameters m_params;
    Renderer m_renderer;
    Ui m_ui;
    float m_diskTime = 0.0f;
};

}  // namespace bh
