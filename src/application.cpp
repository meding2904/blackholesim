#include "application.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "raylib.h"
#include "raylib.h"
#include "rlgl.h"

namespace bh {
namespace {
// The shader's orbital phase uses physical time in r_g/c. Real ISCO periods are
// ~92 t_g, so the animation clock is sped up for a watchable disk.
constexpr float kDiskTimeScale = 12.0f;
}  // namespace

int Application::run(const RunOptions& options) {
    SetTraceLogLevel(LOG_WARNING);
    // vsync would cap the benchmark at the display rate, hiding the real cost.
    const bool benchmark = options.benchFrames > 0;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE |
                   (benchmark ? 0 : static_cast<unsigned int>(FLAG_VSYNC_HINT)));
    InitWindow(options.width, options.height,
               "Schwarzschild Black Hole - Gravitational Lensing");
    if (!IsWindowReady()) {
        std::fprintf(stderr, "Failed to create the application window.\n");
        return 1;
    }

    SetExitKey(KEY_ESCAPE);
    SetTargetFPS(0);  // vsync-limited

    m_params.spin = options.spin;
    m_params.showUi = !options.noUi;
    m_params.paused = options.paused;
    if (options.quality >= 0 && options.quality < kQualityCount) {
        m_params.quality = static_cast<QualityPreset>(options.quality);
    }
    m_params.clamp();

    m_camera.setView(options.distance, options.yaw, options.pitch);

    std::string error;
    if (!m_renderer.initialize(error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        CloseWindow();
        return 1;
    }

    m_diskTime = options.startTime;
    int frameIndex = 0;
    std::vector<double> frameMs;
    std::chrono::steady_clock::time_point prevStart{};
    if (benchmark) frameMs.reserve(static_cast<size_t>(options.benchFrames));

    while (!WindowShouldClose()) {
        // Frame-to-frame wall clock, so the sample covers submit, GPU work and
        // the buffer swap -- i.e. what the user actually feels.
        const auto frameStart = std::chrono::steady_clock::now();
        if (benchmark && prevStart.time_since_epoch().count() != 0) {
            frameMs.push_back(std::chrono::duration<double, std::milli>(frameStart - prevStart)
                                  .count());
            if (static_cast<int>(frameMs.size()) >= options.benchFrames) break;
        }
        prevStart = frameStart;
        const float dt = std::min(GetFrameTime(), 0.1f);
        handleInput(dt);

        if (!m_params.paused) m_diskTime += dt * kDiskTimeScale;

        const CameraFrame frame = m_camera.frame();

        BeginDrawing();
        ClearBackground(BLACK);
        m_renderer.render(frame, m_params, m_diskTime);

        UiReadout readout;
        readout.cameraDistance = m_camera.distance();
        readout.inclinationDegrees = m_camera.inclinationDegrees();
        readout.fps = GetFPS();
        readout.internalWidth = m_renderer.sceneWidth();
        readout.internalHeight = m_renderer.sceneHeight();
        readout.hdr = m_renderer.hdrActive();
        m_ui.draw(m_params, readout);

        ++frameIndex;
        const bool captureNow =
            !options.capturePath.empty() && frameIndex >= std::max(1, options.captureFrames);
        if (captureNow) {
            // TakeScreenshot reads the framebuffer without flushing the pending
            // 2D batch, so the UI would be missing from the image.
            rlDrawRenderBatchActive();
            TakeScreenshot(options.capturePath.c_str());
            EndDrawing();
            break;
        }
        EndDrawing();
    }

    if (benchmark) {
        // Drop the first frames: shader warm-up and the first mip uploads.
        const size_t skip = std::min<size_t>(8, frameMs.size() / 4);
        std::vector<double> samples(frameMs.begin() + static_cast<long>(skip), frameMs.end());
        if (!samples.empty()) {
            std::sort(samples.begin(), samples.end());
            const double median = samples[samples.size() / 2];
            const double p95 = samples[static_cast<size_t>(0.95 * (samples.size() - 1))];
            const double best = samples.front();
            std::printf("bench %dx%d internal %dx%d  median %.2f ms  p95 %.2f ms  best %.2f ms\n",
                        options.width, options.height, m_renderer.sceneWidth(),
                        m_renderer.sceneHeight(), median, p95, best);
        }
    }

    m_renderer.shutdown();
    CloseWindow();

    if (!options.capturePath.empty()) {
        if (FileExists(options.capturePath.c_str())) {
            std::printf("Captured frame to %s\n", options.capturePath.c_str());
            return 0;
        }
        std::fprintf(stderr, "Failed to write screenshot to %s\n", options.capturePath.c_str());
        return 1;
    }
    return 0;
}

void Application::handleInput(float dt) {
    const Vector2 mouseDelta = GetMouseDelta();
    const bool overUi = m_params.showUi && m_ui.containsMouse();

    if (!overUi) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            m_camera.orbit(-mouseDelta.x * 0.006f, mouseDelta.y * 0.005f);
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE) ||
            (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && IsKeyDown(KEY_LEFT_CONTROL))) {
            m_camera.elevate(-mouseDelta.y * 0.004f);
        }
        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) m_camera.zoomBy(wheel);
    }

    if (IsKeyDown(KEY_LEFT)) m_camera.orbit(-1.2f * dt, 0.0f);
    if (IsKeyDown(KEY_RIGHT)) m_camera.orbit(1.2f * dt, 0.0f);
    if (IsKeyDown(KEY_UP)) m_camera.elevate(0.8f * dt);
    if (IsKeyDown(KEY_DOWN)) m_camera.elevate(-0.8f * dt);
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) m_camera.zoomBy(1.0f);
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) m_camera.zoomBy(-1.0f);

    if (IsKeyPressed(KEY_R)) m_camera.reset();
    if (IsKeyPressed(KEY_SPACE)) m_params.paused = !m_params.paused;
    if (IsKeyPressed(KEY_H)) m_params.showUi = !m_params.showUi;
    if (IsKeyPressed(KEY_ONE)) m_params.quality = QualityPreset::Performance;
    if (IsKeyPressed(KEY_TWO)) m_params.quality = QualityPreset::Balanced;
    if (IsKeyPressed(KEY_THREE)) m_params.quality = QualityPreset::High;

    m_params.clamp();
    m_camera.update(dt);
}

}  // namespace bh
