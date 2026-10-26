#pragma once

#include <string>

#include "camera_controller.hpp"
#include "raylib.h"
#include "simulation_parameters.hpp"

namespace bh {

// Owns the shaders, the procedural textures and the HDR render pipeline:
//   scene (RGBA16F) -> bright pass + blur pyramid -> ACES composite -> screen
class Renderer {
public:
    bool initialize(std::string& error);
    void shutdown();
    void render(const CameraFrame& camera, const SimulationParameters& params, float diskTime);
    void invalidateTargets() { m_targetsValid = false; }

    int sceneWidth() const { return m_sceneWidth; }
    int sceneHeight() const { return m_sceneHeight; }
    bool hdrActive() const { return m_hdr; }

private:
    struct SceneUniforms {
        int resolution = -1;
        int camPos = -1;
        int camRight = -1;
        int camUp = -1;
        int camForward = -1;
        int tanHalfFov = -1;
        int time = -1;
        int diskVisible = -1;
        int diskInner = -1;
        int diskOuter = -1;
        int rotationSpeed = -1;
        int tempScale = -1;
        int diskGain = -1;
        int diskDensity = -1;
        int stepSize = -1;
        int maxSteps = -1;
        int aaSamples = -1;
        int starGain = -1;
        int sky = -1;
        int noise = -1;
    };

    bool loadShaders(std::string& error);
    bool ensureTargets(int windowWidth, int windowHeight, float renderScale);
    void releaseTargets();
    void drawScene(const CameraFrame& camera, const SimulationParameters& params,
                   const QualitySettings& quality, float diskTime);
    void drawBloom();
    void drawComposite(const SimulationParameters& params);

    Shader m_sceneShader = {};
    Shader m_blurShader = {};
    Shader m_compositeShader = {};
    SceneUniforms m_scene = {};

    int m_blurTex = -1;
    int m_blurUVScale = -1;
    int m_blurTexel = -1;
    int m_blurDirection = -1;
    int m_blurRadius = -1;
    int m_blurThreshold = -1;

    int m_compScene = -1;
    int m_compNear = -1;
    int m_compFar = -1;
    int m_compResolution = -1;
    int m_compExposure = -1;
    int m_compBloom = -1;

    Texture2D m_sky = {};
    Texture2D m_diskNoise = {};

    RenderTexture2D m_sceneTarget = {};
    RenderTexture2D m_bloomA = {};
    RenderTexture2D m_bloomB = {};
    RenderTexture2D m_bloomC = {};
    RenderTexture2D m_bloomD = {};

    int m_sceneWidth = 0;
    int m_sceneHeight = 0;
    int m_windowWidth = 0;
    int m_windowHeight = 0;
    bool m_targetsValid = false;
    bool m_hdr = true;
    bool m_initialized = false;
};

// Resolves "shaders/x.fs" relative to the executable, the working directory or
// the source tree, so the app runs from the build directory or the repo root.
std::string resolveAssetPath(const std::string& relative);

}  // namespace bh
