#include "renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "rlgl.h"
#include "textures.hpp"

namespace bh {
namespace {

constexpr float kVerticalFovDegrees = 50.0f;
constexpr float kDiskGain = 2.2f;
constexpr float kDiskOpticalDepth = 6.0f;
constexpr float kStarGain = 1.6f;
constexpr float kBloomThreshold = 1.3f;
// Deep-field cube map resolution: one face per 90 degrees, so 512 puts a texel
// at 0.003 rad. That is two to three screen pixels at 720p, which is the right
// order for the unresolved population (a texel is a prefiltered point source).
constexpr int kSkyFaceSize = 512;

RenderTexture2D createTarget(int width, int height) {
    RenderTexture2D target =
        LoadRenderTextureEx(width, height, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16);
    if (target.id == 0) {
        target = LoadRenderTexture(width, height);  // 8-bit fallback
    }
    if (target.id != 0) {
        SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(target.texture, TEXTURE_WRAP_CLAMP);
    }
    return target;
}

void bindSampler(Shader shader, int location, Texture2D texture, int unit) {
    if (location < 0) return;
    int unitValue = unit;
    SetShaderValue(shader, location, &unitValue, SHADER_UNIFORM_INT);
    rlActiveTextureSlot(unit);
    rlEnableTexture(texture.id);
    rlActiveTextureSlot(0);
}

// Same, for the sky cube map: rlEnableTexture would bind GL_TEXTURE_2D.
void bindCubemapSampler(Shader shader, int location, TextureCubemap texture, int unit) {
    if (location < 0 || texture.id == 0) return;
    int unitValue = unit;
    SetShaderValue(shader, location, &unitValue, SHADER_UNIFORM_INT);
    rlActiveTextureSlot(unit);
    rlEnableTextureCubemap(texture.id);
    rlActiveTextureSlot(0);
}

// BeginTextureMode changes the orthographic projection to the target size and
// EndTextureMode does not restore the screen one, so re-establish it explicitly.
void resetScreenProjection() {
    rlDrawRenderBatchActive();
    const int width = GetScreenWidth();
    const int height = GetScreenHeight();
    rlViewport(0, 0, width, height);
    rlMatrixMode(RL_PROJECTION);
    rlLoadIdentity();
    rlOrtho(0, width, height, 0, 0.0f, 1.0f);
    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();
}

}  // namespace

std::string resolveAssetPath(const std::string& relative) {
    std::vector<std::string> candidates;
    const char* appDirectory = GetApplicationDirectory();
    if (appDirectory != nullptr) candidates.emplace_back(std::string(appDirectory) + relative);
    candidates.push_back(relative);
    candidates.push_back("../" + relative);
#ifdef BLACKHOLE_SOURCE_DIR
    candidates.emplace_back(std::string(BLACKHOLE_SOURCE_DIR) + "/" + relative);
#endif
    for (const std::string& candidate : candidates) {
        if (FileExists(candidate.c_str())) return candidate;
    }
    return candidates.front();
}

bool Renderer::initialize(std::string& error) {
    if (!loadShaders(error)) return false;

    m_diskNoise = createDiskNoiseTexture();
    if (m_diskNoise.id == 0) {
        error = "Failed to create the procedural disk turbulence texture.";
        return false;
    }

    m_sky = uploadSkyCubemap(bakeSkyCubemap(kSkyFaceSize));
    if (m_sky.id == 0) {
        error = "Failed to create the procedural deep-field sky cube map.";
        return false;
    }

    // Sampler units are fixed (2..3); raylib's 2D batch uses unit 0.
    int unit = 2;
    SetShaderValue(m_sceneShader, m_scene.noise, &unit, SHADER_UNIFORM_INT);

    m_initialized = true;
    return true;
}

bool Renderer::loadShaders(std::string& error) {
    const std::string scenePath = resolveAssetPath("shaders/blackhole.fs");
    const std::string blurPath = resolveAssetPath("shaders/blur.fs");
    const std::string compositePath = resolveAssetPath("shaders/composite.fs");

    m_sceneShader = LoadShader(nullptr, scenePath.c_str());
    if (!IsShaderValid(m_sceneShader)) {
        error = "Failed to compile " + scenePath + ". See the shader log above.";
        return false;
    }
    m_blurShader = LoadShader(nullptr, blurPath.c_str());
    if (!IsShaderValid(m_blurShader)) {
        error = "Failed to compile " + blurPath + ". See the shader log above.";
        return false;
    }
    m_compositeShader = LoadShader(nullptr, compositePath.c_str());
    if (!IsShaderValid(m_compositeShader)) {
        error = "Failed to compile " + compositePath + ". See the shader log above.";
        return false;
    }

    m_scene.resolution = GetShaderLocation(m_sceneShader, "uResolution");
    m_scene.camPos = GetShaderLocation(m_sceneShader, "uCamPos");
    m_scene.camRight = GetShaderLocation(m_sceneShader, "uCamRight");
    m_scene.camUp = GetShaderLocation(m_sceneShader, "uCamUp");
    m_scene.camForward = GetShaderLocation(m_sceneShader, "uCamForward");
    m_scene.tanHalfFov = GetShaderLocation(m_sceneShader, "uTanHalfFov");
    m_scene.time = GetShaderLocation(m_sceneShader, "uTime");
    m_scene.diskVisible = GetShaderLocation(m_sceneShader, "uDiskVisible");
    m_scene.diskInner = GetShaderLocation(m_sceneShader, "uDiskInner");
    m_scene.diskOuter = GetShaderLocation(m_sceneShader, "uDiskOuter");
    m_scene.rotationSpeed = GetShaderLocation(m_sceneShader, "uRotationSpeed");
    m_scene.tempScale = GetShaderLocation(m_sceneShader, "uTempScale");
    m_scene.diskGain = GetShaderLocation(m_sceneShader, "uDiskGain");
    m_scene.diskDensity = GetShaderLocation(m_sceneShader, "uDiskDensity");
    m_scene.stepSize = GetShaderLocation(m_sceneShader, "uStepSize");
    m_scene.spin = GetShaderLocation(m_sceneShader, "uSpin");
    m_scene.horizon = GetShaderLocation(m_sceneShader, "uHorizon");
    m_scene.maxSteps = GetShaderLocation(m_sceneShader, "uMaxSteps");
    m_scene.aaSamples = GetShaderLocation(m_sceneShader, "uAASamples");
    m_scene.starGain = GetShaderLocation(m_sceneShader, "uStarGain");
    m_scene.noise = GetShaderLocation(m_sceneShader, "uNoise");
    m_scene.sky = GetShaderLocation(m_sceneShader, "uSky");
    m_scene.skyTexel = GetShaderLocation(m_sceneShader, "uSkyTexel");
    m_scene.photonPro = GetShaderLocation(m_sceneShader, "uPhotonPro");

    if (m_scene.resolution < 0 || m_scene.noise < 0 || m_scene.sky < 0) {
        char detail[192];
        std::snprintf(detail, sizeof(detail),
                      "Black hole shader is missing uniforms: res=%d cam=%d noise=%d "
                      "step=%d max=%d",
                      m_scene.resolution, m_scene.camPos, m_scene.noise,
                      m_scene.stepSize, m_scene.maxSteps);
        error = detail;
        return false;
    }

    m_blurTex = GetShaderLocation(m_blurShader, "uTex");
    m_blurUVScale = GetShaderLocation(m_blurShader, "uUVScale");
    m_blurTexel = GetShaderLocation(m_blurShader, "uTexel");
    m_blurDirection = GetShaderLocation(m_blurShader, "uDirection");
    m_blurRadius = GetShaderLocation(m_blurShader, "uRadius");
    m_blurThreshold = GetShaderLocation(m_blurShader, "uThreshold");

    m_compScene = GetShaderLocation(m_compositeShader, "uScene");
    m_compNear = GetShaderLocation(m_compositeShader, "uBloomNear");
    m_compFar = GetShaderLocation(m_compositeShader, "uBloomFar");
    m_compResolution = GetShaderLocation(m_compositeShader, "uResolution");
    m_compExposure = GetShaderLocation(m_compositeShader, "uExposure");
    m_compBloom = GetShaderLocation(m_compositeShader, "uBloomStrength");

    rlDisableDepthTest();
    return true;
}

void Renderer::shutdown() {
    releaseTargets();
    if (m_initialized) {
        UnloadTexture(m_diskNoise);
        if (m_sky.id != 0) rlUnloadTexture(m_sky.id);
        m_diskNoise = {};
        m_sky = {};
        m_initialized = false;
    }
    UnloadShader(m_sceneShader);
    UnloadShader(m_blurShader);
    UnloadShader(m_compositeShader);
    m_sceneShader = {};
    m_blurShader = {};
    m_compositeShader = {};
}

void Renderer::releaseTargets() {
    if (m_sceneTarget.id != 0) UnloadRenderTexture(m_sceneTarget);
    if (m_bloomA.id != 0) UnloadRenderTexture(m_bloomA);
    if (m_bloomB.id != 0) UnloadRenderTexture(m_bloomB);
    if (m_bloomC.id != 0) UnloadRenderTexture(m_bloomC);
    if (m_bloomD.id != 0) UnloadRenderTexture(m_bloomD);
    m_sceneTarget = {};
    m_bloomA = {};
    m_bloomB = {};
    m_bloomC = {};
    m_bloomD = {};
    m_targetsValid = false;
}

bool Renderer::ensureTargets(int windowWidth, int windowHeight, float renderScale) {
    if (windowWidth < 8 || windowHeight < 8) return false;

    const int width = std::max(64, static_cast<int>(std::lround(windowWidth * renderScale)));
    const int height = std::max(64, static_cast<int>(std::lround(windowHeight * renderScale)));

    if (m_targetsValid && width == m_sceneWidth && height == m_sceneHeight &&
        windowWidth == m_windowWidth && windowHeight == m_windowHeight) {
        return true;
    }

    releaseTargets();

    m_sceneTarget = createTarget(width, height);
    if (m_sceneTarget.id == 0) return false;
    m_hdr = m_sceneTarget.texture.format == PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;

    const int nearWidth = std::max(16, width / 4);
    const int nearHeight = std::max(16, height / 4);
    const int farWidth = std::max(8, width / 8);
    const int farHeight = std::max(8, height / 8);

    m_bloomA = createTarget(nearWidth, nearHeight);
    m_bloomB = createTarget(nearWidth, nearHeight);
    m_bloomC = createTarget(farWidth, farHeight);
    m_bloomD = createTarget(farWidth, farHeight);
    if (m_bloomA.id == 0 || m_bloomB.id == 0 || m_bloomC.id == 0 || m_bloomD.id == 0) {
        releaseTargets();
        return false;
    }

    m_sceneWidth = width;
    m_sceneHeight = height;
    m_windowWidth = windowWidth;
    m_windowHeight = windowHeight;
    m_targetsValid = true;
    return true;
}

void Renderer::render(const CameraFrame& camera, const SimulationParameters& params, float diskTime) {
    if (!m_initialized) return;

    const QualitySettings& quality = qualitySettings(params.quality);
    if (!ensureTargets(GetScreenWidth(), GetScreenHeight(), quality.renderScale)) return;

    drawScene(camera, params, quality, diskTime);
    drawBloom();
    drawComposite(params);
}

void Renderer::drawScene(const CameraFrame& camera, const SimulationParameters& params,
                         const QualitySettings& quality, float diskTime) {
    BeginTextureMode(m_sceneTarget);
    ClearBackground(BLANK);

    BeginShaderMode(m_sceneShader);

    const float resolution[2] = {static_cast<float>(m_sceneWidth),
                                 static_cast<float>(m_sceneHeight)};
    SetShaderValue(m_sceneShader, m_scene.resolution, resolution, SHADER_UNIFORM_VEC2);

    const Vector3 position = camera.position;
    const Vector3 forward = camera.forward;
    const Vector3 right = camera.right;
    const Vector3 up = camera.up;
    SetShaderValue(m_sceneShader, m_scene.camPos, &position, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_sceneShader, m_scene.camForward, &forward, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_sceneShader, m_scene.camRight, &right, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_sceneShader, m_scene.camUp, &up, SHADER_UNIFORM_VEC3);
    if (std::getenv("BH_DUMP_CAM") != nullptr) {
        std::printf(
            "CAM %.6f %.6f %.6f | FWD %.6f %.6f %.6f | RIGHT %.6f %.6f %.6f | UP %.6f %.6f %.6f"
            " | RES %d %d\n",
            position.x, position.y, position.z, forward.x, forward.y, forward.z, right.x, right.y,
            right.z, up.x, up.y, up.z, m_sceneWidth, m_sceneHeight);
    }

    const float tanHalfFov = std::tan(kVerticalFovDegrees * 0.5f * DEG2RAD);
    SetShaderValue(m_sceneShader, m_scene.tanHalfFov, &tanHalfFov, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.time, &diskTime, SHADER_UNIFORM_FLOAT);

    const float diskVisible = params.diskVisible ? 1.0f : 0.0f;
    const float diskInner = params.iscoRadius();
    const float diskOuter = params.outerRadius;
    const float rotation = params.rotationSpeed;
    const float tempScale = params.temperature;
    const float gain = kDiskGain;
    const float density = kDiskOpticalDepth;
    const float stepSize = quality.phiStep;
    const float starGain = kStarGain;
    const int maxSteps = quality.maxSteps;
    const int aaSamples = quality.aaSamples;

    SetShaderValue(m_sceneShader, m_scene.diskVisible, &diskVisible, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.diskInner, &diskInner, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.diskOuter, &diskOuter, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.rotationSpeed, &rotation, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.tempScale, &tempScale, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.diskGain, &gain, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.diskDensity, &density, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.stepSize, &stepSize, SHADER_UNIFORM_FLOAT);
    const float spin = params.spin;
    const float horizon = params.horizonRadius();
    // The prograde photon orbit is the capture theorem's threshold: a ray that
    // is inside it and moving inward can never escape, so the shader stops it
    // there instead of letting the Boyer-Lindquist dphi spike creep it to a step
    // budget it cannot spend.
    const float photonPro = static_cast<float>(kerrPhotonOrbitRadius(spin, true));
    SetShaderValue(m_sceneShader, m_scene.spin, &spin, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.horizon, &horizon, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.photonPro, &photonPro, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_sceneShader, m_scene.maxSteps, &maxSteps, SHADER_UNIFORM_INT);
    SetShaderValue(m_sceneShader, m_scene.aaSamples, &aaSamples, SHADER_UNIFORM_INT);
    SetShaderValue(m_sceneShader, m_scene.starGain, &starGain, SHADER_UNIFORM_FLOAT);
    // Angular size of one deep-field texel: a cube face spans 90 degrees, so the
    // shader can turn a pixel footprint into a mip level without knowing the
    // bake size.
    const float skyTexel = 1.5707963f / static_cast<float>(kSkyFaceSize);
    SetShaderValue(m_sceneShader, m_scene.skyTexel, &skyTexel, SHADER_UNIFORM_FLOAT);

    bindSampler(m_sceneShader, m_scene.noise, m_diskNoise, 2);
    bindCubemapSampler(m_sceneShader, m_scene.sky, m_sky, 3);

    rlDisableDepthTest();
    DrawRectangle(0, 0, m_sceneWidth, m_sceneHeight, WHITE);

    EndShaderMode();
    EndTextureMode();

    if (std::getenv("BH_DEBUG") != nullptr) {
        static int logged = 0;
        if (logged < 3) {
            ++logged;
            Image img = LoadImageFromTexture(m_sceneTarget.texture);
            if (img.data != nullptr) {
                const Color a = GetImageColor(img, m_sceneWidth / 2, m_sceneHeight / 2);
                const Color b = GetImageColor(img, m_sceneWidth / 2, m_sceneHeight / 4);
                const Color c = GetImageColor(img, m_sceneWidth / 4, m_sceneHeight / 2);
                TraceLog(LOG_WARNING, "SCENE px center=%d,%d,%d upper=%d,%d,%d left=%d,%d,%d",
                         a.r, a.g, a.b, b.r, b.g, b.b, c.r, c.g, c.b);
                UnloadImage(img);
            } else {
                TraceLog(LOG_WARNING, "SCENE readback failed");
            }
            TraceLog(LOG_WARNING, "UNIFORMS cam=%d step=%d max=%d aa=%d noise=%d gain=%d",
                     m_scene.camPos, m_scene.stepSize, m_scene.maxSteps, m_scene.aaSamples,
                     m_scene.noise, m_scene.diskGain);
        }
    }
}

void Renderer::drawBloom() {
    BeginShaderMode(m_blurShader);

    const auto pass = [&](RenderTexture2D target, Texture2D source, float dirX, float dirY,
                          float radius, float threshold) {
        const float texel[2] = {1.0f / static_cast<float>(source.width),
                                1.0f / static_cast<float>(source.height)};
        const float uvScale[2] = {1.0f / static_cast<float>(target.texture.width),
                                  1.0f / static_cast<float>(target.texture.height)};
        const float direction[2] = {dirX, dirY};
        SetShaderValue(m_blurShader, m_blurUVScale, uvScale, SHADER_UNIFORM_VEC2);
        SetShaderValue(m_blurShader, m_blurTexel, texel, SHADER_UNIFORM_VEC2);
        SetShaderValue(m_blurShader, m_blurDirection, direction, SHADER_UNIFORM_VEC2);
        SetShaderValue(m_blurShader, m_blurRadius, &radius, SHADER_UNIFORM_FLOAT);
        SetShaderValue(m_blurShader, m_blurThreshold, &threshold, SHADER_UNIFORM_FLOAT);
        bindSampler(m_blurShader, m_blurTex, source, 1);

        BeginTextureMode(target);
        ClearBackground(BLANK);
        DrawRectangle(0, 0, target.texture.width, target.texture.height, WHITE);
        EndTextureMode();
    };

    // Near glow: threshold + horizontal blur at 1/4 res, then vertical.
    pass(m_bloomA, m_sceneTarget.texture, 1.0f, 0.0f, 1.0f, kBloomThreshold);
    pass(m_bloomB, m_bloomA.texture, 0.0f, 1.0f, 1.0f, -1.0f);
    // Far halo: blur the already-blurred image at 1/8 res with a wider kernel.
    pass(m_bloomC, m_bloomB.texture, 1.0f, 0.0f, 2.6f, -1.0f);
    pass(m_bloomD, m_bloomC.texture, 0.0f, 1.0f, 2.6f, -1.0f);

    EndShaderMode();
}

void Renderer::drawComposite(const SimulationParameters& params) {
    resetScreenProjection();

    const int width = GetScreenWidth();
    const int height = GetScreenHeight();

    BeginShaderMode(m_compositeShader);

    const float resolution[2] = {static_cast<float>(width), static_cast<float>(height)};
    const float exposure = params.exposure;
    const float bloom = params.bloom;
    SetShaderValue(m_compositeShader, m_compResolution, resolution, SHADER_UNIFORM_VEC2);
    SetShaderValue(m_compositeShader, m_compExposure, &exposure, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_compositeShader, m_compBloom, &bloom, SHADER_UNIFORM_FLOAT);

    bindSampler(m_compositeShader, m_compScene, m_sceneTarget.texture, 1);
    bindSampler(m_compositeShader, m_compNear, m_bloomB.texture, 2);
    bindSampler(m_compositeShader, m_compFar, m_bloomD.texture, 3);

    if (std::getenv("BH_DEBUG") != nullptr) {
        static int logged = 0;
        if (logged++ == 0) {
            TraceLog(LOG_WARNING,
                     "COMPOSITE loc scene=%d near=%d far=%d exp=%d | tex ids scene=%u near=%u "
                     "far=%u",
                     m_compScene, m_compNear, m_compFar, m_compExposure,
                     m_sceneTarget.texture.id, m_bloomB.texture.id, m_bloomD.texture.id);
        }
    }

    DrawRectangle(0, 0, width, height, WHITE);

    EndShaderMode();
}

}  // namespace bh
