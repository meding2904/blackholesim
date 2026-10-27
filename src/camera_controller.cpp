#include "camera_controller.hpp"

#include <algorithm>
#include <cmath>

namespace bh {
namespace {
constexpr float kDefaultDistance = 45.0f;
constexpr float kDefaultYaw = 0.70f;
constexpr float kDefaultPitch = 0.16f;
constexpr float kPi = 3.14159265358979f;
}  // namespace

CameraController::CameraController() = default;

float CameraController::clampDistance(float value) {
    return std::clamp(value, kMinCameraDistance, kMaxCameraDistance);
}

float CameraController::clampPitch(float value) { return std::clamp(value, -kMaxPitch, kMaxPitch); }

void CameraController::reset() {
    m_targetDistance = kDefaultDistance;
    m_targetYaw = kDefaultYaw;
    m_targetPitch = kDefaultPitch;
}

void CameraController::orbit(float deltaX, float deltaY) {
    if (!std::isfinite(deltaX)) deltaX = 0.0f;
    if (!std::isfinite(deltaY)) deltaY = 0.0f;
    m_targetYaw = std::remainder(m_targetYaw + deltaX, 2.0f * kPi);
    m_targetPitch = clampPitch(m_targetPitch + deltaY);
}

void CameraController::elevate(float deltaY) {
    m_targetPitch = clampPitch(m_targetPitch + deltaY);
}

void CameraController::zoomBy(float delta) {
    m_targetDistance = clampDistance(m_targetDistance * std::exp(-delta * 0.14f));
}

void CameraController::setView(float distance, float yaw, float pitch) {
    const float viewDistance = std::isfinite(distance) ? distance : kDefaultDistance;
    const float viewYaw = std::isfinite(yaw) ? std::remainder(yaw, 2.0f * kPi) : kDefaultYaw;
    const float viewPitch = std::isfinite(pitch) ? pitch : kDefaultPitch;
    m_targetDistance = m_distance = clampDistance(viewDistance);
    m_targetYaw = m_yaw = viewYaw;
    m_targetPitch = m_pitch = clampPitch(viewPitch);
}

void CameraController::update(float dt) {
    if (!std::isfinite(dt) || dt < 0.0f) dt = 0.0f;
    // Critically damped approach: independent of frame rate, never overshoots.
    const float alpha = 1.0f - std::exp(-dt * 14.0f);
    m_distance += (m_targetDistance - m_distance) * alpha;
    // Smooth along the shortest arc, then keep the yaw bounded so a long
    // session cannot accumulate float error.
    const float yawDelta = std::remainder(m_targetYaw - m_yaw, 2.0f * kPi);
    m_yaw = std::remainder(m_yaw + yawDelta * alpha, 2.0f * kPi);
    m_pitch += (m_targetPitch - m_pitch) * alpha;
}

CameraFrame CameraController::frame() const {
    const float cp = std::cos(m_pitch);
    CameraFrame cam;
    cam.position = {m_distance * cp * std::cos(m_yaw), m_distance * std::sin(m_pitch),
                    m_distance * cp * std::sin(m_yaw)};

    cam.forward = {-cam.position.x, -cam.position.y, -cam.position.z};
    const float len = std::sqrt(cam.forward.x * cam.forward.x + cam.forward.y * cam.forward.y +
                                cam.forward.z * cam.forward.z);
    cam.forward.x /= len;
    cam.forward.y /= len;
    cam.forward.z /= len;

    // right = normalize(forward x worldUp) with worldUp = +Y => (-fz, 0, fx).
    const float rightLen = std::sqrt(cam.forward.z * cam.forward.z + cam.forward.x * cam.forward.x);
    cam.right = {-cam.forward.z / rightLen, 0.0f, cam.forward.x / rightLen};
    // up = right x forward.
    cam.up = {cam.right.y * cam.forward.z - cam.right.z * cam.forward.y,
              cam.right.z * cam.forward.x - cam.right.x * cam.forward.z,
              cam.right.x * cam.forward.y - cam.right.y * cam.forward.x};
    return cam;
}

float CameraController::inclinationDegrees() const {
    return (0.5f * kPi - std::fabs(m_pitch)) * (180.0f / kPi);
}

}  // namespace bh
