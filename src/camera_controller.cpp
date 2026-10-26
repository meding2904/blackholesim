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
    m_targetYaw += deltaX;
    m_targetPitch = clampPitch(m_targetPitch + deltaY);
    // Keep yaw bounded so it can never lose float precision after long sessions.
    if (m_targetYaw > kPi) m_targetYaw -= 2.0f * kPi;
    if (m_targetYaw < -kPi) m_targetYaw += 2.0f * kPi;
}

void CameraController::elevate(float deltaY) {
    m_targetPitch = clampPitch(m_targetPitch + deltaY);
}

void CameraController::zoomBy(float delta) {
    m_targetDistance = clampDistance(m_targetDistance * std::exp(-delta * 0.14f));
}

void CameraController::setView(float distance, float yaw, float pitch) {
    m_targetDistance = m_distance = clampDistance(distance);
    m_targetYaw = m_yaw = yaw;
    m_targetPitch = m_pitch = clampPitch(pitch);
}

void CameraController::update(float dt) {
    // Critically damped approach: independent of frame rate, never overshoots.
    const float alpha = 1.0f - std::exp(-dt * 14.0f);
    m_distance += (m_targetDistance - m_distance) * alpha;
    m_yaw += (m_targetYaw - m_yaw) * alpha;
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
