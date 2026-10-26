#pragma once

#include "raylib.h"

namespace bh {

inline constexpr float kMinCameraDistance = 8.0f;   // r_g (well outside the ISCO)
inline constexpr float kMaxCameraDistance = 90.0f;
inline constexpr float kMaxPitch = 1.45f;           // ~83 degrees, avoids the poles

struct CameraFrame {
    Vector3 position;
    Vector3 forward;
    Vector3 right;
    Vector3 up;
};

// Orbit camera aimed at the singularity. All state is smoothed towards a target
// so mouse input never produces a jumpy frame.
class CameraController {
public:
    CameraController();

    void update(float dt);
    void reset();

    void orbit(float deltaX, float deltaY);   // radians of yaw / pitch
    void elevate(float deltaY);
    void zoomBy(float delta);                 // mouse wheel notches

    void setView(float distance, float yaw, float pitch);

    CameraFrame frame() const;
    float distance() const { return m_distance; }
    float pitch() const { return m_pitch; }
    float yaw() const { return m_yaw; }
    // Angle between the line of sight and the disk normal: 0 face-on, 90 edge-on.
    float inclinationDegrees() const;

private:
    static float clampDistance(float value);
    static float clampPitch(float value);

    float m_distance = 45.0f;
    float m_yaw = 0.70f;
    float m_pitch = 0.16f;

    float m_targetDistance = 45.0f;
    float m_targetYaw = 0.70f;
    float m_targetPitch = 0.16f;
};

}  // namespace bh
