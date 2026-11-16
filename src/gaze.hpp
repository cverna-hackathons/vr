// Where does the gaze ray hit the curved virtual screen?
// The screen is a cylinder section of radius `radius` around the vertical axis through the origin, spanning `arc` radians
// horizontally (centred on -Z) and `height` metres vertically (centred on y = 0).
#pragma once
#include <glm/glm.hpp>

#include <cmath>

// Returns true and the screen coordinates uv (0..1, u to the right, v up) if the ray from `eye` along `dir` hits the screen.
inline bool gaze_on_screen(const glm::vec3& eye, const glm::vec3& dir, float radius, float arc, float height, glm::vec2& uv) {
    // Solve |eye.xz + t * dir.xz| = radius for the forward intersection (the eye is inside the cylinder).
    const float a = dir.x * dir.x + dir.z * dir.z;
    if (a < 1e-8f) return false;  // looking straight up or down
    const float b = 2.0f * (eye.x * dir.x + eye.z * dir.z);
    const float c = eye.x * eye.x + eye.z * eye.z - radius * radius;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0) return false;
    const float t = (-b + std::sqrt(disc)) / (2.0f * a);
    if (t <= 0) return false;
    const glm::vec3 p = eye + dir * t;
    const float angle = std::atan2(p.x, -p.z);  // 0 straight ahead, positive to the right
    uv.x = angle / arc + 0.5f;
    uv.y = p.y / height + 0.5f;
    return uv.x >= 0.0f && uv.x <= 1.0f && uv.y >= 0.0f && uv.y <= 1.0f;
}
