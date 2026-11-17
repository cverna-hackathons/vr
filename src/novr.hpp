// --no-vr support: a keyboard-driven stand-in for the headset's tracking.
//
//   Ctrl + Left/Right          turn the head left / right
//   Ctrl + Up/Down             look up / down
//   Ctrl + Shift + Left/Right  lean left / right
//   Ctrl + Shift + Up/Down     lean forward / back
//   Ctrl + PageUp/PageDown     lean up / down
//   Ctrl + Backspace           back to looking straight ahead, no lean
//
// Plain keys keep their usual meaning in every program. Hold Alt for slow, fine movement.
#pragma once
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct HeadKeys {
    bool ctrl = false, shift = false, slow = false;
    bool left = false, right = false, up = false, down = false, page_up = false, page_down = false, reset = false;
};

class KeyboardHead {
public:
    void update(float dt, const HeadKeys& k) {
        if (k.reset) { yaw_ = pitch_ = 0; lean_ = glm::vec3(0.0f); return; }
        if (!k.ctrl) return;
        const float f = k.slow ? 0.25f : 1.0f;
        const float turn = glm::radians(70.0f) * f * dt, move = 0.35f * f * dt;
        const float h = float(k.left) - float(k.right), v = float(k.up) - float(k.down);
        if (!k.shift) {
            yaw_ += h * turn;
            pitch_ = std::min(glm::radians(89.0f), std::max(glm::radians(-89.0f), pitch_ + v * turn));
        } else {
            lean_.x -= h * move;      // left key = lean left (-X)
            lean_.z -= v * move;      // up key = lean forward (-Z)
        }
        lean_.y += (float(k.page_up) - float(k.page_down)) * move;
        lean_.x = std::min(0.6f, std::max(-0.6f, lean_.x));
        lean_.y = std::min(0.4f, std::max(-0.6f, lean_.y));
        lean_.z = std::min(0.6f, std::max(-0.6f, lean_.z));
    }
    // Headset-to-world rotation, +Y up, -Z forward; positive yaw turns left.
    glm::quat orientation() const { return glm::angleAxis(yaw_, glm::vec3(0, 1, 0)) * glm::angleAxis(pitch_, glm::vec3(1, 0, 0)); }
    glm::vec3 position() const { return lean_; }

private:
    float yaw_ = 0, pitch_ = 0;
    glm::vec3 lean_{0.0f};
};

#ifdef SDL_h_
inline HeadKeys head_keys_from_sdl() {
    const Uint8* s = SDL_GetKeyboardState(nullptr);
    HeadKeys k;
    k.ctrl = s[SDL_SCANCODE_LCTRL] || s[SDL_SCANCODE_RCTRL];
    k.shift = s[SDL_SCANCODE_LSHIFT] || s[SDL_SCANCODE_RSHIFT];
    k.slow = s[SDL_SCANCODE_LALT] || s[SDL_SCANCODE_RALT];
    k.left = s[SDL_SCANCODE_LEFT];
    k.right = s[SDL_SCANCODE_RIGHT];
    k.up = s[SDL_SCANCODE_UP];
    k.down = s[SDL_SCANCODE_DOWN];
    k.page_up = s[SDL_SCANCODE_PAGEUP];
    k.page_down = s[SDL_SCANCODE_PAGEDOWN];
    k.reset = s[SDL_SCANCODE_BACKSPACE] && k.ctrl;
    return k;
}
#endif
