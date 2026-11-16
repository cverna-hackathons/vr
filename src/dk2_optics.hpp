// Fixed optical/display parameters of the Oculus Rift DK2 (values reported by OpenHMD 0.3 for this device),
// plus the lens-distortion shader (PanoTools radial model with per-channel scale for chromatic aberration).
#pragma once
#include <glm/glm.hpp>

struct Dk2Optics {
    float screen_w_m = 0.12576f, screen_h_m = 0.07074f;   // physical panel size in the landscape orientation
    float lens_separation_m = 0.0635f;
    float lens_vertical_pos_m = 0.03537f;
    float ipd_m = 0.061f;
    float distortion_k[4] = {0.247f, -0.145f, 0.103f, 0.795f};  // [a, b, c, d]
    float aberration[3] = {0.985f, 1.0f, 1.015f};                // r, g, b scale

    // Per-eye off-axis projection (eye 0 = left, 1 = right).
    glm::mat4 projection(int eye, float znear = 0.05f, float zfar = 100.0f) const {
        const float fx = 1.26018f, fy = 1.12016f, ox = eye == 0 ? 0.00986005f : -0.00986005f;
        glm::mat4 p(0.0f);
        p[0][0] = fx; p[1][1] = fy; p[2][0] = ox;
        p[2][2] = -(zfar + znear) / (zfar - znear); p[2][3] = -1.0f;
        p[3][2] = -2.0f * zfar * znear / (zfar - znear);
        return p;
    }
    // Eye offset (left eye sees the world shifted right, i.e. the eye sits at -IPD/2).
    glm::mat4 eye_offset(int eye) const {
        glm::mat4 m(1.0f);
        m[3][0] = eye == 0 ? ipd_m * 0.5f : -ipd_m * 0.5f;
        return m;
    }
};
