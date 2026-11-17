// Stereo rendering for the Rift DK2 via OpenHMD: one offscreen framebuffer per eye, then a
// lens-distortion pass (PanoTools radial model + chromatic aberration) that
// writes the left eye into the left half of the window and the right eye into the right half.
#pragma once
#include <glm/glm.hpp>

#include "dk2_optics.hpp"

class StereoRig {
public:
    // render_scale multiplies the per-eye texture size (1.0 = one texel per panel pixel).
    bool init(const Dk2Optics& optics, int window_w, int window_h, float render_scale);
    void shutdown();

    // Bind the eye's framebuffer, set the viewport and clear it. Draw the scene, then call present().
    void begin_eye(int eye, const glm::vec4& clear);
    // Apply lens distortion and write both eyes to the default framebuffer (window of size w x h).
    void present(int window_w, int window_h);

    // --no-vr: draw one ordinary perspective view straight into the window (no lens distortion, no second eye).
    void set_flat(bool flat) { flat_ = flat; }
    bool flat() const { return flat_; }
    int eye_count() const { return flat_ ? 1 : 2; }

    glm::mat4 projection(int eye) const;
    // Eye offset (translation by +-IPD/2), to be multiplied on the left of the camera's view matrix.
    glm::mat4 eye_offset(int eye) const { return flat_ ? glm::mat4(1.0f) : mv_[eye]; }

private:
    unsigned fbo_[2] = {0, 0}, color_[2] = {0, 0}, depth_[2] = {0, 0};
    int eye_w_ = 0, eye_h_ = 0;
    bool flat_ = false;
    unsigned prog_ = 0, vao_ = 0, vbo_ = 0;
    glm::mat4 proj_[2], mv_[2];
    // Distortion parameters
    float viewport_scale_[2], lens_center_[2][2], warp_scale_, k_[4], aberr_[3];
};
