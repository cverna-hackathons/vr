// A unit cube with per-face colours and simple directional lighting.
#pragma once
#include <glm/glm.hpp>

class Cube {
public:
    bool init();
    // Puts an image on the +Z face (centre-cropped to a square, upright). Returns false if it can't be loaded.
    bool load_texture(const char* path);
    // Video on the +Y (top) face: call once with the frame size, then upload_video() for each new RGB frame.
    void init_video(int w, int h);
    void upload_video(const unsigned char* rgb);
    void shutdown();
    // Draws the cube (edge length 1, centred on the origin, transformed by `model`).
    void draw(const glm::mat4& view, const glm::mat4& proj, const glm::mat4& model = glm::mat4(1.0f));

private:
    unsigned prog_ = 0, vao_ = 0, vbo_ = 0, tex_ = 0, vtex_ = 0;
    int vw_ = 0, vh_ = 0;
    int loc_vaspect_ = -1;
    float uv_crop_[2] = {1.0f, 1.0f};  // fraction of the image used along u and v
    int loc_mvp_ = -1, loc_model_ = -1;
};
