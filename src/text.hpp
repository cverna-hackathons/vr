// Minimal GL 3.3 text renderer: bakes an ASCII font atlas with stb_truetype and draws
// textured quads in pixel coordinates (origin top-left). Flat colour, no kerning/shaping.
#pragma once
#include <string>

class TextRenderer {
public:
    // pixel_height: glyph size baked into the atlas. Returns false (with message on stderr) on failure.
    bool init(const std::string& ttf_path, float pixel_height);
    void shutdown();
    // Width in pixels of `text` at the baked size.
    float measure(const std::string& text) const;
    // Draw with baseline-left at (x, y). Screen is `screen_w` x `screen_h` pixels.
    void draw(const std::string& text, float x, float y, float screen_w, float screen_h,
              float r, float g, float b);
    float pixel_height() const { return px_; }

private:
    unsigned prog_ = 0, vao_ = 0, vbo_ = 0, tex_ = 0;
    int loc_screen_ = -1, loc_color_ = -1;
    float px_ = 0;
    struct Impl;
    Impl* impl_ = nullptr;
};
