#define STB_TRUETYPE_IMPLEMENTATION
#include "../third_party/stb_truetype.h"

#include "testpattern.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace {
void rect(std::vector<uint8_t>& px, int w, int h, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
    for (int y = std::max(0, y0); y < std::min(h, y1); ++y)
        for (int x = std::max(0, x0); x < std::min(w, x1); ++x) {
            uint8_t* p = &px[(size_t(y) * w + x) * 4];
            p[0] = b; p[1] = g; p[2] = r; p[3] = 255;
        }
}
}  // namespace

std::vector<uint8_t> make_test_desktop(int w, int h) {
    std::vector<uint8_t> px(size_t(w) * h * 4, 0);
    rect(px, w, h, 0, 0, w, h, 38, 40, 48);               // desktop
    rect(px, w, h, 0, 0, w, 28, 24, 25, 30);              // top panel
    rect(px, w, h, 120, 60, w - 120, h - 60, 245, 245, 245);   // a big white "window"
    rect(px, w, h, 120, 60, w - 120, 90, 60, 90, 160);    // its title bar

    std::vector<uint8_t> ttf;
    for (const char* path : {"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"}) {
        if (FILE* f = std::fopen(path, "rb")) {
            std::fseek(f, 0, SEEK_END);
            ttf.resize(size_t(std::ftell(f)));
            std::fseek(f, 0, SEEK_SET);
            if (std::fread(ttf.data(), 1, ttf.size(), f) != ttf.size()) ttf.clear();
            std::fclose(f);
            if (!ttf.empty()) break;
        }
    }
    if (ttf.empty()) return px;

    auto draw_lines = [&](float pixel_height, int x, int y0, int line_h, int lines, uint8_t fg, const char* prefix) {
        std::vector<uint8_t> bitmap(1024 * 1024);
        std::vector<stbtt_bakedchar> cdata(96);
        stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, bitmap.data(), 1024, 1024, 32, 96, cdata.data());
        const char* words[] = {"The", "quick", "brown", "fox", "jumps", "over", "the", "lazy", "dog.", "Virtual", "monitor", "text", "should", "be",
                               "readable", "inside", "the", "lens", "even", "when", "small", "0123456789", "{}[]()<>;:", "gaze"};
        for (int line = 0; line < lines; ++line) {
            std::string text = std::string(prefix) + std::to_string(line + 1) + "  ";
            for (int k = 0; k < 11; ++k) text += std::string(words[(line * 7 + k * 3) % 24]) + " ";
            float cx = float(x), cy = float(y0 + line * line_h);
            for (char c : text) {
                if (c < 32) continue;
                stbtt_aligned_quad q;
                stbtt_GetBakedQuad(cdata.data(), 1024, 1024, c - 32, &cx, &cy, &q, 1);
                const int ix0 = int(q.x0 + 0.5f), iy0 = int(q.y0 + 0.5f);
                const int gw = int(cdata[size_t(c - 32)].x1 - cdata[size_t(c - 32)].x0), gh = int(cdata[size_t(c - 32)].y1 - cdata[size_t(c - 32)].y0);
                for (int yy = 0; yy < gh; ++yy)
                    for (int xx = 0; xx < gw; ++xx) {
                        const int dx = ix0 + xx, dy = iy0 + yy;
                        if (dx < 0 || dy < 0 || dx >= w || dy >= h) continue;
                        const float a = bitmap[size_t(int(cdata[size_t(c - 32)].y0) + yy) * 1024 + size_t(int(cdata[size_t(c - 32)].x0) + xx)] / 255.0f;
                        uint8_t* p = &px[(size_t(dy) * w + dx) * 4];
                        for (int ch = 0; ch < 3; ++ch) p[ch] = uint8_t(p[ch] * (1.0f - a) + fg * a);
                    }
            }
        }
    };
    draw_lines(13.0f, 140, 120, 17, 40, 20, "log ");       // typical small UI text
    draw_lines(20.0f, 140, 820, 28, 8, 20, "title ");       // larger text lower down
    return px;
}
