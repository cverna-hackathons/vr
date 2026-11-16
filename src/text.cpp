#define GL_GLEXT_PROTOTYPES 1
#include "text.hpp"

#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdio>
#include <vector>

#define STB_TRUETYPE_IMPLEMENTATION
#include "../third_party/stb_truetype.h"

namespace {
constexpr int kAtlas = 1024;
constexpr int kFirst = 32, kCount = 95;  // printable ASCII

const char* kVert = R"(#version 330 core
layout(location=0) in vec2 pos;   // pixels
layout(location=1) in vec2 uv;
uniform vec2 screen;
out vec2 v_uv;
void main() {
    vec2 ndc = vec2(pos.x / screen.x * 2.0 - 1.0, 1.0 - pos.y / screen.y * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    v_uv = uv;
}
)";

const char* kFrag = R"(#version 330 core
in vec2 v_uv;
uniform sampler2D atlas;
uniform vec3 color;
out vec4 out_color;
void main() { out_color = vec4(color, texture(atlas, v_uv).r); }
)";

GLuint compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::fprintf(stderr, "Shader compile error: %s\n", log);
        return 0;
    }
    return s;
}
}  // namespace

struct TextRenderer::Impl {
    stbtt_bakedchar glyphs[kCount];
};

bool TextRenderer::init(const std::string& ttf_path, float pixel_height) {
    std::vector<unsigned char> ttf;
    if (SDL_RWops* f = SDL_RWFromFile(ttf_path.c_str(), "rb")) {
        Sint64 n = SDL_RWsize(f);
        ttf.resize(static_cast<size_t>(n));
        SDL_RWread(f, ttf.data(), 1, ttf.size());
        SDL_RWclose(f);
    } else {
        std::fprintf(stderr, "Cannot open font %s\n", ttf_path.c_str());
        return false;
    }

    impl_ = new Impl;
    px_ = pixel_height;
    std::vector<unsigned char> bitmap(kAtlas * kAtlas);
    if (stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, bitmap.data(), kAtlas, kAtlas,
                             kFirst, kCount, impl_->glyphs) <= 0) {
        std::fprintf(stderr, "Font atlas too small for size %.0f\n", pixel_height);
        return false;
    }

    glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, kAtlas, kAtlas, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    GLuint vs = compile(GL_VERTEX_SHADER, kVert), fs = compile(GL_FRAGMENT_SHADER, kFrag);
    if (!vs || !fs) return false;
    prog_ = glCreateProgram();
    glAttachShader(prog_, vs);
    glAttachShader(prog_, fs);
    glLinkProgram(prog_);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(prog_, GL_LINK_STATUS, &ok);
    if (!ok) { std::fprintf(stderr, "Shader link failed\n"); return false; }
    loc_screen_ = glGetUniformLocation(prog_, "screen");
    loc_color_ = glGetUniformLocation(prog_, "color");
    glUseProgram(prog_);
    glUniform1i(glGetUniformLocation(prog_, "atlas"), 0);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    return true;
}

void TextRenderer::shutdown() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (tex_) glDeleteTextures(1, &tex_);
    if (prog_) glDeleteProgram(prog_);
    delete impl_;
    impl_ = nullptr;
    vbo_ = vao_ = tex_ = prog_ = 0;
}

float TextRenderer::measure(const std::string& text) const {
    float w = 0;
    for (unsigned char c : text)
        if (c >= kFirst && c < kFirst + kCount) w += impl_->glyphs[c - kFirst].xadvance;
    return w;
}

void TextRenderer::draw(const std::string& text, float x, float y, float sw, float sh,
                        float r, float g, float b) {
    std::vector<float> v;
    v.reserve(text.size() * 24);
    float cx = x, cy = y;
    for (unsigned char c : text) {
        if (c < kFirst || c >= kFirst + kCount) continue;
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(impl_->glyphs, kAtlas, kAtlas, c - kFirst, &cx, &cy, &q, 1);
        const float t[6][4] = {{q.x0, q.y0, q.s0, q.t0}, {q.x1, q.y0, q.s1, q.t0}, {q.x1, q.y1, q.s1, q.t1},
                               {q.x0, q.y0, q.s0, q.t0}, {q.x1, q.y1, q.s1, q.t1}, {q.x0, q.y1, q.s0, q.t1}};
        for (auto& vert : t) v.insert(v.end(), vert, vert + 4);
    }
    if (v.empty()) return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(prog_);
    glUniform2f(loc_screen_, sw, sh);
    glUniform3f(loc_color_, r, g, b);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(v.size() / 4));
    glDisable(GL_BLEND);
}
