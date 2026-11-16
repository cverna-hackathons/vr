#define GL_GLEXT_PROTOTYPES 1
#include "cube.hpp"

#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <glm/gtc/type_ptr.hpp>
#define STB_IMAGE_IMPLEMENTATION
#include "../third_party/stb_image.h"

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 normal;
layout(location=2) in vec3 color;
layout(location=3) in vec3 uvt;  // u, v, textured flag
uniform mat4 mvp;
uniform mat4 model;
out vec3 v_normal;
out vec3 v_color;
out vec3 v_uvt;
void main() {
    v_uvt = uvt;
    gl_Position = mvp * vec4(pos, 1.0);
    v_normal = mat3(model) * normal;
    v_color = color;
}
)";

const char* kFrag = R"(#version 330 core
in vec3 v_normal;
in vec3 v_color;
in vec3 v_uvt;
uniform sampler2D tex;
uniform sampler2D vtex;
uniform float vaspect;  // video width / height
out vec4 out_color;
void main() {
    vec3 light = normalize(vec3(0.35, 0.8, 0.5));
    float diffuse = max(dot(normalize(v_normal), light), 0.0);
    vec3 base = v_color;
    if (v_uvt.z > 1.5) {  // video face: fit inside the square (letterbox), top of the picture = +v
        vec2 p = v_uvt.xy - 0.5;
        if (vaspect > 1.0) p.y *= vaspect; else p.x /= vaspect;
        p += 0.5;
        base = (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0) ? vec3(0.0) : texture(vtex, vec2(p.x, 1.0 - p.y)).rgb;
    } else if (v_uvt.z > 0.5) {
        base = texture(tex, v_uvt.xy).rgb;
    }
    out_color = vec4(base * (0.45 + 0.55 * diffuse), 1.0);
}
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
        std::fprintf(stderr, "Cube shader compile error: %s\n", log);
        return 0;
    }
    return s;
}
}  // namespace

bool Cube::init() {
    // Six faces: +X red, -X cyan, +Y green, -Y magenta, +Z blue, -Z yellow. pos(3) normal(3) colour(3).
    struct Face { float n[3]; float u[3]; float v[3]; float c[3]; };
    const Face faces[6] = {
        {{ 1, 0, 0}, {0, 0,-1}, {0, 1, 0}, {0.90f, 0.20f, 0.20f}},
        {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0.20f, 0.80f, 0.80f}},
        {{ 0, 1, 0}, {1, 0, 0}, {0, 0,-1}, {0.20f, 0.85f, 0.25f}},
        {{ 0,-1, 0}, {1, 0, 0}, {0, 0, 1}, {0.85f, 0.25f, 0.80f}},
        {{ 0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0.25f, 0.35f, 0.95f}},
        {{ 0, 0,-1}, {-1,0, 0}, {0, 1, 0}, {0.95f, 0.85f, 0.25f}},
    };
    float verts[6 * 6 * 12];
    int k = 0;
    const int corner[6][2] = {{-1,-1}, {1,-1}, {1,1}, {-1,-1}, {1,1}, {-1,1}};
    for (const Face& f : faces) {
        for (const auto& c : corner) {
            for (int i = 0; i < 3; ++i) verts[k++] = 0.5f * (f.n[i] + c[0] * f.u[i] + c[1] * f.v[i]);
            for (int i = 0; i < 3; ++i) verts[k++] = f.n[i];
            for (int i = 0; i < 3; ++i) verts[k++] = f.c[i];
            const float kind = f.n[2] > 0.5f ? 1.0f : (f.n[1] > 0.5f ? 2.0f : 0.0f);  // +Z image, +Y video
            verts[k++] = 0.5f + 0.5f * c[0];
            verts[k++] = 0.5f + 0.5f * c[1];
            verts[k++] = kind;
        }
    }

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
    if (!ok) { std::fprintf(stderr, "Cube program link failed\n"); return false; }
    loc_mvp_ = glGetUniformLocation(prog_, "mvp");
    loc_model_ = glGetUniformLocation(prog_, "model");
    glUseProgram(prog_);
    glUniform1i(glGetUniformLocation(prog_, "tex"), 0);
    glUniform1i(glGetUniformLocation(prog_, "vtex"), 1);
    loc_vaspect_ = glGetUniformLocation(prog_, "vaspect");
    glUniform1f(loc_vaspect_, 1.0f);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof verts, verts, GL_STATIC_DRAW);
    for (int i = 0; i < 4; ++i) {
        glVertexAttribPointer(i, 3, GL_FLOAT, GL_FALSE, 12 * sizeof(float), (void*)(i * 3 * sizeof(float)));
        glEnableVertexAttribArray(i);
    }
    return true;
}

bool Cube::load_texture(const char* path) {
    stbi_set_flip_vertically_on_load(1);  // GL's v axis points up
    int w = 0, h = 0, n = 0;
    unsigned char* px = stbi_load(path, &w, &h, &n, 3);
    if (!px) { std::fprintf(stderr, "Texture '%s': %s\n", path, stbi_failure_reason()); return false; }
    // Centre-crop to a square by sampling only part of the image (done in the UVs below via clamp offsets).
    if (!tex_) glGenTextures(1, &tex_);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // Crop rows/columns on the CPU so the face shows an undistorted square.
    const int side = w < h ? w : h;
    const int x0 = (w - side) / 2, y0 = (h - side) / 2;
    std::vector<unsigned char> sq(size_t(side) * side * 3);
    for (int y = 0; y < side; ++y)
        std::memcpy(&sq[size_t(y) * side * 3], px + (size_t(y + y0) * w + x0) * 3, size_t(side) * 3);
    stbi_image_free(px);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, side, side, 0, GL_RGB, GL_UNSIGNED_BYTE, sq.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    std::printf("Texture: %s (%dx%d, cropped to %dx%d)\n", path, w, h, side, side);
    return true;
}

void Cube::init_video(int w, int h) {
    vw_ = w;
    vh_ = h;
    glGenTextures(1, &vtex_);
    glBindTexture(GL_TEXTURE_2D, vtex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glUseProgram(prog_);
    glUniform1f(loc_vaspect_, float(w) / float(h));
}

void Cube::upload_video(const unsigned char* rgb) {
    if (!vtex_) return;
    glBindTexture(GL_TEXTURE_2D, vtex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, vw_, vh_, GL_RGB, GL_UNSIGNED_BYTE, rgb);
}

void Cube::shutdown() {
    if (vtex_) glDeleteTextures(1, &vtex_);
    vtex_ = 0;
    if (tex_) glDeleteTextures(1, &tex_);
    tex_ = 0;
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (prog_) glDeleteProgram(prog_);
    vbo_ = vao_ = prog_ = 0;
}

void Cube::draw(const glm::mat4& view, const glm::mat4& proj, const glm::mat4& model) {
    glUseProgram(prog_);
    const glm::mat4 mvp = proj * view * model;
    glUniformMatrix4fv(loc_mvp_, 1, GL_FALSE, glm::value_ptr(mvp));
    glUniformMatrix4fv(loc_model_, 1, GL_FALSE, glm::value_ptr(model));
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex_);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, vtex_);
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}
