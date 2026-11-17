#define GL_GLEXT_PROTOTYPES 1
#include "stereo.hpp"

#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdio>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


// Distortion shaders (from OpenHMD's GLSL 3.30 sources): warp a per-eye texture through the lens model.
static const char* kDistortVert = R"(#version 330
layout (location=0) in vec2 coords;
uniform mat4 mvp;
out vec2 T;
void main(void) {
    T = coords;
    gl_Position = mvp * vec4(coords, 0.0, 1.0);
}
)";

static const char* kDistortFrag = R"(#version 330
uniform sampler2D warpTexture;
uniform vec2 LensCenter;      // lens centre in metres
uniform vec2 ViewportScale;   // texture coords -> metres
uniform float WarpScale;      // overall distortion scale in metres
uniform vec4 HmdWarpParam;    // PanoTools coefficients [a,b,c,d]
uniform vec3 aberr;           // chromatic scale per channel
in vec2 T;
out vec4 color;
void main() {
    vec2 r = T * ViewportScale - LensCenter;
    r /= WarpScale;
    float r_mag = length(r);
    vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
                            HmdWarpParam.y * r_mag * r_mag +
                            HmdWarpParam.x * r_mag * r_mag * r_mag);
    r_displaced *= WarpScale;
    vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
    vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
    vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;
    float red = texture(warpTexture, tc_r).r;
    float green = texture(warpTexture, tc_g).g;
    float blue = texture(warpTexture, tc_b).b;
    color = ((tc_g.x < 0.0) || (tc_g.x > 1.0) || (tc_g.y < 0.0) || (tc_g.y > 1.0)) ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(red, green, blue, 1.0);
}
)";

static GLuint compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::fprintf(stderr, "Distortion shader compile error: %s\n", log);
        return 0;
    }
    return s;
}

bool StereoRig::init(const Dk2Optics& optics, int window_w, int window_h, float scale) {
    eye_w_ = static_cast<int>(window_w / 2 * scale);
    eye_h_ = static_cast<int>(window_h * scale);

    for (int e = 0; e < 2; ++e) { proj_[e] = optics.projection(e); mv_[e] = optics.eye_offset(e); }

    const float hw = optics.screen_w_m, hh = optics.screen_h_m, sep = optics.lens_separation_m, vpos = optics.lens_vertical_pos_m;
    std::memcpy(k_, optics.distortion_k, sizeof k_);
    std::memcpy(aberr_, optics.aberration, sizeof aberr_);
    viewport_scale_[0] = hw / 2.0f;
    viewport_scale_[1] = hh;
    lens_center_[0][0] = viewport_scale_[0] - sep / 2.0f;  // left eye: lens centre is toward the nose (right side)
    lens_center_[0][1] = vpos;
    lens_center_[1][0] = sep / 2.0f;                       // right eye: toward the nose (left side)
    lens_center_[1][1] = vpos;
    warp_scale_ = lens_center_[0][0] > lens_center_[1][0] ? lens_center_[0][0] : lens_center_[1][0];
    std::printf("Stereo: eye texture %dx%d, screen %.1f x %.1f mm, lens sep %.1f mm\n",
                eye_w_, eye_h_, hw * 1000, hh * 1000, sep * 1000);

    // Per-eye framebuffers
    for (int e = 0; e < 2; ++e) {
        glGenTextures(1, &color_[e]);
        glBindTexture(GL_TEXTURE_2D, color_[e]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, eye_w_, eye_h_, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glGenRenderbuffers(1, &depth_[e]);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_[e]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, eye_w_, eye_h_);
        glGenFramebuffers(1, &fbo_[e]);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_[e]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_[e], 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_[e]);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::fprintf(stderr, "Eye framebuffer %d incomplete\n", e);
            return false;
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    GLuint vs = compile(GL_VERTEX_SHADER, kDistortVert), fs = compile(GL_FRAGMENT_SHADER, kDistortFrag);
    if (!vs || !fs) return false;
    prog_ = glCreateProgram();
    glAttachShader(prog_, vs);
    glAttachShader(prog_, fs);
    glLinkProgram(prog_);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(prog_, GL_LINK_STATUS, &ok);
    if (!ok) { std::fprintf(stderr, "Distortion program link failed\n"); return false; }

    const float quad[] = {0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1};
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof quad, quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    return true;
}

void StereoRig::shutdown() {
    for (int e = 0; e < 2; ++e) {
        if (fbo_[e]) glDeleteFramebuffers(1, &fbo_[e]);
        if (color_[e]) glDeleteTextures(1, &color_[e]);
        if (depth_[e]) glDeleteRenderbuffers(1, &depth_[e]);
        fbo_[e] = color_[e] = depth_[e] = 0;
    }
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (prog_) glDeleteProgram(prog_);
    vbo_ = vao_ = prog_ = 0;
}

glm::mat4 StereoRig::projection(int eye) const {
    if (!flat_) return proj_[eye];
    int w = 16, h = 9;
    if (SDL_Window* win = SDL_GL_GetCurrentWindow()) SDL_GL_GetDrawableSize(win, &w, &h);
    return glm::perspective(glm::radians(75.0f), float(w) / float(h > 0 ? h : 1), 0.05f, 100.0f);
}

void StereoRig::begin_eye(int eye, const glm::vec4& clear) {
    if (flat_) {
        int w = 1, h = 1;
        if (SDL_Window* win = SDL_GL_GetCurrentWindow()) SDL_GL_GetDrawableSize(win, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(clear.r, clear.g, clear.b, clear.a);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_[eye]);
    glViewport(0, 0, eye_w_, eye_h_);
    glEnable(GL_DEPTH_TEST);
    glClearColor(clear.r, clear.g, clear.b, clear.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void StereoRig::present(int w, int h) {
    if (flat_) return;   // already in the window
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(prog_);
    const glm::mat4 mvp = glm::ortho(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);
    glUniformMatrix4fv(glGetUniformLocation(prog_, "mvp"), 1, GL_FALSE, glm::value_ptr(mvp));
    glUniform1i(glGetUniformLocation(prog_, "warpTexture"), 0);
    glUniform2fv(glGetUniformLocation(prog_, "ViewportScale"), 1, viewport_scale_);
    glUniform1f(glGetUniformLocation(prog_, "WarpScale"), warp_scale_);
    glUniform4fv(glGetUniformLocation(prog_, "HmdWarpParam"), 1, k_);
    glUniform3fv(glGetUniformLocation(prog_, "aberr"), 1, aberr_);

    glBindVertexArray(vao_);
    glActiveTexture(GL_TEXTURE0);
    for (int e = 0; e < 2; ++e) {
        glViewport(e * (w / 2), 0, w / 2, h);
        glUniform2fv(glGetUniformLocation(prog_, "LensCenter"), 1, lens_center_[e]);
        glBindTexture(GL_TEXTURE_2D, color_[e]);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }
}
