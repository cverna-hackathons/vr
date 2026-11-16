// Theater: a big virtual cinema screen in front of the viewer, playing a movie with sound on the DK2.
//
// Usage: theater [--video FILE] [--display N] [--no-setup] [--predict MS] [--no-camera] [--leds FILE]
//   --video FILE   movie to play (default media/video.mp4)
// Keys: Space/P pause, Left/Right seek -/+10 s (Shift: 60 s), Up/Down volume, M mute, C re-centre,
//       F1/F2/F3 flip tracked position X/Y/Z, Esc quit.
#define GL_GLEXT_PROTOTYPES 1
#include <SDL.h>
#include <SDL_opengl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dk2_display.hpp"
#include "head_tracker.hpp"
#include "movie.hpp"
#include "stereo.hpp"

namespace {
const char* kQuadVert = R"(#version 330 core
layout(location=0) in vec2 pos;   // unit quad, -0.5..0.5
uniform mat4 mvp;
out vec2 uv;
out vec2 world_xz;
uniform mat4 model;
void main() {
    uv = pos + 0.5;
    vec4 w = model * vec4(pos, 0.0, 1.0);
    world_xz = w.xz;
    gl_Position = mvp * vec4(pos, 0.0, 1.0);
}
)";
const char* kQuadFrag = R"(#version 330 core
in vec2 uv;
in vec2 world_xz;
uniform sampler2D tex;
uniform vec4 color;
uniform int mode;   // 0 solid colour, 1 video texture, 2 floor grid
out vec4 out_color;
void main() {
    if (mode == 1) {
        out_color = vec4(texture(tex, vec2(uv.x, 1.0 - uv.y)).rgb, 1.0);
    } else if (mode == 2) {
        vec2 g = abs(fract(world_xz) - 0.5);
        float line = 1.0 - smoothstep(0.0, 0.03, 0.5 - max(g.x, g.y));
        float fade = clamp(1.0 - length(world_xz - vec2(0.0, -2.0)) / 14.0, 0.0, 1.0);
        out_color = vec4(vec3(0.10, 0.12, 0.16) * line * fade + vec3(0.012), 1.0);
    } else {
        out_color = color;
    }
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
        std::fprintf(stderr, "Shader error: %s\n", log);
        return 0;
    }
    return s;
}

bool looks_like_dk2(const SDL_Rect& r) { return (r.w == 1080 && r.h == 1920) || (r.w == 1920 && r.h == 1080); }

int pick_display(int requested) {
    const int count = SDL_GetNumVideoDisplays();
    if (count < 1) return 0;
    if (requested >= 0 && requested < count) return requested;
    for (int i = count - 1; i > 0; --i) {
        SDL_Rect r;
        if (SDL_GetDisplayBounds(i, &r) == 0 && looks_like_dk2(r)) return i;
    }
    return count - 1;
}

std::string clock_text(double s) {
    const int t = int(s + 0.5);
    char b[32];
    std::snprintf(b, sizeof b, "%d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
    return b;
}
}  // namespace

int main(int argc, char** argv) {
    std::string video_path = "media/video.mp4", led_file;
    int requested = -1;
    bool use_camera = true, skip_setup = false;
    float predict_ms = 20.0f;  // head-pose prediction horizon (see HeadTracker::set_prediction)
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--video") && i + 1 < argc) video_path = argv[++i];
        else if (!std::strcmp(argv[i], "--display") && i + 1 < argc) requested = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--leds") && i + 1 < argc) led_file = argv[++i];
        else if (!std::strcmp(argv[i], "--no-setup")) skip_setup = true;
        else if (!std::strcmp(argv[i], "--predict") && i + 1 < argc) predict_ms = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--no-camera")) use_camera = false;
        else { std::fprintf(stderr, "Usage: %s [--video FILE] [--display N] [--no-setup] [--predict MS] [--no-camera] [--leds FILE]\n", argv[0]); return 2; }
    }

    if (!skip_setup) ensure_dk2_display();
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) { std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1; }

    if (led_file.empty()) {
        led_file = "data/dk2_leds.txt";
        if (FILE* f = std::fopen(led_file.c_str(), "r")) std::fclose(f);
        else if (char* base = SDL_GetBasePath()) { led_file = std::string(base) + "../data/dk2_leds.txt"; SDL_free(base); }
    }
    HeadTracker tracker;
    const bool tracking_ok = tracker.start(led_file, use_camera);
    tracker.set_prediction(predict_ms / 1000.0f);
    if (!tracking_ok) std::fprintf(stderr, "Warning: DK2 not opened; static view.\n");

    const int disp = pick_display(requested);
    std::printf("Using display %d\n", disp);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_Window* win = SDL_CreateWindow("theater", SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp), SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp),
                                       1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!win) { std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl) { std::fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetSwapInterval(1);

    StereoRig rig;
    {
        int w0, h0;
        SDL_GL_GetDrawableSize(win, &w0, &h0);
        if (!rig.init(Dk2Optics(), w0, h0, 1.0f)) { std::fprintf(stderr, "Stereo setup failed.\n"); return 1; }
    }

    // --- Scene geometry: one unit quad reused for screen, frame, floor and bars ---
    GLuint prog = glCreateProgram();
    {
        GLuint vs = compile(GL_VERTEX_SHADER, kQuadVert), fs = compile(GL_FRAGMENT_SHADER, kQuadFrag);
        if (!vs || !fs) return 1;
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
    }
    const GLint loc_mvp = glGetUniformLocation(prog, "mvp"), loc_model = glGetUniformLocation(prog, "model"),
                loc_color = glGetUniformLocation(prog, "color"), loc_mode = glGetUniformLocation(prog, "mode");
    GLuint vao, vbo;
    {
        const float quad[12] = {-0.5f, -0.5f, 0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 0.5f};
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof quad, quad, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    }
    GLuint tex = 0;
    glGenTextures(1, &tex);

    // --- Movie ---
    MoviePlayer movie;
    std::string vp = video_path;
    if (FILE* f = std::fopen(vp.c_str(), "r")) std::fclose(f);
    else if (char* base = SDL_GetBasePath()) { vp = std::string(base) + "../" + video_path; SDL_free(base); }
    const bool movie_ok = movie.open(vp);
    float screen_w = 6.0f, screen_h = 6.0f * 9.0f / 16.0f;
    if (movie_ok) {
        screen_h = screen_w * float(movie.height()) / float(movie.width());
        glBindTexture(GL_TEXTURE_2D, tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, movie.width(), movie.height(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    const glm::vec3 screen_c(0.0f, 0.2f, -4.5f);
    const float floor_y = screen_c.y - screen_h * 0.5f - 0.15f;
    std::vector<uint8_t> frame;
    uint32_t osd_until = 0;

    bool running = true;
    uint32_t last_print = 0;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type != SDL_KEYDOWN) continue;
            const SDL_Keycode k = e.key.keysym.sym;
            const bool shift = (e.key.keysym.mod & KMOD_SHIFT) != 0;
            if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {  // tune the prediction by feel: too little = lag, too much = overshoot
                tracker.set_prediction(tracker.prediction() + (k == SDLK_RIGHTBRACKET ? 0.002f : -0.002f));
                std::printf("Prediction %.0f ms\n", tracker.prediction() * 1000.0f);
            }
            if (k == SDLK_ESCAPE) running = false;
            else if (k == SDLK_c) { tracker.recentre(); std::printf("Re-centred.\n"); }
            else if (k == SDLK_SPACE || k == SDLK_p) { movie.toggle_pause(); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_RIGHT) { movie.seek_relative(shift ? 60 : 10); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_LEFT) { movie.seek_relative(shift ? -60 : -10); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_UP) { movie.set_volume(movie.volume() + 0.1f); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_DOWN) { movie.set_volume(movie.volume() - 0.1f); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_m) { movie.set_volume(movie.volume() > 0 ? 0.0f : 1.0f); osd_until = SDL_GetTicks() + 3000; }
            else if (k == SDLK_F1 || k == SDLK_F2 || k == SDLK_F3) {
                const int ax = k == SDLK_F1 ? 0 : (k == SDLK_F2 ? 1 : 2);
                tracker.flip_position_axis(ax);
            }
        }

        glm::quat head_q(1, 0, 0, 0);
        glm::vec3 head_pos(0.0f);
        if (tracking_ok) {
            tracker.update();
            head_q = tracker.orientation();
            head_pos = tracker.position();
        }
        const glm::mat4 view = glm::inverse(glm::translate(glm::mat4(1.0f), head_pos) * glm::mat4_cast(head_q));

        if (movie_ok) movie.service();   // reconnect the audio if the device was taken away or changed
        if (movie_ok && movie.take_frame(frame)) {
            glBindTexture(GL_TEXTURE_2D, tex);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, movie.width(), movie.height(), GL_RGB, GL_UNSIGNED_BYTE, frame.data());
        }

        int w, h;
        SDL_GL_GetDrawableSize(win, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);

        const uint32_t now = SDL_GetTicks();
        const bool show_osd = now < osd_until || movie.paused();
        for (int eye = 0; eye < 2; ++eye) {
            rig.begin_eye(eye, glm::vec4(0.01f, 0.01f, 0.012f, 1.0f));
            glDisable(GL_CULL_FACE);
            const glm::mat4 vp_mat = rig.projection(eye) * rig.eye_offset(eye) * view;
            glUseProgram(prog);
            glBindVertexArray(vao);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex);
            glUniform1i(glGetUniformLocation(prog, "tex"), 0);

            auto draw = [&](const glm::mat4& model, int mode, glm::vec4 color) {
                const glm::mat4 mvp = vp_mat * model;
                glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
                glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(model));
                glUniform4fv(loc_color, 1, glm::value_ptr(color));
                glUniform1i(loc_mode, mode);
                glDrawArrays(GL_TRIANGLES, 0, 6);
            };
            // Floor (lies in XZ: rotate the unit quad about X).
            glm::mat4 fm = glm::translate(glm::mat4(1.0f), glm::vec3(0, floor_y, -2.0f));
            fm = glm::rotate(fm, glm::radians(-90.0f), glm::vec3(1, 0, 0));
            fm = glm::scale(fm, glm::vec3(40.0f, 40.0f, 1.0f));
            draw(fm, 2, glm::vec4(0));
            // Screen frame and screen.
            draw(glm::scale(glm::translate(glm::mat4(1.0f), screen_c + glm::vec3(0, 0, -0.02f)),
                            glm::vec3(screen_w + 0.12f, screen_h + 0.12f, 1.0f)), 0, glm::vec4(0.02f, 0.02f, 0.025f, 1));
            draw(glm::scale(glm::translate(glm::mat4(1.0f), screen_c), glm::vec3(screen_w, screen_h, 1.0f)),
                 movie_ok ? 1 : 0, glm::vec4(0.1f, 0.1f, 0.1f, 1));
            // Progress and volume bars under the screen.
            if (show_osd && movie_ok) {
                const float y = screen_c.y - screen_h * 0.5f - 0.3f, z = screen_c.z + 0.01f;
                const float frac = movie.duration() > 0 ? float(std::min(1.0, movie.position() / movie.duration())) : 0.0f;
                const float left = -screen_w * 0.5f;
                draw(glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(0, y, z)), glm::vec3(screen_w, 0.06f, 1)), 0,
                     glm::vec4(0.12f, 0.12f, 0.14f, 1));
                if (frac > 0.0f)
                    draw(glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(left + frac * screen_w * 0.5f, y, z + 0.005f)),
                                    glm::vec3(frac * screen_w, 0.06f, 1)), 0, glm::vec4(0.75f, 0.75f, 0.8f, 1));
                const float vw = screen_w * 0.25f, vy = y - 0.15f, vf = std::min(movie.volume(), 1.0f);
                draw(glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(left + vw * 0.5f, vy, z)), glm::vec3(vw, 0.04f, 1)), 0,
                     glm::vec4(0.12f, 0.12f, 0.14f, 1));
                if (vf > 0.0f)
                    draw(glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(left + vf * vw * 0.5f, vy, z + 0.005f)),
                                    glm::vec3(vf * vw, 0.04f, 1)), 0, glm::vec4(0.35f, 0.7f, 0.4f, 1));
                if (movie.paused())  // pause marker: two bars to the right of the volume bar
                    for (int i = 0; i < 2; ++i)
                        draw(glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(left + vw + 0.2f + i * 0.12f, vy, z)),
                                        glm::vec3(0.06f, 0.14f, 1)), 0, glm::vec4(0.9f, 0.9f, 0.9f, 1));
            }
        }
        rig.present(w, h);
        SDL_GL_SwapWindow(win);

        if (now - last_print > 5000) {
            std::printf("%s / %s%s  vol %.0f%%%s\n", clock_text(movie.position()).c_str(), clock_text(movie.duration()).c_str(),
                        movie.paused() ? " (paused)" : "", movie.volume() * 100.0f,
                        tracking_ok && tracker.camera_active() ? (tracker.camera_tracking() ? "  PTC tracking" : "  PTC lost") : "");
            last_print = now;
        }
    }

    movie.close();
    glDeleteTextures(1, &tex);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(prog);
    rig.shutdown();
    tracker.stop();
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
