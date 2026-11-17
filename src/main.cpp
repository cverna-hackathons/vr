// Modes (keys 2-5, default 5):
//   2  colour shift only   3  colour shift + text   4  stereo isometric cube, fixed camera
//   5  same cube, camera follows the headset (look around, roll) + keyboard movement
// Colour modes: dark blue by default; turning the head left fades to dark red, right to dark green
// (full colour at 45 degrees of yaw relative to the centre pose). Flat white text is drawn on top.
//
// Tracking: the headset's IMU gives orientation; the PTC camera (if connected, with its sync cable) adds position and
// removes yaw drift. Without the camera, only orientation is tracked.
//
// Usage: vr [--display N] [--font FILE.ttf] [--no-setup] [--predict MS] [--no-camera] [--no-vr] [--leds FILE] [--texture IMG] [--video FILE] [--list]
//   --no-camera  orientation-only tracking (do not use the PTC)
//   --leds FILE  LED layout file (default: data/dk2_leds.txt, made by build/ledlayout)
//   --list       print detected displays and exit
//   --display N  use display N (default: auto-detect the DK2, else the last display)
//   --texture IMG  image for the cube's front face (default media/lady.jpg)
//   --video FILE   looping video for the cube's top face (default media/video.mp4)
//   --font FILE  TTF font for the text (default: DejaVu Sans / Liberation Sans)
// Keys: Esc quit, 2-5 mode, Space/C re-centre (current heading becomes "straight ahead").
//       F1/F2/F3 flip the sign of tracked head position on X/Y/Z (diagnostic; state is printed).
//       Mode 5: W/S forward/back, A/D strafe, Q/E down/up, R reset keyboard offset (hold Shift = slow).
//               Physically leaning/moving is tracked by the PTC and adds to the keyboard offset.

#define GL_GLEXT_PROTOTYPES 1
#include <SDL.h>
#include <SDL_opengl.h>
#include <string>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>
#include "cube.hpp"
#include "video.hpp"
#include "dk2_display.hpp"
#include "head_tracker.hpp"
#include "novr.hpp"
#include "stereo.hpp"
#include "text.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static constexpr float kPi = 3.14159265358979f;
static constexpr float kFullColourDeg = 45.0f;
static constexpr float kBrightness = 0.25f;  // "dark" colours: max channel value

// The DK2 panel is 1080x1920 natively; X may report it as either 1080x1920 or 1920x1080.
static bool looks_like_dk2(const SDL_Rect& r) {
    return (r.w == 1080 && r.h == 1920) || (r.w == 1920 && r.h == 1080);
}

static int pick_display(int requested) {
    int count = SDL_GetNumVideoDisplays();
    if (count < 1) return 0;
    if (requested >= 0 && requested < count) return requested;
    for (int i = count - 1; i > 0; --i) {
        SDL_Rect r;
        if (SDL_GetDisplayBounds(i, &r) == 0 && looks_like_dk2(r)) return i;
    }
    return count - 1;
}

// Heading (yaw) in degrees of a headset->world rotation (+Y up, -Z forward). Positive = turned left (CCW from above).
static float yaw_degrees(const glm::quat& q) {
    const glm::vec3 f = q * glm::vec3(0, 0, -1);
    return std::atan2(-f.x, -f.z) * 180.0f / kPi;
}

static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

int main(int argc, char** argv) {
    int requested = -1;
    std::string font_path;
    std::string led_file;
    std::string texture_path = "media/lady.jpg";
    std::string video_path = "media/video.mp4";
    bool use_camera = true, skip_setup = false;
    float predict_ms = 20.0f;  // head-pose prediction horizon (see HeadTracker::set_prediction)
    bool list_only = false, no_vr = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--list")) list_only = true;
        else if (!std::strcmp(argv[i], "--display") && i + 1 < argc) requested = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--font") && i + 1 < argc) font_path = argv[++i];
        else if (!std::strcmp(argv[i], "--leds") && i + 1 < argc) led_file = argv[++i];
        else if (!std::strcmp(argv[i], "--texture") && i + 1 < argc) texture_path = argv[++i];
        else if (!std::strcmp(argv[i], "--video") && i + 1 < argc) video_path = argv[++i];
        else if (!std::strcmp(argv[i], "--no-setup")) skip_setup = true;
        else if (!std::strcmp(argv[i], "--predict") && i + 1 < argc) predict_ms = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--no-camera")) use_camera = false;
        else if (!std::strcmp(argv[i], "--no-vr")) no_vr = true;
        else { std::fprintf(stderr, "Usage: %s [--display N] [--font FILE] [--no-setup] [--predict MS] [--no-camera] [--leds FILE] [--texture IMG] [--video FILE] [--list]\n", argv[0]); return 2; }
    }

    if (!skip_setup && !no_vr) ensure_dk2_display();
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    int count = SDL_GetNumVideoDisplays();
    for (int i = 0; i < count; ++i) {
        SDL_Rect r{};
        SDL_GetDisplayBounds(i, &r);
        const char* name = SDL_GetDisplayName(i);
        std::printf("Display %d: %s  %dx%d at (%d,%d)%s\n", i, name ? name : "?",
                    r.w, r.h, r.x, r.y, looks_like_dk2(r) ? "  <- DK2-sized" : "");
    }
    if (list_only) { SDL_Quit(); return 0; }

    // --- Headset -------------------------------------------------------------------------
    if (led_file.empty()) {
        // Try the working directory first, then next to the executable's parent (build/../data).
        led_file = "data/dk2_leds.txt";
        if (FILE* f = std::fopen(led_file.c_str(), "r")) std::fclose(f);
        else if (char* base = SDL_GetBasePath()) { led_file = std::string(base) + "../data/dk2_leds.txt"; SDL_free(base); }
    }
    HeadTracker tracker;
    const bool tracking_ok = !no_vr && tracker.start(led_file, use_camera);
    tracker.set_prediction(predict_ms / 1000.0f);
    KeyboardHead kb;
    if (no_vr) std::printf("No-VR mode: Ctrl+arrows turn the head, Ctrl+Shift+arrows and Ctrl+PageUp/PageDown lean, Ctrl+Backspace resets (Alt = slow).\n");
    else if (!tracking_ok) std::fprintf(stderr, "Warning: DK2 not opened; showing static view.\n");
    else std::printf("Headset tracking started (camera %s).\n", tracker.camera_active() ? "active" : "off");

    // --- Window --------------------------------------------------------------------------
    int disp = pick_display(requested);
    std::printf("Using display %d\n", disp);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window* win = no_vr
        ? SDL_CreateWindow("vr (no headset)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE)
        : SDL_CreateWindow("vr", SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp), SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp),
                           1920, 1080, SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!win) { std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }

    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl) { std::fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetSwapInterval(1);

    // --- Text ----------------------------------------------------------------------------
    TextRenderer text;
    {
        const char* candidates[] = {"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"};
        bool ok = false;
        if (!font_path.empty()) ok = text.init(font_path, 96.0f);
        for (size_t i = 0; !ok && font_path.empty() && i < 2; ++i) ok = text.init(candidates[i], 96.0f);
        if (!ok) { std::fprintf(stderr, "No usable font; pass --font FILE.ttf\n"); return 1; }
    }

    // --- Stereo + scene --------------------------------------------------------------------
    StereoRig rig;
    Cube cube;
    VideoPlayer video;
    std::vector<uint8_t> video_frame;
    bool stereo_ok = false;
    {
        int w0, h0;
        SDL_GL_GetDrawableSize(win, &w0, &h0);
        stereo_ok = rig.init(Dk2Optics(), w0, h0, 1.0f) && cube.init();
        rig.set_flat(no_vr);
        if (!stereo_ok) std::fprintf(stderr, "Stereo setup failed; cube mode unavailable.\n");
    }
    if (stereo_ok) {
        std::string tp = texture_path;
        if (FILE* f = std::fopen(tp.c_str(), "r")) std::fclose(f);
        else if (char* base = SDL_GetBasePath()) { tp = std::string(base) + "../" + texture_path; SDL_free(base); }
        cube.load_texture(tp.c_str());
        std::string vp = video_path;
        if (FILE* f = std::fopen(vp.c_str(), "r")) std::fclose(f);
        else if (char* base = SDL_GetBasePath()) { vp = std::string(base) + "../" + video_path; SDL_free(base); }
        if (video.start(vp)) cube.init_video(video.width(), video.height());
    }
    int mode = stereo_ok ? 5 : 3;

    // Isometric camera: looks at the origin from direction (1,1,1), 2 m from the cube centre (cube is 1 m per side).
    const glm::vec3 cam_pos = 2.0f * glm::normalize(glm::vec3(1, 1, 1));
    const glm::mat4 cam_view = glm::lookAt(cam_pos, glm::vec3(0), glm::vec3(0, 1, 0));

    // --- Loop ----------------------------------------------------------------------------
    bool running = true;
    Uint32 last_print = 0;
    Uint32 last_tick = SDL_GetTicks();
    glm::vec3 cam_offset(0.0f);                 // keyboard-driven camera translation (world space, metres)
    glm::quat head_q(1, 0, 0, 0);               // latest headset orientation
    // Measured: the camera-derived position axes are opposite to the real motion; flip all three by default.
    // F1/F2/F3 toggle X/Y/Z so the right combination can be found by feel.

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN) {
                SDL_Keycode k = e.key.keysym.sym;
                if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {  // tune the prediction by feel: too little = lag, too much = overshoot
                tracker.set_prediction(tracker.prediction() + (k == SDLK_RIGHTBRACKET ? 0.002f : -0.002f));
                std::printf("Prediction %.0f ms\n", tracker.prediction() * 1000.0f);
            }
            if (k == SDLK_ESCAPE) running = false;
                if (k == SDLK_SPACE || (k == SDLK_c && !(SDL_GetModState() & KMOD_CTRL))) { tracker.recentre(); std::printf("Re-centred.\n"); }
                if (k == SDLK_2) mode = 2;
                if (k == SDLK_3) mode = 3;
                if (k == SDLK_4 && stereo_ok) mode = 4;
                if (k == SDLK_5 && stereo_ok) mode = 5;
                if (k == SDLK_r) cam_offset = glm::vec3(0);
                if (k == SDLK_F1 || k == SDLK_F2 || k == SDLK_F3) {
                    const int ax = k == SDLK_F1 ? 0 : (k == SDLK_F2 ? 1 : 2);
                    tracker.flip_position_axis(ax);
                }
            }
        }

        const Uint32 tick = SDL_GetTicks();
        const float dt = (tick - last_tick) / 1000.0f;
        last_tick = tick;

        float delta = 0.0f;  // degrees; + = left, - = right
        glm::vec3 head_pos(0.0f);
        if (tracking_ok) {
            tracker.update();
            head_q = tracker.orientation();     // already relative to the centre pose
            head_pos = tracker.position();
            delta = wrap180(yaw_degrees(head_q));
        } else if (no_vr) {
            kb.update(dt, head_keys_from_sdl());
            head_q = kb.orientation();
            head_pos = kb.position();
            delta = wrap180(yaw_degrees(head_q));
        }

        int w, h;
        SDL_GL_GetDrawableSize(win, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);

        if (video.take_frame(video_frame)) cube.upload_video(video_frame.data());

        if (mode == 4 || mode == 5) {
            const glm::mat4 model(1.0f);
            glm::mat4 view = cam_view;  // mode 4: fixed isometric camera
            if (mode == 5) {
                // World transform of the isometric camera, with the headset rotation applied in its local frame.
                // Rotation is taken relative to the centre pose so "straight ahead" = the isometric view direction.
                const glm::mat4 rot = glm::mat4_cast(head_q);
                const glm::mat4 base = glm::inverse(cam_view);
                // Tracked head position is expressed in the user's (re-centred) frame, i.e. the camera's local axes.
                const glm::mat4 lean = glm::translate(glm::mat4(1.0f), head_pos);
                glm::mat4 camw = glm::translate(glm::mat4(1.0f), cam_offset) * base * lean * rot;

                // Keyboard movement along the camera's own axes (column 0 = right, 1 = up, 2 = back).
                const Uint8* keys = SDL_GetKeyboardState(nullptr);
                const float speed = (keys[SDL_SCANCODE_LSHIFT] ? 0.4f : 1.5f) * dt;
                const glm::vec3 right = glm::vec3(camw[0]), up = glm::vec3(camw[1]), back = glm::vec3(camw[2]);
                if (keys[SDL_SCANCODE_W]) cam_offset -= back * speed;
                if (keys[SDL_SCANCODE_S]) cam_offset += back * speed;
                if (keys[SDL_SCANCODE_D]) cam_offset += right * speed;
                if (keys[SDL_SCANCODE_A]) cam_offset -= right * speed;
                if (keys[SDL_SCANCODE_E]) cam_offset += up * speed;
                if (keys[SDL_SCANCODE_Q]) cam_offset -= up * speed;

                camw = glm::translate(glm::mat4(1.0f), cam_offset) * base * lean * rot;
                view = glm::inverse(camw);
            }
            for (int eye = 0; eye < rig.eye_count(); ++eye) {
                rig.begin_eye(eye, glm::vec4(0.04f, 0.04f, 0.08f, 1.0f));
                glDisable(GL_CULL_FACE);
                cube.draw(rig.eye_offset(eye) * view, rig.projection(eye), model);
            }
            rig.present(w, h);
        } else {
            // Blend dark blue -> dark red (left) / dark green (right).
            float t = std::min(std::fabs(delta) / kFullColourDeg, 1.0f);
            float r = 0, g = 0, b = kBrightness * (1.0f - t);
            if (delta > 0) r = kBrightness * t; else g = kBrightness * t;

            glDisable(GL_DEPTH_TEST);
            glClearColor(r, g, b, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            if (mode == 3) {
                // The DK2 shows one image split across the two lenses: left half = left eye, right half = right eye.
                // So draw the text once per eye, centred in each half, otherwise it straddles the split.
                const float sw = static_cast<float>(w), sh = static_cast<float>(h);
                const std::string title = "Hello DK2";
                char line[64];
                std::snprintf(line, sizeof line, "Yaw %+.0f deg", delta);
                for (int eye = 0; eye < (no_vr ? 1 : 2); ++eye) {
                    const float cx = no_vr ? sw * 0.5f : sw * (eye == 0 ? 0.25f : 0.75f);  // centre of this eye's half
                    text.draw(title, cx - text.measure(title) * 0.5f, sh * 0.5f - 10.0f, sw, sh, 1, 1, 1);
                    text.draw(line, cx - text.measure(line) * 0.5f, sh * 0.5f + text.pixel_height() + 10.0f, sw, sh, 1, 1, 1);
                }
            }
        }
        SDL_GL_SwapWindow(win);

        Uint32 now = SDL_GetTicks();
        if (tracking_ok && now - last_print > 500) {
            if (tracker.camera_active())
                std::printf("yaw %+7.1f deg | camera %s (%2d LEDs, %.2f px) pos %+.3f %+.3f %+.3f m | yaw drift fix %+.1f deg (mode %d)\n",
                            delta, tracker.camera_tracking() ? "TRACKING" : "lost    ", tracker.camera_leds(), tracker.camera_rms_px(),
                            head_pos.x, head_pos.y, head_pos.z, tracker.yaw_correction_deg(), mode);
            else
                std::printf("yaw offset: %+7.1f deg  (mode %d)\n", delta, mode);
            last_print = now;
        }
    }

    if (stereo_ok) { cube.shutdown(); rig.shutdown(); }
    text.shutdown();
    tracker.stop();
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
