// Virtual display: mirrors the laptop's built-in screen onto a big monitor floating in front of the viewer.
//
// Usage: display [--source OUTPUT] [--display N] [--fps N] [--arc DEG] [--distance M] [--cursor N] [--no-lens] [--lens-zoom Z] [--lens-size M] [--no-setup] [--predict MS] [--no-camera] [--no-vr] [--leds FILE]
//   --source OUTPUT  RandR output to mirror (default: the built-in eDP output)
//   --display N      SDL display index of the headset (default: auto)
//   --fps N          capture rate (default 60, the headset refresh)
//   --arc DEG        horizontal angle the curved screen spans around you (default 110, range 20-180)
//   --distance M     radius of the curve = distance to the screen, metres (default 1.5)
//   --cursor N       magnification of the mouse pointer in the picture (default 2)
//   --no-lens        start with the gaze magnifier off (it starts on; Ctrl+F12 toggles it); --lens-zoom Z (default 2.2) and --lens-size M (width in metres, default 1.5)
// The gaze lens magnifies the part of the screen you are looking at, at the capture's full resolution, so small text is readable even
// though the DK2 shows only about 10 pixels per degree. Ctrl+F12 toggles it.
//
// Input is confined to the mirrored screen: the pointer cannot leave it (it is drawn in the picture) and keyboard focus is kept
// on a window on that screen, so everything typed or clicked acts on what you see. The headset window never takes focus.
// Global hotkeys (grabbed while running):
//   Ctrl+X       exit      Ctrl+F12   gaze lens on/off
//   Ctrl+F9      re-centre         Ctrl+F10 / Ctrl+F11   screen wider / narrower
#define GL_GLEXT_PROTOTYPES 1
#include <SDL.h>
#include <SDL_opengl.h>
#include <X11/XKBlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/Xlib.h>
#include <X11/extensions/Xfixes.h>

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
#include "gaze.hpp"
#include "head_tracker.hpp"
#include "novr.hpp"
#include "stereo.hpp"
#include "testpattern.hpp"
#include "xcapture.hpp"

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec2 pos;
uniform mat4 mvp;
uniform mat4 model;
uniform vec3 curve;   // x: arc angle (rad), y: radius, z: height; arc > 0 draws a cylinder section around the origin
out vec2 uv;
out vec2 world_xz;
void main() {
    uv = pos + 0.5;
    if (curve.x > 0.0) {
        float a = pos.x * curve.x;
        vec3 p = vec3(curve.y * sin(a), pos.y * curve.z, -curve.y * cos(a));
        world_xz = p.xz;
        gl_Position = mvp * vec4(p, 1.0);
    } else {
        world_xz = (model * vec4(pos, 0.0, 1.0)).xz;
        gl_Position = mvp * vec4(pos, 0.0, 1.0);
    }
}
)";
const char* kFrag = R"(#version 330 core
in vec2 uv;
in vec2 world_xz;
uniform sampler2D tex;
uniform vec4 color;
uniform int mode;   // 0 solid, 1 screen texture, 2 floor grid
uniform vec4 lens;  // x,y: gaze point (screen uv); z: half-width in metres; w: zoom (0 = lens off)
uniform vec2 size;  // screen width / height in metres
out vec4 out_color;
void main() {
    if (mode == 1) {
        vec2 u = uv;
        float ring = 0.0;
        if (lens.w > 0.0) {
            vec2 d = (uv - lens.xy) * size;
            float e = length(vec2(d.x / lens.z, d.y / (lens.z * 0.55)));   // elliptical lens: wide enough for a line of text
            if (e < 1.0) {
                // Soft lens: full zoom in the middle, easing smoothly down to 1x at the rim, so there is no hard border or seam,
                // just a gentle swell of the text under the gaze.
                float z = mix(1.0, lens.w, smoothstep(1.0, 0.8, e));
                u = lens.xy + (uv - lens.xy) / z;                         // magnified: samples a finer mip level, i.e. real detail
                ring = 0.0;
            }
        }
        vec3 c = texture(tex, vec2(u.x, 1.0 - u.y)).rgb;
        out_color = vec4(mix(c, vec3(0.95, 0.85, 0.3), ring), 1.0);
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
    if (!ok) { char log[1024]; glGetShaderInfoLog(s, sizeof log, nullptr, log); std::fprintf(stderr, "Shader error: %s\n", log); return 0; }
    return s;
}

bool looks_like_dk2(const SDL_Rect& r) { return (r.w == 1080 && r.h == 1920) || (r.w == 1920 && r.h == 1080); }

// Headset display: the last DK2-sized display that is not the mirrored source.
int pick_display(int requested, const XRect& src) {
    const int count = SDL_GetNumVideoDisplays();
    if (requested >= 0 && requested < count) return requested;
    for (int i = count - 1; i >= 0; --i) {
        SDL_Rect r;
        if (SDL_GetDisplayBounds(i, &r) != 0 || !looks_like_dk2(r)) continue;
        if (r.x == src.x && r.y == src.y) continue;
        return i;
    }
    return count - 1;
}

constexpr int kCols = 96;
bool g_x_error = false;
int x_error_handler(Display*, XErrorEvent*) { g_x_error = true; return 0; }


// ---- EWMH helpers: keep keyboard focus on a window that is on the mirrored screen ---------------------
std::vector<Window> window_list(Display* d, const char* prop) {
    std::vector<Window> out;
    Atom a = XInternAtom(d, prop, False), type;
    int fmt;
    unsigned long n, after;
    unsigned char* data = nullptr;
    if (XGetWindowProperty(d, DefaultRootWindow(d), a, 0, 4096, False, XA_WINDOW, &type, &fmt, &n, &after, &data) == Success && data) {
        const Window* w = reinterpret_cast<const Window*>(data);
        for (unsigned long i = 0; i < n; ++i) out.push_back(w[i]);
        XFree(data);
    }
    return out;
}

bool window_centre(Display* d, Window w, int& cx, int& cy) {
    XWindowAttributes wa;
    if (!XGetWindowAttributes(d, w, &wa) || wa.map_state != IsViewable) return false;
    Window child;
    int x, y;
    XTranslateCoordinates(d, w, DefaultRootWindow(d), 0, 0, &x, &y, &child);
    cx = x + wa.width / 2;
    cy = y + wa.height / 2;
    return true;
}

bool has_atom(Display* d, Window w, const char* prop, const char* value) {
    Atom type, want = XInternAtom(d, value, False);
    int fmt;
    unsigned long n, after;
    unsigned char* data = nullptr;
    bool found = false;
    if (XGetWindowProperty(d, w, XInternAtom(d, prop, False), 0, 64, False, XA_ATOM, &type, &fmt, &n, &after, &data) == Success && data) {
        const Atom* a = reinterpret_cast<const Atom*>(data);
        for (unsigned long i = 0; i < n; ++i) if (a[i] == want) found = true;
        XFree(data);
    }
    return found;
}

std::string window_title(Display* d, Window w) {
    char* name = nullptr;
    std::string t;
    if (XFetchName(d, w, &name) && name) { t = name; XFree(name); }
    return t;
}

void activate_window(Display* d, Window w);

// If the active window is not on the mirrored screen, activate the top-most normal window that is.
void keep_focus_on_screen(Display* d, const XRect& r) {
    auto inside = [&](int x, int y) { return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h; };
    const std::vector<Window> act = window_list(d, "_NET_ACTIVE_WINDOW");
    int cx, cy;
    if (!act.empty() && act[0] && window_centre(d, act[0], cx, cy) && inside(cx, cy)) return;  // already fine
    const std::vector<Window> stack = window_list(d, "_NET_CLIENT_LIST_STACKING");  // bottom -> top
    for (size_t i = stack.size(); i-- > 0;) {
        const Window w = stack[i];
        if (!window_centre(d, w, cx, cy) || !inside(cx, cy)) continue;
        if (has_atom(d, w, "_NET_WM_STATE", "_NET_WM_STATE_HIDDEN")) continue;
        if (has_atom(d, w, "_NET_WM_WINDOW_TYPE", "_NET_WM_WINDOW_TYPE_DOCK") ||
            has_atom(d, w, "_NET_WM_WINDOW_TYPE", "_NET_WM_WINDOW_TYPE_DESKTOP")) continue;
        activate_window(d, w);
        std::printf("Keyboard focus moved to \"%s\" (on the mirrored screen)\n", window_title(d, w).c_str());
        return;
    }
}

void activate_window(Display* d, Window w) {
    XEvent ev{};
    ev.xclient.type = ClientMessage;
    ev.xclient.window = w;
    ev.xclient.message_type = XInternAtom(d, "_NET_ACTIVE_WINDOW", False);
    ev.xclient.format = 32;
    ev.xclient.data.l[0] = 2;  // request comes from a pager: window managers honour this more readily
    XSendEvent(d, DefaultRootWindow(d), False, SubstructureRedirectMask | SubstructureNotifyMask, &ev);
    XFlush(d);
}

void set_wm_state(Display* d, Window w, bool add, const char* a, const char* b = nullptr) {
    XEvent ev{};
    ev.xclient.type = ClientMessage;
    ev.xclient.window = w;
    ev.xclient.message_type = XInternAtom(d, "_NET_WM_STATE", False);
    ev.xclient.format = 32;
    ev.xclient.data.l[0] = add ? 1 : 0;
    ev.xclient.data.l[1] = long(XInternAtom(d, a, False));
    ev.xclient.data.l[2] = b ? long(XInternAtom(d, b, False)) : 0;
    ev.xclient.data.l[3] = 2;
    XSendEvent(d, DefaultRootWindow(d), False, SubstructureRedirectMask | SubstructureNotifyMask, &ev);
}

void move_window(Display* d, Window w, int x, int y) {
    XEvent ev{};
    ev.xclient.type = ClientMessage;
    ev.xclient.window = w;
    ev.xclient.message_type = XInternAtom(d, "_NET_MOVERESIZE_WINDOW", False);
    ev.xclient.format = 32;
    ev.xclient.data.l[0] = 1 | (1 << 8) | (1 << 9) | (2 << 12);  // NorthWest gravity, x and y valid, source = pager
    ev.xclient.data.l[1] = x;
    ev.xclient.data.l[2] = y;
    XSendEvent(d, DefaultRootWindow(d), False, SubstructureRedirectMask | SubstructureNotifyMask, &ev);
}

// Keeps every interaction on the mirrored screen: windows opened while we run (a video player, a dialog...) are moved
// onto it, because anything left on another display could not be seen or reached with the confined pointer.
// Windows that already existed at start are left alone (they stay where the user put them).
class WindowHerder {
public:
    WindowHerder(Display* d, const XRect& r) : d_(d), r_(r) {
        for (Window w : window_list(d_, "_NET_CLIENT_LIST")) legacy_.push_back(w);
    }
    void tick(uint32_t now) {
        if (now - last_tick_ < 150) return;  // a few times a second is plenty
        last_tick_ = now;
        // Re-apply fullscreen/maximised state to windows we moved (state is tied to the monitor, so it was dropped first).
        for (size_t i = 0; i < restore_.size();) {
            if (now >= restore_[i].when) {
                if (restore_[i].full) set_wm_state(d_, restore_[i].w, true, "_NET_WM_STATE_FULLSCREEN");
                if (restore_[i].max) set_wm_state(d_, restore_[i].w, true, "_NET_WM_STATE_MAXIMIZED_HORZ", "_NET_WM_STATE_MAXIMIZED_VERT");
                XFlush(d_);
                restore_.erase(restore_.begin() + long(i));
            } else ++i;
        }
        for (Window w : window_list(d_, "_NET_CLIENT_LIST")) {
            if (std::find(legacy_.begin(), legacy_.end(), w) != legacy_.end()) continue;
            if (std::find_if(restore_.begin(), restore_.end(), [&](const Restore& r) { return r.w == w; }) != restore_.end()) continue;
            XWindowAttributes wa;
            int cx, cy;
            if (!XGetWindowAttributes(d_, w, &wa) || wa.map_state != IsViewable || !window_centre(d_, w, cx, cy)) continue;
            if (cx >= r_.x && cx < r_.x + r_.w && cy >= r_.y && cy < r_.y + r_.h) {
                if (std::find(seen_.begin(), seen_.end(), w) == seen_.end()) { seen_.push_back(w); activate_window(d_, w); }
                continue;  // already on the mirrored screen (new windows there get the keyboard focus once)
            }
            if (has_atom(d_, w, "_NET_WM_WINDOW_TYPE", "_NET_WM_WINDOW_TYPE_DOCK") ||
                has_atom(d_, w, "_NET_WM_WINDOW_TYPE", "_NET_WM_WINDOW_TYPE_DESKTOP")) continue;
            Restore rs{w, now + 400, has_atom(d_, w, "_NET_WM_STATE", "_NET_WM_STATE_FULLSCREEN"),
                       has_atom(d_, w, "_NET_WM_STATE", "_NET_WM_STATE_MAXIMIZED_HORZ")};
            if (rs.full) set_wm_state(d_, w, false, "_NET_WM_STATE_FULLSCREEN");
            if (rs.max) set_wm_state(d_, w, false, "_NET_WM_STATE_MAXIMIZED_HORZ", "_NET_WM_STATE_MAXIMIZED_VERT");
            const int nx = r_.x + std::max(0, (r_.w - wa.width) / 2), ny = r_.y + std::max(0, (r_.h - wa.height) / 2);
            move_window(d_, w, nx, ny);
            activate_window(d_, w);
            XFlush(d_);
            std::printf("Moved new window \"%s\" onto the mirrored screen%s\n", window_title(d_, w).c_str(),
                        (rs.full || rs.max) ? " (will re-apply fullscreen/maximise)" : "");
            if (rs.full || rs.max) restore_.push_back(rs);
            seen_.push_back(w);
        }
    }

private:
    struct Restore { Window w; uint32_t when; bool full, max; };
    Display* d_;
    XRect r_;
    std::vector<Window> legacy_, seen_;
    uint32_t last_tick_ = 0;
    std::vector<Restore> restore_;
};

struct Hotkey { KeySym sym; KeyCode code; };

// Grab Ctrl+key on the root window, regardless of NumLock/CapsLock state.
bool grab_ctrl_key(Display* dpy, KeySym sym, Hotkey& out) {
    out = {sym, XKeysymToKeycode(dpy, sym)};
    if (!out.code) return false;
    const unsigned locks[4] = {0, LockMask, Mod2Mask, LockMask | Mod2Mask};
    g_x_error = false;
    for (unsigned l : locks) XGrabKey(dpy, out.code, ControlMask | l, DefaultRootWindow(dpy), False, GrabModeAsync, GrabModeAsync);
    XSync(dpy, False);
    return !g_x_error;
}
}  // namespace

int main(int argc, char** argv) {
    std::string source_name, led_file;
    int requested = -1, fps = 60, cursor_scale = 2;
    float arc_deg = 110.0f, mon_d = 1.5f, lens_zoom = 2.2f, lens_width = 1.5f;
    bool lens_on = true;   // on at start; Ctrl+F12 or --no-lens turns it off
    std::string shot_file;           // hidden: --shot FILE.ppm renders one frame of a synthetic desktop and exits
    float shot_gaze_u = -1, shot_gaze_v = -1;
    bool use_camera = true, skip_setup = false, no_vr = false;
    float predict_ms = 20.0f;  // head-pose prediction horizon (see HeadTracker::set_prediction)
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--source") && i + 1 < argc) source_name = argv[++i];
        else if (!std::strcmp(argv[i], "--display") && i + 1 < argc) requested = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--fps") && i + 1 < argc) fps = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-lens")) lens_on = false;
        else if (!std::strcmp(argv[i], "--lens")) lens_on = true;
        else if (!std::strcmp(argv[i], "--lens-zoom") && i + 1 < argc) lens_zoom = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--lens-size") && i + 1 < argc) lens_width = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot_file = argv[++i];
        else if (!std::strcmp(argv[i], "--shot-gaze") && i + 2 < argc) { shot_gaze_u = float(std::atof(argv[i + 1])); shot_gaze_v = float(std::atof(argv[i + 2])); i += 2; }
        else if (!std::strcmp(argv[i], "--arc") && i + 1 < argc) arc_deg = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--cursor") && i + 1 < argc) cursor_scale = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--distance") && i + 1 < argc) mon_d = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--leds") && i + 1 < argc) led_file = argv[++i];
        else if (!std::strcmp(argv[i], "--no-setup")) skip_setup = true;
        else if (!std::strcmp(argv[i], "--predict") && i + 1 < argc) predict_ms = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--no-camera")) use_camera = false;
        else if (!std::strcmp(argv[i], "--no-vr")) no_vr = true;
        else {
            std::fprintf(stderr, "Usage: %s [--source OUTPUT] [--display N] [--fps N] [--arc DEG] [--distance M] [--cursor N] [--no-lens] [--lens-zoom Z] [--lens-size M] [--no-setup] [--predict MS] [--no-camera] [--no-vr] [--leds FILE]\n", argv[0]);
            return 2;
        }
    }

    if (!skip_setup && !no_vr) ensure_dk2_display();
    XRect src;
    std::string src_name;
    if (!find_output_rect(source_name, src, src_name)) {
        std::fprintf(stderr, "Cannot find the source output%s%s (see xrandr).\n", source_name.empty() ? "" : " ", source_name.c_str());
        return 1;
    }
    std::printf("Mirroring %s: %dx%d at (%d,%d)\n", src_name.c_str(), src.w, src.h, src.x, src.y);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) { std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1; }
    for (int i = 0; i < SDL_GetNumVideoDisplays(); ++i) {
        SDL_Rect r;
        SDL_GetDisplayBounds(i, &r);
        std::printf("Display %d: %dx%d at (%d,%d)\n", i, r.w, r.h, r.x, r.y);
    }
    int disp = pick_display(requested, src);
    SDL_Rect dk;
    if (no_vr) {
        // No headset: show the virtual monitor in a window on another display, so it does not sit on top of the screen it mirrors.
        disp = -1;
        if (requested >= 0 && requested < SDL_GetNumVideoDisplays()) disp = requested;
        for (int i = 0; disp < 0 && i < SDL_GetNumVideoDisplays(); ++i) {
            SDL_Rect r;
            if (SDL_GetDisplayBounds(i, &r) == 0 && !(r.x == src.x && r.y == src.y)) disp = i;
        }
        if (disp >= 0) SDL_GetDisplayBounds(disp, &dk);
        else {
            std::fprintf(stderr, "No second display: the window will cover part of the mirrored screen and show itself (use --display N to choose).\n");
            dk = {src.x + 20, src.y + 20, std::min(1280, src.w - 40), std::min(720, src.h - 40)};
        }
        std::printf("No-VR window: %dx%d at (%d,%d)\n", dk.w, dk.h, dk.x, dk.y);
    } else {
        SDL_GetDisplayBounds(disp, &dk);
        std::printf("Headset display %d: %dx%d at (%d,%d)\n", disp, dk.w, dk.h, dk.x, dk.y);
    }

    if (led_file.empty()) {
        led_file = "data/dk2_leds.txt";
        if (FILE* f = std::fopen(led_file.c_str(), "r")) std::fclose(f);
        else if (char* base = SDL_GetBasePath()) { led_file = std::string(base) + "../data/dk2_leds.txt"; SDL_free(base); }
    }
    HeadTracker tracker;
    const bool tracking_ok = !no_vr && tracker.start(led_file, use_camera);
    tracker.set_prediction(predict_ms / 1000.0f);
    KeyboardHead kb;
    if (no_vr) std::printf("No-VR mode: Ctrl+arrows turn the head, Ctrl+Shift+arrows and Ctrl+PageUp/PageDown lean, Ctrl+Backspace resets (Alt = slow).\n");
    else if (!tracking_ok) std::fprintf(stderr, "Warning: DK2 not opened; static view.\n");

    // --- Window: override-redirect (tooltip) so it never takes keyboard focus ---------------------
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_Window* win = SDL_CreateWindow("display", dk.x, dk.y, dk.w, dk.h,
                                       SDL_WINDOW_OPENGL | SDL_WINDOW_TOOLTIP | SDL_WINDOW_BORDERLESS | SDL_WINDOW_SHOWN);
    if (!win) { std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl) { std::fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetSwapInterval(1);

    // --- Keyboard/pointer control through our own X connection --------------------------------------
    Display* xd = XOpenDisplay(nullptr);
    if (!xd) { std::fprintf(stderr, "Cannot open X display\n"); return 1; }
    XSetErrorHandler(x_error_handler);
    Hotkey hk_exit, hk_centre, hk_big, hk_small, hk_lens, hk_flip[3];
    if (!grab_ctrl_key(xd, XK_x, hk_exit)) {
        std::fprintf(stderr, "Cannot grab Ctrl+X (another program holds it). Exiting so you are not locked in.\n");
        return 1;
    }
    grab_ctrl_key(xd, XK_F9, hk_centre);
    grab_ctrl_key(xd, XK_F10, hk_big);
    grab_ctrl_key(xd, XK_F11, hk_small);
    grab_ctrl_key(xd, XK_F12, hk_lens);
    grab_ctrl_key(xd, XK_F6, hk_flip[0]);   // Ctrl+F6/F7/F8: flip the tracked position X/Y/Z (saved for all programs)
    grab_ctrl_key(xd, XK_F7, hk_flip[1]);
    grab_ctrl_key(xd, XK_F8, hk_flip[2]);
    // --no-vr: the keyboard acts as the head tracker. Grabbed so the mirrored applications do not also see these keys.
    const KeySym head_syms[7] = {XK_Left, XK_Right, XK_Up, XK_Down, XK_Prior, XK_Next, XK_BackSpace};
    KeyCode head_codes[7] = {0, 0, 0, 0, 0, 0, 0};
    if (no_vr) {
        const unsigned locks[4] = {0, LockMask, Mod2Mask, LockMask | Mod2Mask};
        const unsigned extra[4] = {0, ShiftMask, Mod1Mask, ShiftMask | Mod1Mask};
        for (int i = 0; i < 7; ++i) {
            head_codes[i] = XKeysymToKeycode(xd, head_syms[i]);
            if (!head_codes[i]) continue;
            for (unsigned x : extra) for (unsigned l : locks) XGrabKey(xd, head_codes[i], ControlMask | x | l, DefaultRootWindow(xd), False, GrabModeAsync, GrabModeAsync);
        }
        XSync(xd, False);
    }

    // Confine the pointer to the mirrored screen: block all four edges of its rectangle (the picture shows the pointer, so
    // it must never be somewhere you cannot see it).
    PointerBarrier barriers[4] = {0, 0, 0, 0};
    int xf_ev = 0, xf_er = 0;
    if (XFixesQueryExtension(xd, &xf_ev, &xf_er)) {
        Window root = DefaultRootWindow(xd);
        const int x0 = src.x, y0 = src.y, x1 = src.x + src.w, y1 = src.y + src.h;
        barriers[0] = XFixesCreatePointerBarrier(xd, root, x0, y0, x0, y1, 0, 0, nullptr);  // left edge
        barriers[1] = XFixesCreatePointerBarrier(xd, root, x1, y0, x1, y1, 0, 0, nullptr);  // right edge
        barriers[2] = XFixesCreatePointerBarrier(xd, root, x0, y0, x1, y0, 0, 0, nullptr);  // top edge
        barriers[3] = XFixesCreatePointerBarrier(xd, root, x0, y1, x1, y1, 0, 0, nullptr);  // bottom edge
        // If the pointer is currently elsewhere, bring it onto the mirrored screen.
        Window rr, cr;
        int px, py, wx, wy;
        unsigned mask;
        XQueryPointer(xd, root, &rr, &cr, &px, &py, &wx, &wy, &mask);
        if (px < x0 || px >= x1 || py < y0 || py >= y1) XWarpPointer(xd, None, root, 0, 0, 0, 0, src.x + src.w / 2, src.y + src.h / 2);
    }
    keep_focus_on_screen(xd, src);
    XSync(xd, False);
    WindowHerder herder(xd, src);

    // --- GL scene -------------------------------------------------------------------------------
    StereoRig rig;
    {
        int w0, h0;
        SDL_GL_GetDrawableSize(win, &w0, &h0);
        if (!rig.init(Dk2Optics(), w0, h0, 1.0f)) { std::fprintf(stderr, "Stereo setup failed.\n"); return 1; }
        rig.set_flat(no_vr);
    }
    GLuint prog = glCreateProgram();
    {
        GLuint vs = compile(GL_VERTEX_SHADER, kVert), fs = compile(GL_FRAGMENT_SHADER, kFrag);
        if (!vs || !fs) return 1;
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
    }
    const GLint loc_mvp = glGetUniformLocation(prog, "mvp"), loc_model = glGetUniformLocation(prog, "model"),
                loc_color = glGetUniformLocation(prog, "color"), loc_mode = glGetUniformLocation(prog, "mode"), loc_curve = glGetUniformLocation(prog, "curve"), loc_lens = glGetUniformLocation(prog, "lens"),
                loc_size = glGetUniformLocation(prog, "size");
    GLuint vao, vbo;
    {
        // Unit quad subdivided into kCols columns so the shader can bend it into a cylinder section.
        std::vector<float> quad;
        for (int i = 0; i < kCols; ++i) {
            const float xa = float(i) / kCols - 0.5f, xb = float(i + 1) / kCols - 0.5f;
            const float v[12] = {xa, -0.5f, xb, -0.5f, xb, 0.5f, xa, -0.5f, xb, 0.5f, xa, 0.5f};
            quad.insert(quad.end(), v, v + 12);
        }
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(quad.size() * sizeof(float)), quad.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    }
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, src.w, src.h, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);  // desktop is minified ~2-3x: avoid shimmer
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, 0x84FE /*GL_TEXTURE_MAX_ANISOTROPY_EXT*/, 8.0f);

    ScreenGrabber grabber;
    if (shot_file.empty()) grabber.start(src, fps, cursor_scale);
    else {   // test image instead of the real desktop
        const std::vector<uint8_t> pat = make_test_desktop(src.w, src.h);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src.w, src.h, GL_BGRA, GL_UNSIGNED_BYTE, pat.data());
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    std::vector<uint8_t> frame;

    const float aspect = float(src.h) / float(src.w);
    arc_deg = std::min(std::max(arc_deg, 20.0f), 180.0f);
    bool running = true;
    uint32_t last_print = 0, t_start = SDL_GetTicks(), last_focus_check = 0;
    bool auto_centred = false, lens_active = false;
    glm::vec2 lens_uv(0.5f);
    uint32_t last_frame = SDL_GetTicks();
    HeadKeys head_keys;
    int shot_frames = 0;
    float upload_ms = 0;   // smoothed CPU time of the texture upload (diagnostics)
    std::printf("Running. Ctrl+X exits, Ctrl+F9 re-centres, Ctrl+F10/F11 widen/narrow the curved screen.\n");

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) if (e.type == SDL_QUIT) running = false;  // e.g. SIGTERM

        while (XPending(xd)) {
            XEvent ev;
            XNextEvent(xd, &ev);
            if (ev.type != KeyPress) continue;
            const unsigned code = ev.xkey.keycode;
            if (std::getenv("DISPLAY_DEBUG")) std::printf("KeyPress code %u state 0x%x\n", code, ev.xkey.state);
            if (code == hk_exit.code) running = false;
            else if (code == hk_centre.code) { tracker.recentre(); std::printf("Re-centred.\n"); }
            else if (code == hk_flip[0].code) tracker.flip_position_axis(0);
            else if (code == hk_flip[1].code) tracker.flip_position_axis(1);
            else if (code == hk_flip[2].code) tracker.flip_position_axis(2);
            else if (code == hk_lens.code) { lens_on = !lens_on; std::printf("Gaze lens %s\n", lens_on ? "on" : "off"); }
            else if (code == hk_big.code) { arc_deg = std::min(180.0f, arc_deg * 1.1f); std::printf("Screen spans %.0f degrees\n", arc_deg); }
            else if (code == hk_small.code) { arc_deg = std::max(20.0f, arc_deg / 1.1f); std::printf("Screen spans %.0f degrees\n", arc_deg); }
        }
        {   // Backup for the hotkey grab: another program may hold an active keyboard grab, so also read the physical key state.
            char keys[32];
            XQueryKeymap(xd, keys);
            auto down = [&](KeyCode c) { return c && (keys[c / 8] & (1 << (c % 8))); };
            if (down(hk_exit.code) && (down(XKeysymToKeycode(xd, XK_Control_L)) || down(XKeysymToKeycode(xd, XK_Control_R)))) running = false;
            if (no_vr) {
                head_keys.ctrl = down(XKeysymToKeycode(xd, XK_Control_L)) || down(XKeysymToKeycode(xd, XK_Control_R));
                head_keys.shift = down(XKeysymToKeycode(xd, XK_Shift_L)) || down(XKeysymToKeycode(xd, XK_Shift_R));
                head_keys.slow = down(XKeysymToKeycode(xd, XK_Alt_L)) || down(XKeysymToKeycode(xd, XK_Alt_R));
                head_keys.left = down(head_codes[0]);
                head_keys.right = down(head_codes[1]);
                head_keys.up = down(head_codes[2]);
                head_keys.down = down(head_codes[3]);
                head_keys.page_up = down(head_codes[4]);
                head_keys.page_down = down(head_codes[5]);
                head_keys.reset = head_keys.ctrl && down(head_codes[6]);
            }
        }
        herder.tick(SDL_GetTicks());
        if (SDL_GetTicks() - last_focus_check > 500) {  // keyboard input must act on the mirrored screen
            keep_focus_on_screen(xd, src);
            last_focus_check = SDL_GetTicks();
        }
        if (!auto_centred && SDL_GetTicks() - t_start > 4000) {  // settle, then take the first heading as "straight ahead"
            tracker.recentre();
            auto_centred = true;
        }

        const uint32_t frame_now = SDL_GetTicks();
        const float frame_dt = std::min(0.1f, float(frame_now - last_frame) / 1000.0f);
        last_frame = frame_now;
        glm::quat head_q(1, 0, 0, 0);
        glm::vec3 head_pos(0.0f);
        if (tracking_ok) {
            tracker.update();
            head_q = tracker.orientation();
            head_pos = tracker.position();
        } else if (no_vr) {
            kb.update(frame_dt, head_keys);
            head_q = kb.orientation();
            head_pos = kb.position();
        }
        if (!shot_file.empty()) { head_q = glm::quat(1, 0, 0, 0); head_pos = glm::vec3(0.0f); }  // screenshots: level, centred view
        const glm::mat4 view = glm::inverse(glm::translate(glm::mat4(1.0f), head_pos) * glm::mat4_cast(head_q));

        if (grabber.take(frame)) {
            const uint32_t u0 = SDL_GetTicks();
            glBindTexture(GL_TEXTURE_2D, tex);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src.w, src.h, GL_BGRA, GL_UNSIGNED_BYTE, frame.data());
            glGenerateMipmap(GL_TEXTURE_2D);
            upload_ms += (float(SDL_GetTicks() - u0) - upload_ms) * 0.05f;
        }

        int w, h;
        SDL_GL_GetDrawableSize(win, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);

        // Curved screen: a cylinder section of radius mon_d around the viewer, spanning arc_deg horizontally.
        // arc_deg == 0 would be a flat panel; keep a tiny minimum so the curve branch is used.
        const float arc = glm::radians(arc_deg);
        const float mon_w = mon_d * arc;           // arc length
        const float mon_h = mon_w * aspect;
        const float floor_y = -mon_h * 0.5f - 0.3f;
        // Gaze lens: where the head points on the screen, smoothed so it doesn't jitter.
        glm::vec4 lens_u(0.0f);
        {
            glm::vec2 uv;
            const glm::vec3 gaze_dir = head_q * glm::vec3(0, 0, -1);
            const bool hit = shot_gaze_u >= 0 ? (uv = glm::vec2(shot_gaze_u, shot_gaze_v), true) : gaze_on_screen(head_pos, gaze_dir, mon_d, arc, mon_h, uv);
            if (hit && lens_on) {
                if (!lens_active) lens_uv = uv;
                lens_uv += (uv - lens_uv) * std::min(1.0f, 10.0f * frame_dt);
                lens_u = glm::vec4(lens_uv.x, lens_uv.y, lens_width * 0.5f, lens_zoom);
            }
            lens_active = hit && lens_on;
        }
        for (int eye = 0; eye < rig.eye_count(); ++eye) {
            rig.begin_eye(eye, glm::vec4(0.01f, 0.01f, 0.012f, 1.0f));
            glDisable(GL_CULL_FACE);
            const glm::mat4 vp = rig.projection(eye) * rig.eye_offset(eye) * view;
            glUseProgram(prog);
            glBindVertexArray(vao);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, tex);
            glUniform1i(glGetUniformLocation(prog, "tex"), 0);
            glUniform4fv(loc_lens, 1, glm::value_ptr(lens_u));
            glUniform2f(loc_size, mon_w, mon_h);
            auto draw = [&](const glm::mat4& mvp, const glm::mat4& model, int mode, glm::vec4 color, glm::vec3 curve) {
                glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
                glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(model));
                glUniform4fv(loc_color, 1, glm::value_ptr(color));
                glUniform3fv(loc_curve, 1, glm::value_ptr(curve));
                glUniform1i(loc_mode, mode);
                glDrawArrays(GL_TRIANGLES, 0, kCols * 6);
            };
            glm::mat4 fm = glm::translate(glm::mat4(1.0f), glm::vec3(0, floor_y, -2.0f));
            fm = glm::rotate(fm, glm::radians(-90.0f), glm::vec3(1, 0, 0));
            fm = glm::scale(fm, glm::vec3(40.0f, 40.0f, 1.0f));
            draw(vp * fm, fm, 2, glm::vec4(0), glm::vec3(0));
            // Bezel (slightly larger, slightly behind) and the screen itself.
            draw(vp, glm::mat4(1.0f), 0, glm::vec4(0.02f, 0.02f, 0.025f, 1),
                 glm::vec3(arc + 0.03f / mon_d, mon_d + 0.01f, mon_h + 0.06f));
            draw(vp, glm::mat4(1.0f), 1, glm::vec4(0), glm::vec3(arc, mon_d, mon_h));
        }
        rig.present(w, h);
        if (!shot_file.empty() && ++shot_frames == 8) {
            std::vector<unsigned char> px(size_t(w) * h * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadBuffer(GL_BACK);
            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            if (FILE* f = std::fopen(shot_file.c_str(), "wb")) {
                std::fprintf(f, "P6\n%d %d\n255\n", w, h);
                for (int y = h - 1; y >= 0; --y) std::fwrite(&px[size_t(y) * w * 3], 1, size_t(w) * 3, f);
                std::fclose(f);
            }
            running = false;
        }
        SDL_GL_SwapWindow(win);

        const uint32_t now = SDL_GetTicks();
        if (now - last_print > 5000) {
            std::printf("capture %.0f fps (screen copy %.1f ms, frame copy %.1f ms, upload %.1f ms)%s\n", grabber.measured_fps(), grabber.grab_ms(), grabber.copy_ms(), upload_ms,
                        tracking_ok && tracker.camera_active() ? (tracker.camera_tracking() ? "  PTC tracking" : "  PTC lost") : "");
            last_print = now;
        }
    }

    grabber.stop();
    for (PointerBarrier b : barriers) if (b) XFixesDestroyPointerBarrier(xd, b);
    XSync(xd, False);
    XCloseDisplay(xd);
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
