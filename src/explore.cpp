// Explore: walk through a text map rendered in 3D on the DK2.
//
// Usage: explore [--leap-seat RIGHT UP FWD] [--leap-offset RIGHT UP FWD] [--no-leap] [--sim-hand] [--height M] [--map FILE] [--glossary FILE] [--display N] [--no-setup] [--height M] [--predict MS] [--no-camera] [--no-vr] [--leds FILE]
//        --leap-seat R U F     where the Leap Motion controller is relative to your head when you sit at the start (metres; default 0 -0.55 0.45: on the desk in front of your belly)
//        --leap-offset R U F   the same, but measured from the PTC instead (overrides --leap-seat; the saved ~/.config/dk2vr/leap_offset overrides the seat guess too)
//        Alt+Left/Right/Up/Down and Alt+PageUp/PageDown nudge the controller's placed position (saved), Alt+Backspace resets it; --no-leap hides it
//        --reach M   crosshair / interaction distance in metres (default 5; 0 hides the crosshair and disables interaction)
//        explore --check [--map FILE] [--glossary FILE] [--reach M]     parse the map, print a report and exit (no headset needed)
//        explore --print-glossary                           print the built-in symbol glossary and exit
// Keys: W/S walk forward/back (in the direction you look), A/D strafe, Shift run, Left/Right turn the body,
//       F toggle fly mode (Space up, Left Ctrl down, W/S fly along your gaze), C re-centre, F5 reload the map,
//       E or left mouse button: interact with the object under the crosshair (a small inverting dot; it grows when something can be used),
//       I/K hand forward/back, J/L hand left/right, U/O hand up/down, H show/hide the (simulated) red hand over the Leap controller,
//       M mute, - / = sound volume, F1/F2/F3 flip tracked position X/Y/Z, Esc quit.
// Sound: water, trees (leaves and birds) and gems are 3D sound sources (see `sound=` in the glossary); footsteps depend on the ground (`step=`).
#define GL_GLEXT_PROTOTYPES 1
#include <SDL.h>
#include <SDL_opengl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dk2_display.hpp"
#include "head_tracker.hpp"
#include "leap.hpp"
#include "novr.hpp"
#include "mapdata.hpp"
#include "spatial_audio.hpp"
#include "stereo.hpp"

namespace {
const char* kVert = R"(#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 normal;
layout(location=2) in vec3 color;
layout(location=3) in vec2 uv;
uniform mat4 mvp;
uniform mat4 model;   // identity for the static map; a turn about the vertical axis for animated objects
out vec3 v_pos;
out vec3 v_normal;
out vec3 v_color;
out vec2 v_uv;
void main() {
    v_uv = uv;
    vec4 wp = model * vec4(pos, 1.0);
    v_pos = wp.xyz;
    v_normal = mat3(model) * normal;
    v_color = color;
    gl_Position = mvp * wp;
}
)";
const char* kFrag = R"(#version 330 core
in vec3 v_pos;
in vec3 v_normal;
in vec3 v_color;
in vec2 v_uv;
uniform sampler2D tex;
uniform int use_tex;
uniform vec3 eye;
uniform vec3 sky;
uniform float fog;
out vec4 out_color;
void main() {
    vec3 n = normalize(v_normal);
    float sun = max(dot(n, normalize(vec3(0.45, 0.85, 0.3))), 0.0);
    float amb = mix(0.30, 0.48, n.y * 0.5 + 0.5);
    vec3 base = v_color;
    if (use_tex != 0) {
        vec4 t = texture(tex, v_uv);
        if (t.a < 0.5) discard;  // cut-out leaves, fences, ...
        base *= t.rgb;
    }
    vec3 col = base * (amb + 0.62 * sun);
    float d = length(v_pos - eye);
    float f = 1.0 - exp(-fog * d);
    out_color = vec4(mix(col, sky, clamp(f, 0.0, 1.0)), 1.0);
}
)";

// Crosshair: a small world-space disc on the gaze ray, drawn at the depth of whatever it points at (so both eyes converge on it).
// Blended as out = a*(1-dst) + (1-a)*dst, i.e. a semi-transparent colour inversion of what is behind it.
const char* kDotVert = R"(#version 330 core
uniform mat4 mvp;
uniform vec3 centre, right, up;
uniform float radius;
out vec2 v_p;
void main() {
    vec2 p = vec2((gl_VertexID & 1) != 0 ? 1.0 : -1.0, (gl_VertexID & 2) != 0 ? 1.0 : -1.0);
    v_p = p;
    gl_Position = mvp * vec4(centre + (right * p.x + up * p.y) * radius, 1.0);
}
)";
const char* kDotFrag = R"(#version 330 core
uniform float alpha;
in vec2 v_p;
out vec4 out_color;
void main() {
    float a = alpha * (1.0 - smoothstep(0.6, 1.0, length(v_p)));
    out_color = vec4(a, a, a, a);
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

std::string find_file(const std::string& rel) {  // working directory first, then next to the executable's parent
    if (FILE* f = std::fopen(rel.c_str(), "r")) { std::fclose(f); return rel; }
    if (char* base = SDL_GetBasePath()) {
        std::string p = std::string(base) + "../" + rel;
        SDL_free(base);
        if (FILE* f = std::fopen(p.c_str(), "r")) { std::fclose(f); return p; }
    }
    return rel;
}


// Sound sources of a map (every cell whose glossary entry has sound=...). Trees get birds now and then.
std::vector<SoundSource> collect_sources(const GameMap& map) {
    std::vector<SoundSource> v;
    for (int z = 0; z < map.height(); ++z)
        for (int x = 0; x < map.width(); ++x) {
            const Glyph& g = map.glyph(x, z);
            int kind = SpatialAudio::sound_kind(g.sound);
            if (kind < 0) continue;
            const unsigned h = unsigned(x) * 73856093u ^ unsigned(z) * 19349663u;
            if (g.kind == "tree" && kind == kSoundLeaves && h % 6 == 0) kind = kSoundBird;
            const float cx = float(x) + 0.5f, cz = float(z) + 0.5f, base = map.ground_height(cx, cz);
            float y = base + 1.0f;
            if (g.kind == "water") y = base - 0.1f;
            else if (g.kind == "tree") y = base + (kind == kSoundBird ? 2.6f : 2.2f);
            else if (g.kind == "item") y = base + 0.85f;
            SoundSource s;
            s.id = z * map.width() + x;
            s.kind = kind;
            s.pos = glm::vec3(cx, y, cz);
            v.push_back(s);
        }
    return v;
}

// 0 = clear line from the ear to the source, 1 = behind walls (only walls taller than the ray at that point count).
float occlusion_between(const GameMap& map, const glm::vec3& ear, const glm::vec3& src) {
    const glm::vec2 a(ear.x, ear.z), b(src.x, src.z);
    const float len = glm::length(b - a);
    const int n = std::max(1, int(len / 0.25f));
    const int ex = int(std::floor(a.x)), ez = int(std::floor(a.y)), sx = int(std::floor(b.x)), sz = int(std::floor(b.y));
    int hits = 0;
    for (int i = 1; i < n; ++i) {
        const float t = float(i) / float(n);
        const glm::vec2 p = a + (b - a) * t;
        const int cx = int(std::floor(p.x)), cz = int(std::floor(p.y));
        if ((cx == ex && cz == ez) || (cx == sx && cz == sz)) continue;
        const Glyph& g = map.glyph(cx, cz);
        if (g.kind != "wall") continue;
        const float ray_y = ear.y + (src.y - ear.y) * t;
        if (ray_y < map.ground_height(p.x, p.y) + g.height) ++hits;
    }
    return std::min(1.0f, float(hits) * 0.5f);
}

void print_report(const GameMap& map) {
    std::printf("Map: %d x %d cells (%d x %d m), start at (%.1f, %.1f)\n", map.width(), map.height(), map.width(), map.height(),
                map.start().x, map.start().y);
    std::printf("Symbols used:\n");
    for (const auto& kv : map.used_glyphs()) {
        const Glyph& g = kv.second;
        int n = 0;
        for (int z = 0; z < map.height(); ++z) for (int x = 0; x < map.width(); ++x) if (map.symbol(x, z) == kv.first) ++n;
        std::printf("  '%c'  %-6s %-18s %5d cells  %s\n", kv.first, g.kind.c_str(), g.label.c_str(), n, g.walk ? "walkable" : "solid");
        if (!g.action.empty()) std::printf("        action: %s, %.1f s%s\n", g.action.c_str(), g.duration, g.interact ? "" : " (interact=no, so it cannot be triggered)");
        if (const Model* m = map.model_of(g))
            std::printf("        model %s: %zu triangles, %zu textures, size %.2f x %.2f x %.2f m (x scale %.2f)\n", g.model_file.c_str(), m->triangles(),
                        m->images.size(), m->hi.x - m->lo.x, m->hi.y - m->lo.y, m->hi.z - m->lo.z, g.scale);
    }
    for (const std::string& w : map.warnings()) std::printf("Warning: %s\n", w.c_str());
    {
        int counts[kSoundKinds] = {0};
        for (const SoundSource& src : collect_sources(map)) ++counts[src.kind];
        std::printf("Sound sources: water %d, leaves %d, birds %d, gem hum %d, fire %d\n", counts[kSoundWater], counts[kSoundLeaves], counts[kSoundBird], counts[kSoundHum], counts[kSoundFire]);
    }
    if (map.has_heights()) std::printf("Terrain: elevation %.2f .. %.2f m\n", map.min_height(), map.max_height());
    std::printf("Mesh: %zu vertices\n", map.build_mesh().size());
}
}  // namespace

int main(int argc, char** argv) {
    std::string map_file = "data/maps/demo.txt", glossary_file, led_file;
    int requested = -1;
    bool use_camera = true, skip_setup = false, check = false, map_given = false, no_vr = false;
    bool use_leap = true, sim_only = false;   // sim_only: ignore the Ultraleap service, use the keyboard-driven hand
    glm::vec3 leap_off(0.0f);                      // Leap relative to the PTC in the tracking frame (+X right, +Y up, +Z back)
    glm::vec3 leap_seat(0.0f, -0.55f, -0.45f);     // where the Leap is relative to the seated head at the start (same frame); used to place it on the first PTC fix
    bool leap_off_known = false;
    float reach = 5.0f;   // crosshair / interaction distance in metres (0 = off)
    float predict_ms = 20.0f, user_height = 1.9f;  // your height in metres sets eye height and step length; predict_ms = head-pose prediction (see HeadTracker::set_prediction)
    std::string shot_file;  // hidden: --shot FILE.ppm writes one frame (both eyes, after lens distortion) and exits
    bool test_interact = false;   // hidden: --test-interact pushes E key presses at 2 s, 3 s and 7.5 s (exercises the interaction path without a keyboard)
    float shot_spin = -1.0f;   // hidden: --shot-spin S starts every animated object S seconds into its action (for --shot)
    float shot_yaw = 0.0f, shot_x = -1.0f, shot_z = -1.0f, shot_y = -1.0f;  // hidden: place the camera for --shot (shot_y = fixed eye height)
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--map") && i + 1 < argc) { map_file = argv[++i]; map_given = true; }
        else if (!std::strcmp(argv[i], "--glossary") && i + 1 < argc) glossary_file = argv[++i];
        else if (!std::strcmp(argv[i], "--display") && i + 1 < argc) requested = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--leds") && i + 1 < argc) led_file = argv[++i];
        else if (!std::strcmp(argv[i], "--height") && i + 1 < argc) user_height = std::min(2.3f, std::max(1.2f, float(std::atof(argv[++i]))));
        else if (!std::strcmp(argv[i], "--reach") && i + 1 < argc) reach = std::max(0.0f, float(std::atof(argv[++i])));
        else if (!std::strcmp(argv[i], "--predict") && i + 1 < argc) predict_ms = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--leap-offset") && i + 3 < argc) { leap_off = glm::vec3(float(std::atof(argv[i + 1])), float(std::atof(argv[i + 2])), -float(std::atof(argv[i + 3]))); leap_off_known = true; i += 3; }
        else if (!std::strcmp(argv[i], "--leap-seat") && i + 3 < argc) { leap_seat = glm::vec3(float(std::atof(argv[i + 1])), float(std::atof(argv[i + 2])), -float(std::atof(argv[i + 3]))); i += 3; }
        else if (!std::strcmp(argv[i], "--no-leap")) use_leap = false;
        else if (!std::strcmp(argv[i], "--sim-hand")) sim_only = true;
        else if (!std::strcmp(argv[i], "--no-camera")) use_camera = false;
        else if (!std::strcmp(argv[i], "--no-vr")) no_vr = true;
        else if (!std::strcmp(argv[i], "--no-setup")) skip_setup = true;
        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot_file = argv[++i];
        else if (!std::strcmp(argv[i], "--shot-pos") && i + 2 < argc) { shot_x = float(std::atof(argv[i + 1])); shot_z = float(std::atof(argv[i + 2])); i += 2; }
        else if (!std::strcmp(argv[i], "--shot-eye") && i + 1 < argc) shot_y = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--test-interact")) test_interact = true;
        else if (!std::strcmp(argv[i], "--shot-spin") && i + 1 < argc) shot_spin = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--shot-yaw") && i + 1 < argc) shot_yaw = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--check")) check = true;
        else if (!std::strcmp(argv[i], "--print-glossary")) { std::fputs(kBuiltinGlossary, stdout); return 0; }
        else {
            std::fprintf(stderr, "Usage: %s [--map FILE] [--glossary FILE] [--display N] [--no-setup] [--height M] [--predict MS] [--no-camera] [--no-vr] [--reach M] [--leds FILE]\n"
                                 "       %s --check [--map FILE] [--glossary FILE] [--reach M]\n       %s --print-glossary\n", argv[0], argv[0], argv[0]);
            return 2;
        }
    }
    if (!map_given) map_file = find_file(map_file);

    GameMap map;
    {
        std::string err;
        if (!map.load(map_file, glossary_file, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    }
    if (check) {
        print_report(map);
        // Collision self-test: walk 12 m in each compass direction from the start; the position must never be inside a solid cell.
        const glm::vec2 dirs[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
        const char* names[4] = {"north", "east", "south", "west"};
        for (int d = 0; d < 4; ++d) {
            glm::vec2 p = map.start();
            bool bad = false;
            for (int i = 0; i < 240; ++i) { map.move(p, dirs[d] * 0.05f); if (map.blocked(p)) bad = true; }
            std::printf("Walk %-5s 12 m from start: ends at (%.2f, %.2f)%s\n", names[d], p.x, p.y, bad ? "  ** INSIDE A SOLID CELL **" : "");
        }
        // Crosshair self-test: what the crosshair would pick looking along each compass direction from the start (reach %.1f m).
        std::printf("Crosshair (reach %.1f m):\n", reach > 0 ? reach : 5.0f);
        for (int d = 0; d < 4; ++d) {
            std::string line;
            for (float eye_h : {1.6f, 0.9f}) {
                const glm::vec2 s0 = map.start();
                const glm::vec3 o(s0.x, map.ground_height(s0.x, s0.y) + eye_h, s0.y);
                const PickResult r = map.pick(o, glm::vec3(dirs[d].x, 0, dirs[d].y), reach > 0 ? reach : 5.0f);
                char buf[160];
                if (r.hit) std::snprintf(buf, sizeof buf, "%s%.1f m: '%c' %s at %.2f m", line.empty() ? "" : "   ", eye_h, map.symbol(r.x, r.z), map.glyph(r.x, r.z).label.c_str(), r.dist);
                else std::snprintf(buf, sizeof buf, "%s%.1f m: nothing (stopped at %.2f m)", line.empty() ? "" : "   ", eye_h, r.dist);
                line += buf;
            }
            std::printf("  aim %-5s  %s\n", names[d], line.c_str());
        }
        return 0;
    }
    print_report(map);

    if (!skip_setup && !no_vr) ensure_dk2_display();
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) { std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1; }

    if (led_file.empty()) led_file = find_file("data/dk2_leds.txt");
    HeadTracker tracker;
    const bool tracking_ok = !no_vr && tracker.start(led_file, use_camera);
    tracker.set_prediction(predict_ms / 1000.0f);
    KeyboardHead kb;
    if (no_vr) std::printf("No-VR mode: W/A/S/D walk, arrows turn the body, Ctrl+arrows turn the head, Ctrl+Shift+arrows and Ctrl+PageUp/PageDown lean, Ctrl+Backspace resets (Alt = slow).\n");
    else if (!tracking_ok) std::fprintf(stderr, "Warning: DK2 not opened; static view.\n");

    const int disp = pick_display(requested);
    std::printf("Using display %d\n", disp);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_Window* win = no_vr
        ? SDL_CreateWindow("explore (no headset)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE)
        : SDL_CreateWindow("explore", SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp), SDL_WINDOWPOS_UNDEFINED_DISPLAY(disp), 1920, 1080,
                           SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN_DESKTOP);
    if (!win) { std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    if (!gl) { std::fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetSwapInterval(1);

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
    const GLint loc_mvp = glGetUniformLocation(prog, "mvp"), loc_eye = glGetUniformLocation(prog, "eye"),
                loc_sky = glGetUniformLocation(prog, "sky"), loc_fog = glGetUniformLocation(prog, "fog"),
                loc_tex = glGetUniformLocation(prog, "tex"), loc_use_tex = glGetUniformLocation(prog, "use_tex"), loc_model = glGetUniformLocation(prog, "model");
    GLuint dot_prog = glCreateProgram();
    {
        GLuint vs = compile(GL_VERTEX_SHADER, kDotVert), fs = compile(GL_FRAGMENT_SHADER, kDotFrag);
        if (!vs || !fs) return 1;
        glAttachShader(dot_prog, vs);
        glAttachShader(dot_prog, fs);
        glLinkProgram(dot_prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
    }
    const GLint dot_mvp = glGetUniformLocation(dot_prog, "mvp"), dot_centre = glGetUniformLocation(dot_prog, "centre"),
                dot_right = glGetUniformLocation(dot_prog, "right"), dot_up = glGetUniformLocation(dot_prog, "up"),
                dot_radius = glGetUniformLocation(dot_prog, "radius"), dot_alpha = glGetUniformLocation(dot_prog, "alpha");
    PickResult target;               // what the crosshair points at (updated every frame)
    uint32_t pulse_until = 0;        // feedback flash after an interaction
    GLuint vao = 0, vbo = 0;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    GLsizei vertex_count = 0;
    GLint leap_first[2] = {0, 0};   // Leap model (dim, lit) in the vertex buffer
    GLsizei leap_count = 0;
    GLuint hand_vao = 0, hand_vbo = 0;   // the hand is rebuilt every frame
    GLsizei hand_vertices = 0;
    struct DrawBatch { GLint first; GLsizei count; GLuint texture; };   // a range of the vertex buffer with one texture (0 = none)
    struct DynObject {                                                  // an object of a cell with an `action`: drawn with its own matrix
        int x, z;
        float duration;
        std::vector<DrawBatch> parts;
        bool spinning = false;
        uint32_t start = 0;
    };
    std::vector<DrawBatch> batches;   // placed glTF models (static)
    std::vector<DynObject> dyn;
    std::vector<GLuint> textures;
    auto upload_mesh = [&]() {
        std::vector<MapVertex> mesh = map.build_mesh();
        vertex_count = GLsizei(mesh.size());
        if (!textures.empty()) glDeleteTextures(GLsizei(textures.size()), textures.data());
        textures.clear();
        batches.clear();
        dyn.clear();
        auto add_models = [&](const std::vector<ModelBatch>& mbs, std::vector<DrawBatch>& out) {
            for (const ModelBatch& mb : mbs) {
                DrawBatch b{GLint(mesh.size()), GLsizei(mb.verts.size()), 0};
                if (mb.image) {
                    glGenTextures(1, &b.texture);
                    textures.push_back(b.texture);
                    glBindTexture(GL_TEXTURE_2D, b.texture);
                    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mb.image->w, mb.image->h, 0, GL_RGBA, GL_UNSIGNED_BYTE, mb.image->rgba.data());
                    glGenerateMipmap(GL_TEXTURE_2D);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                }
                mesh.insert(mesh.end(), mb.verts.begin(), mb.verts.end());
                out.push_back(b);
            }
        };
        add_models(map.build_models(), batches);
        for (const glm::ivec2& c : map.action_cells()) {   // animated objects: procedural geometry and models of that cell
            DynObject d{c.x, c.y, map.glyph(c.x, c.y).duration, {}};
            const std::vector<MapVertex> own = map.build_mesh(c.x, c.y);
            if (!own.empty()) {
                d.parts.push_back({GLint(mesh.size()), GLsizei(own.size()), 0});
                mesh.insert(mesh.end(), own.begin(), own.end());
            }
            add_models(map.build_models(c.x, c.y), d.parts);
            dyn.push_back(std::move(d));
        }
        for (int lit = 0; lit < 2; ++lit) {
            const std::vector<MapVertex> lm = build_leap_mesh(lit != 0);
            leap_first[lit] = GLint(mesh.size());
            leap_count = GLsizei(lm.size());
            mesh.insert(mesh.end(), lm.begin(), lm.end());
        }
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(mesh.size() * sizeof(MapVertex)), mesh.data(), GL_STATIC_DRAW);
        for (int i = 0; i < 3; ++i) {
            glVertexAttribPointer(GLuint(i), 3, GL_FLOAT, GL_FALSE, sizeof(MapVertex), reinterpret_cast<void*>(size_t(i) * 3 * sizeof(float)));
            glEnableVertexAttribArray(GLuint(i));
        }
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(MapVertex), reinterpret_cast<void*>(9 * sizeof(float)));
        glEnableVertexAttribArray(3);
    };
    upload_mesh();
    glGenVertexArrays(1, &hand_vao);
    glGenBuffers(1, &hand_vbo);
    glBindVertexArray(hand_vao);
    glBindBuffer(GL_ARRAY_BUFFER, hand_vbo);
    for (int i = 0; i < 3; ++i) {
        glVertexAttribPointer(GLuint(i), 3, GL_FLOAT, GL_FALSE, sizeof(MapVertex), reinterpret_cast<void*>(size_t(i) * 3 * sizeof(float)));
        glEnableVertexAttribArray(GLuint(i));
    }
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(MapVertex), reinterpret_cast<void*>(9 * sizeof(float)));
    glEnableVertexAttribArray(3);
    glBindVertexArray(vao);
    if (shot_spin >= 0) for (DynObject& d : dyn) { d.spinning = true; d.start = SDL_GetTicks() - uint32_t(shot_spin * 1000.0f); }
    auto draw_batch = [&](const DrawBatch& b) {
        glUniform1i(loc_use_tex, b.texture ? 1 : 0);
        if (b.texture) { glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, b.texture); }
        glDrawArrays(GL_TRIANGLES, b.first, b.count);
    };

    SpatialAudio audio;
    audio.open();
    std::vector<SoundSource> all_sources = collect_sources(map);
    float step_distance = 0.0f, stride_jitter = 1.0f;
    uint32_t stride_seed = 2463534242u;
    int step_foot = 1;

    const float kEye = user_height - 0.12f;   // eye height (about 12 cm below the top of the head)
    glm::vec2 pos = shot_x >= 0 ? glm::vec2(shot_x, shot_z) : map.start();
    float body_yaw = shot_yaw * 3.14159265f / 180.0f, fly_y = map.ground_height(pos.x, pos.y) + kEye, eye_ground = map.ground_height(pos.x, pos.y);  // radians, CCW seen from above (0 = looking north / -Z)
    bool fly = false;
    int last_cell_x = -1, last_cell_z = -1;

    LeapInfo leap;
    SimHand sim_hand;
    LeapTracker leap_tracker;
    const bool leap_service = use_leap && !sim_only && leap_tracker.start();
    uint32_t leap_checked = 0;
    if (!leap_off_known && load_leap_offset(leap_off)) { leap_off_known = true; std::printf("Leap placement loaded: %.2f right, %.2f up, %.2f forward of the PTC\n", leap_off.x, leap_off.y, -leap_off.z); }
    glm::vec3 leap_rel = leap_seat;   // Leap position relative to the head origin (the seat guess until the PTC has a fix)
    uint32_t leap_nudged = 0; bool leap_dirty = false;

    bool running = true;
    int shot_frames = 0;
    uint32_t last_tick = SDL_GetTicks(), last_print = 0;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            const bool use_click = e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT;
            if (reach > 0 && (use_click || (e.type == SDL_KEYDOWN && !e.key.repeat && e.key.keysym.sym == SDLK_e))) {
                if (target.hit) {   // interact with the object under the crosshair
                    const Glyph& g = map.glyph(target.x, target.z);
                    std::printf("Interact: %s (%s '%c') at cell (%d,%d), %.1f m away\n", g.label.c_str(), g.kind.c_str(), map.symbol(target.x, target.z), target.x, target.z, target.dist);
                    pulse_until = SDL_GetTicks() + 300;
                    if (g.action == "spin") {
                        for (DynObject& d : dyn)
                            if (d.x == target.x && d.z == target.z) {
                                if (d.spinning) std::printf("  (already turning)\n");
                                else { d.spinning = true; d.start = SDL_GetTicks(); std::printf("  turns for %.1f s\n", d.duration); }
                            }
                    }
                } else {
                    std::printf("Nothing to interact with within %.1f m\n", reach);
                }
            }
            if (e.type != SDL_KEYDOWN || e.key.repeat) continue;
            const SDL_Keycode k = e.key.keysym.sym;
            if (k == SDLK_LEFTBRACKET || k == SDLK_RIGHTBRACKET) {  // tune the prediction by feel: too little = lag, too much = overshoot
                tracker.set_prediction(tracker.prediction() + (k == SDLK_RIGHTBRACKET ? 0.002f : -0.002f));
                std::printf("Prediction %.0f ms\n", tracker.prediction() * 1000.0f);
            }
            if (k == SDLK_ESCAPE) running = false;
            else if (k == SDLK_h) { sim_hand.toggle(); std::printf("Hand %s\n", sim_hand.state().present ? "shown" : "hidden"); }
            else if (k == SDLK_m) { audio.toggle_mute(); std::printf("Sound %s\n", audio.muted() ? "muted" : "on"); }
            else if (k == SDLK_MINUS || k == SDLK_EQUALS) { audio.set_volume(audio.volume() + (k == SDLK_EQUALS ? 0.1f : -0.1f)); std::printf("Volume %.0f%%\n", audio.volume() * 100.0f); }
            else if (k == SDLK_c && !(SDL_GetModState() & KMOD_CTRL)) { tracker.recentre(); std::printf("Re-centred.\n"); }
            else if (k == SDLK_f) {
                fly = !fly;
                if (fly) fly_y = eye_ground + kEye;
                std::printf("%s mode\n", fly ? "Fly" : "Walk");
            } else if (k == SDLK_F5) {
                std::string err;
                GameMap fresh;
                if (fresh.load(map_file, glossary_file, err)) { map = fresh; upload_mesh(); pos = map.start(); eye_ground = map.ground_height(pos.x, pos.y); all_sources = collect_sources(map); std::printf("Map reloaded.\n"); print_report(map); }
                else std::fprintf(stderr, "Reload failed: %s\n", err.c_str());
            } else if (k == SDLK_F1 || k == SDLK_F2 || k == SDLK_F3) {
                const int ax = k == SDLK_F1 ? 0 : (k == SDLK_F2 ? 1 : 2);
                tracker.flip_position_axis(ax);
            }
        }
        const uint32_t tick = SDL_GetTicks();
        if (test_interact) {
            static const uint32_t at[3] = {2000, 3000, 7500};
            static int fired = 0;
            if (fired < 3 && tick >= at[fired]) {
                SDL_Event ev;
                std::memset(&ev, 0, sizeof ev);
                ev.type = SDL_KEYDOWN;
                ev.key.keysym.sym = SDLK_e;
                SDL_PushEvent(&ev);
                ++fired;
            }
        }
        const float dt = std::min(0.1f, (tick - last_tick) / 1000.0f);
        last_tick = tick;

        glm::quat head_q(1, 0, 0, 0);
        glm::vec3 lean(0.0f);
        if (tracking_ok) {
            tracker.update();
            head_q = tracker.orientation();
            lean = tracker.position();
        } else if (no_vr) {
            kb.update(dt, head_keys_from_sdl());
            head_q = kb.orientation();
            lean = kb.position();
        }

        // Keyboard: turn the body, then move relative to where the head is looking.
        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        const bool ctrl_held = keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL];   // Ctrl+arrows move the head instead (--no-vr)
        const bool alt_held = keys[SDL_SCANCODE_LALT] || keys[SDL_SCANCODE_RALT];   // Alt+arrows nudge the Leap placement
        if (keys[SDL_SCANCODE_LEFT] && !ctrl_held && !alt_held) body_yaw += 1.6f * dt;
        if (keys[SDL_SCANCODE_RIGHT] && !ctrl_held && !alt_held) body_yaw -= 1.6f * dt;
        const glm::quat body = glm::angleAxis(body_yaw, glm::vec3(0, 1, 0));
        const glm::vec3 gaze = body * head_q * glm::vec3(0, 0, -1);
        const glm::vec3 right3 = body * head_q * glm::vec3(1, 0, 0);
        const bool run = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT];
        const float speed = (fly ? (run ? 12.0f : 4.0f) : (run ? 4.5f : 2.0f)) * dt;
        glm::vec3 fwd3 = fly ? gaze : glm::vec3(gaze.x, 0, gaze.z);
        glm::vec3 side3 = fly ? right3 : glm::vec3(right3.x, 0, right3.z);
        if (glm::length(fwd3) > 1e-4f) fwd3 = glm::normalize(fwd3);
        if (glm::length(side3) > 1e-4f) side3 = glm::normalize(side3);
        glm::vec3 move(0.0f);
        if (keys[SDL_SCANCODE_W]) move += fwd3;
        if (keys[SDL_SCANCODE_S]) move -= fwd3;
        if (keys[SDL_SCANCODE_D]) move += side3;
        if (keys[SDL_SCANCODE_A]) move -= side3;
        if (fly) {
            if (keys[SDL_SCANCODE_SPACE]) move.y += 1.0f;
            if (keys[SDL_SCANCODE_LCTRL]) move.y -= 1.0f;
        }
        if (glm::length(move) > 1e-4f) {
            move = glm::normalize(move) * speed;
            if (fly) {
                pos += glm::vec2(move.x, move.z);  // fly: no collision
                fly_y = fly_y + move.y;
            } else {
                const glm::vec2 before = pos;
                map.move(pos, glm::vec2(move.x, move.z));
                step_distance += glm::length(pos - before);
            }
        }

        // View: body position + body heading, then the tracked lean and head rotation.
        // Eye height: walking follows the ground (smoothed so slopes and steps don't jolt the view); flying stays above it.
        const float ground = map.ground_height(pos.x, pos.y);
        eye_ground += (ground - eye_ground) * std::min(1.0f, 12.0f * dt);
        fly_y = std::min(80.0f + map.max_height(), std::max(ground + 0.3f, fly_y));
        const float eye_y = shot_y >= 0 ? shot_y : (fly ? fly_y : eye_ground + kEye);
        glm::vec3 lean_world = body * lean;
        if (!fly && map.blocked(pos + glm::vec2(lean_world.x, lean_world.z))) lean_world = glm::vec3(0, lean_world.y, 0);  // don't lean through walls
        const glm::vec3 eye_pos = glm::vec3(pos.x, eye_y, pos.y) + lean_world;
        if (reach > 0) target = map.pick(eye_pos, gaze, reach);
        if (use_leap && (leap_checked == 0 || tick - leap_checked > 1500)) {   // hot-plug detection
            leap_checked = tick ? tick : 1;
            const LeapInfo now = detect_leap();
            if (now.present != leap.present)
                { if (now.present) std::printf("Leap Motion controller found: %s (USB %s%s%s)\n", now.name.c_str(), now.usb_id.c_str(), now.video.empty() ? "" : ", ", now.video.c_str());
                  else std::printf("Leap Motion controller removed\n"); }
            leap = now;
        }
        if (use_leap && !(SDL_GetModState() & KMOD_CTRL)) {
            const Uint8* hk = SDL_GetKeyboardState(nullptr);
            sim_hand.update(dt, glm::vec3(float(hk[SDL_SCANCODE_L]) - float(hk[SDL_SCANCODE_J]), float(hk[SDL_SCANCODE_U]) - float(hk[SDL_SCANCODE_O]),
                                          float(hk[SDL_SCANCODE_K]) - float(hk[SDL_SCANCODE_I])));
        }
        if (leap_service) {
            leap_tracker.poll();
            static int last_hands = -1; static bool last_dev = false, announced = false;
            const int n = int(leap_tracker.hands().size());
            if (!announced || leap_tracker.device() != last_dev) { announced = true; last_dev = leap_tracker.device(); std::printf("Ultraleap service: %s, controller %s\n", leap_tracker.connected() ? "connected" : "not connected", last_dev ? "ready" : "not reported"); }
            if (n != last_hands) { last_hands = n; std::printf("Leap: %d hand%s tracked\n", n, n == 1 ? "" : "s"); }
        }
        glm::mat4 leap_model(1.0f);
        if (use_leap) {
            if (no_vr) {   // no headset: fixed at the bottom of the view, tilted towards the viewer, enlarged a little to stay readable
                leap_model = glm::translate(glm::mat4(1.0f), eye_pos) * glm::mat4_cast(body * head_q) *
                             glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.2f, -0.5f)) *
                             glm::rotate(glm::mat4(1.0f), glm::radians(25.0f), glm::vec3(1, 0, 0)) * glm::scale(glm::mat4(1.0f), glm::vec3(1.5f));
            } else {
                glm::vec3 cam;
                const bool have_cam = tracking_ok && tracker.camera_position(cam);
                if (have_cam && !leap_off_known) {   // first PTC fix: put the controller where it is expected to be relative to the seated head
                    leap_off = leap_seat - cam;
                    leap_off_known = true;
                    std::printf("Leap placed %.2f right, %.2f up, %.2f forward of your head (Alt+arrows to adjust)\n", leap_seat.x, leap_seat.y, -leap_seat.z);
                }
                if (use_leap && alt_held) {   // nudge: Alt+Left/Right = left/right, Alt+Up/Down = away/towards you, Alt+PageUp/PageDown = up/down
                    const Uint8* nk = keys;
                    glm::vec3 d(float(nk[SDL_SCANCODE_RIGHT]) - float(nk[SDL_SCANCODE_LEFT]), float(nk[SDL_SCANCODE_PAGEUP]) - float(nk[SDL_SCANCODE_PAGEDOWN]),
                                float(nk[SDL_SCANCODE_DOWN]) - float(nk[SDL_SCANCODE_UP]));   // +Z is towards you
                    if (nk[SDL_SCANCODE_BACKSPACE] && have_cam) { leap_off = leap_seat - cam; leap_dirty = true; leap_nudged = tick; std::printf("Leap placement reset\n"); }
                    if (glm::length(d) > 0) {
                        (have_cam ? leap_off : leap_seat) += d * (0.15f * dt);
                        leap_dirty = true; leap_nudged = tick;
                    }
                }
                if (leap_dirty && tick - leap_nudged > 600 && !alt_held) {   // save once the key is released
                    leap_dirty = false;
                    if (leap_off_known) { save_leap_offset(leap_off); std::printf("Leap placement saved: %.2f right, %.2f up, %.2f forward of the PTC\n", leap_off.x, leap_off.y, -leap_off.z); }
                }
                leap_rel = have_cam ? cam + leap_off : leap_seat;
                leap_model = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, eye_y, pos.y) + body * leap_rel) * glm::mat4_cast(body);
            }
        }
        if (use_leap) {   // hands in the controller's frame: tracked by the Ultraleap service, or simulated without it
            std::vector<TrackedHand> hands;
            if (leap_service && leap_tracker.connected()) hands = leap_tracker.hands();
            else if (sim_hand.state().present) hands.push_back(sim_hand.hand());
            std::vector<MapVertex> hv;
            for (const TrackedHand& h : hands) append_hand_geometry(h, hv);
            hand_vertices = GLsizei(hv.size());
            if (!hv.empty()) {
                glBindBuffer(GL_ARRAY_BUFFER, hand_vbo);
                glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(hv.size() * sizeof(MapVertex)), hv.data(), GL_STREAM_DRAW);
            }
        }
        audio.service();   // reconnect if the sound device was taken away or changed
        // Audio: listener at the ears, the nearest sound sources (with wall occlusion), and footsteps.
        {
            const glm::quat head_world = body * head_q;
            audio.set_listener(eye_pos, head_world);
            struct Cand { float d2; size_t i; };
            std::vector<Cand> near;
            for (size_t i = 0; i < all_sources.size(); ++i) {
                const glm::vec3 d = all_sources[i].pos - eye_pos;
                const float d2 = glm::dot(d, d);
                if (d2 < 18.0f * 18.0f) near.push_back({d2, i});
            }
            const size_t keep = std::min<size_t>(near.size(), 12);
            std::partial_sort(near.begin(), near.begin() + long(keep), near.end(), [](const Cand& a, const Cand& b) { return a.d2 < b.d2; });
            std::vector<SoundSource> active;
            for (size_t k = 0; k < keep; ++k) {
                SoundSource src = all_sources[near[k].i];
                src.occlusion = occlusion_between(map, eye_pos, src.pos);
                active.push_back(src);
            }
            audio.set_sources(active);
            // Step length scales with body height: ~0.5 H walking, ~0.82 H running (a 1.9 m person: 0.95 m and 1.56 m).
            const float stride = (run ? 0.82f : 0.50f) * user_height * stride_jitter;
            if (!fly && step_distance >= stride) {
                step_distance = 0.0f;
                stride_seed ^= stride_seed << 13; stride_seed ^= stride_seed >> 17; stride_seed ^= stride_seed << 5;
                stride_jitter = 0.88f + 0.24f * float(stride_seed & 0xffff) / 65536.0f;   // human steps are never evenly spaced
                step_foot = -step_foot;
                const int sk = SpatialAudio::step_kind(map.glyph(int(std::floor(pos.x)), int(std::floor(pos.y))).step);
                const glm::vec3 foot = glm::vec3(pos.x, eye_ground, pos.y) + body * (head_q * glm::vec3(0.12f * float(step_foot), 0.0f, 0.0f));
                audio.play_step(sk, foot, run ? 1.2f : 1.0f);
            }
        }
        const glm::mat4 camw = glm::translate(glm::mat4(1.0f), eye_pos) * glm::mat4_cast(body * head_q);
        const glm::mat4 view = glm::inverse(camw);

        int w, h;
        SDL_GL_GetDrawableSize(win, &w, &h);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, w, h);
        const glm::vec3 sky = map.options().sky;
        for (int eye = 0; eye < rig.eye_count(); ++eye) {
            rig.begin_eye(eye, glm::vec4(sky, 1.0f));
            glDisable(GL_CULL_FACE);
            const glm::mat4 mvp = rig.projection(eye) * rig.eye_offset(eye) * view;
            glUseProgram(prog);
            glUniformMatrix4fv(loc_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
            glUniform3fv(loc_eye, 1, glm::value_ptr(eye_pos));
            glUniform3fv(loc_sky, 1, glm::value_ptr(sky));
            glUniform1f(loc_fog, map.options().fog);
            glBindVertexArray(vao);
            glUniform1i(loc_tex, 0);
            glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
            glUniform1i(loc_use_tex, 0);
            glDrawArrays(GL_TRIANGLES, 0, vertex_count);
            for (const DrawBatch& b : batches) draw_batch(b);
            for (DynObject& d : dyn) {   // animated objects: one full turn about the vertical axis through the cell centre
                glm::mat4 m(1.0f);
                if (d.spinning) {
                    const float t = float(tick - d.start) / (d.duration * 1000.0f);
                    if (t >= 1.0f) d.spinning = false;
                    else {
                        const float ease = t * t * (3.0f - 2.0f * t);   // gentle start and stop; the turn is exactly 360 degrees
                        const glm::vec3 c(float(d.x) + 0.5f, 0.0f, float(d.z) + 0.5f);
                        m = glm::translate(glm::mat4(1.0f), c) * glm::rotate(glm::mat4(1.0f), 6.2831853f * ease, glm::vec3(0, 1, 0)) * glm::translate(glm::mat4(1.0f), -c);
                    }
                }
                glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(m));
                for (const DrawBatch& b : d.parts) draw_batch(b);
            }
            if (use_leap) {   // the controller: always visible, so the depth buffer is cleared first (it is close to you; terrain must not hide it)
                glClear(GL_DEPTH_BUFFER_BIT);
                glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(leap_model));
                glUniform1i(loc_use_tex, 0);
                glDrawArrays(GL_TRIANGLES, leap_first[leap.present ? 1 : 0], leap_count);
                if (hand_vertices > 0) {   // the hand, flat shapes drawn back to front (painter's order), so no depth test
                    glDisable(GL_DEPTH_TEST);
                    glBindVertexArray(hand_vao);
                    glDrawArrays(GL_TRIANGLES, 0, hand_vertices);
                    glBindVertexArray(vao);
                    glEnable(GL_DEPTH_TEST);
                }
            }
            glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
            if (reach > 0) {   // crosshair: an inverting dot on the gaze ray, at the depth of what it points at
                const float d = std::max(0.3f, target.dist - 0.03f);
                const bool pulse = tick < pulse_until;
                const float deg = pulse ? 1.2f : (target.hit ? 0.6f : 0.35f);   // angular radius: grows when something can be used
                const GLboolean depth_was_on = glIsEnabled(GL_DEPTH_TEST);
                glDisable(GL_DEPTH_TEST);
                glEnable(GL_BLEND);
                glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ONE_MINUS_SRC_ALPHA);
                glUseProgram(dot_prog);
                glUniformMatrix4fv(dot_mvp, 1, GL_FALSE, glm::value_ptr(mvp));
                glUniform3fv(dot_centre, 1, glm::value_ptr(glm::vec3(eye_pos + gaze * d)));
                glUniform3fv(dot_right, 1, glm::value_ptr(right3));
                glUniform3fv(dot_up, 1, glm::value_ptr(glm::vec3(body * head_q * glm::vec3(0, 1, 0))));
                glUniform1f(dot_radius, d * std::tan(deg * 3.14159265f / 180.0f));
                glUniform1f(dot_alpha, pulse ? 1.0f : (target.hit ? 0.8f : 0.55f));
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                glDisable(GL_BLEND);
                if (depth_was_on) glEnable(GL_DEPTH_TEST);
            }
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

        const int cx = int(std::floor(pos.x)), cz = int(std::floor(pos.y));
        if (cx != last_cell_x || cz != last_cell_z) {
            last_cell_x = cx;
            last_cell_z = cz;
            std::printf("Cell (%d,%d) '%c' %s%s\n", cx, cz, map.symbol(cx, cz), map.glyph(cx, cz).label.c_str(),
                        map.has_heights() ? (", elevation " + std::to_string(map.ground_height(pos.x, pos.y)).substr(0, 4) + " m").c_str() : "");
        }
        if (tick - last_print > 5000) {
            std::printf("%s  pos (%.1f, %.1f)%s\n", fly ? "fly" : "walk", pos.x, pos.y,
                        tracking_ok && tracker.camera_active() ? (tracker.camera_tracking() ? "  PTC tracking" : "  PTC lost") : "");
            if (tracking_ok && tracker.camera_active() && !tracker.camera_tracking()) {
                const auto c = tracker.camera_status();
                std::printf("  camera: %d LEDs seen in the last fix (need %d); rejected so far: too few LEDs %d, disagrees with IMU %d, position jump %d\n",
                            c.last_leds, 8, c.rejected_leds, c.rejected_orientation, c.rejected_jump);
            }
            last_print = tick;
        }
    }

    audio.close();
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
