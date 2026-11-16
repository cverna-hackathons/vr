#include "mapdata.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

const char* kBuiltinGlossary =
    "; symbol kind [key=value ...] | label\n"
    "#  wall   color=#8a8f98 height=2.5 | Stone wall\n"
    "W  wall   color=#8b5a2b height=2.5 | Wooden wall\n"
    "H  wall   color=#2f6d2a height=1.4 | Hedge\n"
    ".  floor  color=#6b6f73 | Stone floor\n"
    ",  floor  color=#4f8a3c step=grass | Grass\n"
    ":  floor  color=#c9b27c step=sand | Sand\n"
    "~  water  color=#2a6fb0 | Water\n"
    "T  tree   color=#2f7d32 color2=#6b4423 height=3.2 | Tree\n"
    "o  rock   color=#7d7b78 height=0.9 | Boulder\n"
    "I  pillar color=#b9b4a8 height=2.5 | Pillar\n"
    "+  door   color=#8b5a2b color2=#5c3a1a height=2.1 | Doorway\n"
    "@  start  color=#e0c040 | Start position\n"
    "*  item   color=#ffd23f | Treasure\n"
    "space empty | Void\n";

namespace {
std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

bool parse_colour(const std::string& v, glm::vec3& out) {
    if (v.size() < 2 || v[0] != '#') return false;
    std::string h = v.substr(1);
    if (h.size() == 3) h = std::string{h[0], h[0], h[1], h[1], h[2], h[2]};
    if (h.size() != 6) return false;
    char* end = nullptr;
    const long x = std::strtol(h.c_str(), &end, 16);
    if (*end) return false;
    out = glm::vec3(float((x >> 16) & 255), float((x >> 8) & 255), float(x & 255)) / 255.0f;
    return true;
}

bool parse_bool(const std::string& v, bool& out) {
    if (v == "yes" || v == "true" || v == "1") { out = true; return true; }
    if (v == "no" || v == "false" || v == "0") { out = false; return true; }
    return false;
}

float hash01(int x, int z) { return float((unsigned(x) * 73856093u ^ unsigned(z) * 19349663u) % 1000u) / 1000.0f; }

// ---- mesh helpers ------------------------------------------------------------------------------
struct Mesh {
    std::vector<MapVertex> v;
    void tri(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 col, glm::vec3 inside_hint) {
        glm::vec3 n = glm::cross(b - a, c - a);
        if (glm::dot(n, n) < 1e-12f) return;
        n = glm::normalize(n);
        if (glm::dot(n, (a + b + c) / 3.0f - inside_hint) < 0) n = -n;
        for (const glm::vec3& p : {a, b, c}) v.push_back({{p.x, p.y, p.z}, {n.x, n.y, n.z}, {col.x, col.y, col.z}});
    }
    void quad(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 col, glm::vec3 hint) {
        tri(a, b, c, col, hint);
        tri(a, c, d, col, hint);
    }
    // Axis-aligned box; `skip` bits: 1 +x, 2 -x, 4 +y, 8 -y, 16 +z, 32 -z.
    void box(glm::vec3 lo, glm::vec3 hi, glm::vec3 col, int skip = 8) {
        const glm::vec3 c = (lo + hi) * 0.5f;
        const glm::vec3 p[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                                {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
        if (!(skip & 1)) quad(p[1], p[2], p[6], p[5], col, c);
        if (!(skip & 2)) quad(p[0], p[3], p[7], p[4], col, c);
        if (!(skip & 4)) quad(p[3], p[2], p[6], p[7], col, c);
        if (!(skip & 8)) quad(p[0], p[1], p[5], p[4], col, c);
        if (!(skip & 16)) quad(p[4], p[5], p[6], p[7], col, c);
        if (!(skip & 32)) quad(p[0], p[1], p[2], p[3], col, c);
    }
    // Vertical frustum with `sides` sides: r0 at y0, r1 at y1 (r1 = 0 gives a cone, r0 = r1 a prism).
    void frustum(float cx, float cz, float r0, float r1, float y0, float y1, int sides, glm::vec3 col) {
        const glm::vec3 centre(cx, (y0 + y1) * 0.5f, cz);
        for (int i = 0; i < sides; ++i) {
            const float a0 = 6.2831853f * i / sides, a1 = 6.2831853f * (i + 1) / sides;
            const glm::vec3 b0(cx + r0 * std::cos(a0), y0, cz + r0 * std::sin(a0)), b1(cx + r0 * std::cos(a1), y0, cz + r0 * std::sin(a1));
            const glm::vec3 t0(cx + r1 * std::cos(a0), y1, cz + r1 * std::sin(a0)), t1(cx + r1 * std::cos(a1), y1, cz + r1 * std::sin(a1));
            tri(b0, b1, t1, col, centre);
            if (r1 > 0) tri(b0, t1, t0, col, centre);
            if (r1 > 0) tri(glm::vec3(cx, y1, cz), t0, t1, col, centre);
        }
    }
    void octahedron(glm::vec3 c, float r, glm::vec3 col) {
        const glm::vec3 px = c + glm::vec3(r, 0, 0), nx = c - glm::vec3(r, 0, 0), py = c + glm::vec3(0, r * 1.5f, 0),
                        ny = c - glm::vec3(0, r * 1.5f, 0), pz = c + glm::vec3(0, 0, r), nz = c - glm::vec3(0, 0, r);
        const glm::vec3 eq[4] = {px, pz, nx, nz};
        for (int i = 0; i < 4; ++i) {
            tri(eq[i], eq[(i + 1) % 4], py, col, c);
            tri(eq[i], eq[(i + 1) % 4], ny, col, c);
        }
    }
};
}  // namespace

// ---- glossary --------------------------------------------------------------------------------------

bool GameMap::apply_glossary_text(const std::string& text, const std::string& origin, std::string& error) {
    std::istringstream in(text);
    std::string raw;
    int lineno = 0;
    while (std::getline(in, raw)) {
        ++lineno;
        const std::string line = trim(raw);
        if (line.empty() || line[0] == ';') continue;
        std::string label;
        std::string head = line;
        const size_t bar = line.find('|');
        if (bar != std::string::npos) { head = trim(line.substr(0, bar)); label = trim(line.substr(bar + 1)); }
        std::istringstream ts(head);
        std::vector<std::string> tok;
        for (std::string t; ts >> t;) tok.push_back(t);
        auto fail = [&](const std::string& why) {
            error = origin + ":" + std::to_string(lineno) + ": " + why + "  (\"" + line + "\")";
            return false;
        };
        if (tok.empty()) continue;
        if (tok[0] == "option") {
            for (size_t i = 1; i < tok.size(); ++i) {
                const size_t eq = tok[i].find('=');
                if (eq == std::string::npos) return fail("expected key=value");
                const std::string k = tok[i].substr(0, eq), v = tok[i].substr(eq + 1);
                bool ok = true;
                if (k == "sky") ok = parse_colour(v, opt_.sky);
                else if (k == "fog") opt_.fog = float(std::atof(v.c_str()));
                else if (k == "ceiling") ok = parse_bool(v, opt_.ceiling);
                else if (k == "ceiling_height") opt_.ceiling_height = float(std::atof(v.c_str()));
                else if (k == "height_step") opt_.height_step = float(std::atof(v.c_str()));
                else return fail("unknown option '" + k + "'");
                if (!ok) return fail("bad value for '" + k + "'");
            }
            continue;
        }
        Glyph g;
        if (tok[0] == "space") g.sym = ' ';
        else if (tok[0].size() == 1) g.sym = tok[0][0];
        else return fail("symbol must be one character (or 'space')");
        if (tok.size() < 2) return fail("missing kind");
        g.kind = tok[1];
        static const char* kinds[] = {"wall", "floor", "water", "tree", "rock", "pillar", "door", "item", "start", "empty"};
        if (std::find_if(std::begin(kinds), std::end(kinds), [&](const char* k) { return g.kind == k; }) == std::end(kinds))
            return fail("unknown kind '" + g.kind + "'");
        g.label = label.empty() ? g.kind : label;
        const bool solid_kind = g.kind == "wall" || g.kind == "water" || g.kind == "tree" || g.kind == "rock" || g.kind == "pillar" || g.kind == "empty";
        g.walk = !solid_kind;
        if (g.kind == "wall") g.height = 2.5f;
        else if (g.kind == "tree") g.height = 3.0f;
        else if (g.kind == "pillar") g.height = 2.5f;
        else if (g.kind == "rock") g.height = 0.8f;
        else if (g.kind == "door") g.height = 2.1f;
        g.has_floor = g.kind != "empty";
        bool sound_set = false, step_set = false;
        for (size_t i = 2; i < tok.size(); ++i) {
            const size_t eq = tok[i].find('=');
            if (eq == std::string::npos) return fail("expected key=value, got '" + tok[i] + "'");
            const std::string k = tok[i].substr(0, eq), v = tok[i].substr(eq + 1);
            bool ok = true;
            if (k == "color") ok = parse_colour(v, g.color);
            else if (k == "color2") ok = parse_colour(v, g.color2);
            else if (k == "floor") { if (v == "no") g.has_floor = false; else { ok = parse_colour(v, g.floor_color); g.has_floor = true; g.floor_set = true; } }
            else if (k == "height") g.height = float(std::atof(v.c_str()));
            else if (k == "walk") ok = parse_bool(v, g.walk);
            else if (k == "sound") { ok = v == "none" || v == "water" || v == "leaves" || v == "bird" || v == "hum" || v == "fire"; g.sound = v; sound_set = true; }
            else if (k == "step") { ok = v == "none" || v == "stone" || v == "grass" || v == "sand" || v == "wood"; g.step = v; step_set = true; }
            else return fail("unknown key '" + k + "'");
            if (!ok) return fail("bad value for '" + k + "'");
        }
        if (g.kind == "floor" || g.kind == "start" || g.kind == "water") g.floor_color = g.color;
        // Sound defaults by kind (a glossary entry can override them with sound= / step=).
        if (!sound_set) g.sound = g.kind == "water" ? "water" : g.kind == "tree" ? "leaves" : g.kind == "item" ? "hum" : "none";
        if (!step_set) g.step = (g.kind == "floor" || g.kind == "start" || g.kind == "item") ? "stone" : g.kind == "door" ? "wood" : "none";
        glossary_[g.sym] = g;
    }
    return true;
}

// ---- loading ---------------------------------------------------------------------------------------

bool GameMap::load(const std::string& map_file, const std::string& glossary_file, std::string& error) {
    rows_.clear();
    glossary_.clear();
    warnings_.clear();
    unknown_seen_.clear();
    opt_ = MapOptions();
    if (!apply_glossary_text(kBuiltinGlossary, "built-in glossary", error)) return false;
    if (!glossary_file.empty()) {
        std::ifstream gf(glossary_file);
        if (!gf) { error = "cannot open glossary " + glossary_file; return false; }
        std::stringstream ss;
        ss << gf.rdbuf();
        if (!apply_glossary_text(ss.str(), glossary_file, error)) return false;
    }
    std::ifstream f(map_file);
    if (!f) { error = "cannot open map " + map_file; return false; }
    std::vector<std::string> lines;
    for (std::string l; std::getline(f, l);) {
        if (!l.empty() && l.back() == '\r') l.pop_back();
        for (char& c : l) if (c == '\t') c = ' ';
        lines.push_back(l);
    }
    bool sectioned = false;
    for (const std::string& l : lines) if (trim(l) == "@map" || trim(l) == "@glossary" || trim(l) == "@height") sectioned = true;
    std::vector<std::string> hrows;

    std::string gloss_text;
    if (!sectioned) {
        rows_ = lines;
    } else {
        int mode = 0;  // 0 ignore, 1 glossary, 2 map, 3 height
        for (const std::string& l : lines) {
            const std::string t = trim(l);
            if (t == "@glossary") { mode = 1; continue; }
            if (t == "@map") { mode = 2; continue; }
            if (t == "@height") { mode = 3; continue; }
            if (mode == 1) gloss_text += l + "\n";
            else if (mode == 2) rows_.push_back(l);
            else if (mode == 3) hrows.push_back(l);
        }
    }
    if (!gloss_text.empty() && !apply_glossary_text(gloss_text, map_file + " @glossary", error)) return false;
    while (!rows_.empty() && trim(rows_.back()).empty()) rows_.pop_back();
    if (rows_.empty()) { error = map_file + ": the map has no rows"; return false; }
    w_ = 0;
    for (const std::string& r : rows_) w_ = std::max(w_, int(r.size()));
    h_ = int(rows_.size());
    for (std::string& r : rows_) r.resize(size_t(w_), ' ');

    void_ = glossary_.count(' ') ? glossary_[' '] : Glyph();
    unknown_ = Glyph();
    unknown_.sym = '?';
    unknown_.kind = "pillar";
    unknown_.label = "undefined symbol";
    unknown_.color = glm::vec3(1.0f, 0.1f, 0.9f);
    unknown_.height = 0.6f;
    unknown_.walk = false;
    // Elevation layer.
    cell_h_.assign(size_t(w_) * size_t(h_), 0.0f);
    has_heights_ = false;
    while (!hrows.empty() && trim(hrows.back()).empty()) hrows.pop_back();
    if (!hrows.empty()) {
        has_heights_ = true;
        if (int(hrows.size()) != h_) warnings_.push_back("@height has " + std::to_string(hrows.size()) + " rows but the map has " + std::to_string(h_));
        bool bad_char = false;
        for (int z = 0; z < h_ && z < int(hrows.size()); ++z)
            for (int x = 0; x < w_ && x < int(hrows[size_t(z)].size()); ++x) {
                const char c = hrows[size_t(z)][size_t(x)];
                int v = 0;
                if (c >= '0' && c <= '9') v = c - '0';
                else if (c >= 'a' && c <= 'z') v = 10 + c - 'a';
                else if (c >= 'A' && c <= 'Z') v = 10 + c - 'A';
                else if (c != '.' && c != ' ') bad_char = true;
                cell_h_[size_t(z) * size_t(w_) + size_t(x)] = float(v) * opt_.height_step;
            }
        for (const std::string& r : hrows) if (int(r.size()) > w_) { warnings_.push_back("@height rows are wider than the map"); break; }
        if (bad_char) warnings_.push_back("@height contains characters other than 0-9, a-z, '.' and space (treated as 0)");
    }
    build_terrain();
    bool found = false;
    for (int z = 0; z < h_ && !found; ++z)
        for (int x = 0; x < w_ && !found; ++x)
            if (glyph(x, z).kind == "start") { start_ = glm::vec2(x + 0.5f, z + 0.5f); found = true; }
    for (int z = 0; z < h_ && !found; ++z)
        for (int x = 0; x < w_ && !found; ++x)
            if (glyph(x, z).walk && glyph(x, z).has_floor && glyph(x, z).kind != "empty") { start_ = glm::vec2(x + 0.5f, z + 0.5f); found = true; }
    if (!found) warnings_.push_back("no walkable cell found; starting at (1.5, 1.5)");
    for (const auto& u : unknown_seen_) warnings_.push_back(std::string("symbol '") + u.first + "' is not in the glossary (drawn as a magenta post)");
    return true;
}

char GameMap::symbol(int x, int z) const {
    if (x < 0 || z < 0 || x >= w_ || z >= h_) return ' ';
    return rows_[size_t(z)][size_t(x)];
}

const Glyph& GameMap::glyph(int x, int z) const {
    if (x < 0 || z < 0 || x >= w_ || z >= h_) return void_;
    const char c = rows_[size_t(z)][size_t(x)];
    auto it = glossary_.find(c);
    if (it != glossary_.end()) return it->second;
    const_cast<GameMap*>(this)->unknown_seen_[c] = true;
    return unknown_;
}

std::map<char, Glyph> GameMap::used_glyphs() const {
    std::map<char, Glyph> out;
    for (int z = 0; z < h_; ++z)
        for (int x = 0; x < w_; ++x) {
            const char c = symbol(x, z);
            out[c] = glyph(x, z);
            out[c].sym = c;
        }
    return out;
}

// ---- collision -------------------------------------------------------------------------------------

bool GameMap::blocked(glm::vec2 p) const {
    const float r = 0.25f;
    for (int dz = -1; dz <= 1; dz += 2)
        for (int dx = -1; dx <= 1; dx += 2) {
            const int cx = int(std::floor(p.x + dx * r)), cz = int(std::floor(p.y + dz * r));
            if (!walkable(cx, cz)) return true;
        }
    return false;
}

void GameMap::build_terrain() {
    corner_h_.assign(size_t(w_ + 1) * size_t(h_ + 1), 0.0f);
    min_h_ = max_h_ = 0.0f;
    bool first = true;
    for (int j = 0; j <= h_; ++j)
        for (int i = 0; i <= w_; ++i) {
            float sum = 0, all = 0, water = 1e9f;
            int n = 0, na = 0;
            for (int dz = -1; dz <= 0; ++dz)
                for (int dx = -1; dx <= 0; ++dx) {
                    const int cx = i + dx, cz = j + dz;
                    if (cx < 0 || cz < 0 || cx >= w_ || cz >= h_) continue;
                    const float ch = cell_h_[size_t(cz) * size_t(w_) + size_t(cx)];
                    const Glyph& g = glyph(cx, cz);
                    all += ch;
                    ++na;
                    if (g.kind == "water") water = std::min(water, ch);
                    if (g.kind == "wall" || g.kind == "empty" || g.kind == "water") continue;
                    sum += ch;
                    ++n;
                }
            float c = n ? sum / float(n) : (na ? all / float(na) : 0.0f);
            if (water < 1e8f) c = std::min(c, water);  // shores meet the water surface
            corner_h_[size_t(j) * size_t(w_ + 1) + size_t(i)] = c;
            if (first) { min_h_ = max_h_ = c; first = false; }
            min_h_ = std::min(min_h_, c);
            max_h_ = std::max(max_h_, c);
        }
}

float GameMap::ground_height(float x, float z) const {
    if (w_ == 0) return 0.0f;
    x = std::min(std::max(x, 0.0f), float(w_) - 1e-4f);
    z = std::min(std::max(z, 0.0f), float(h_) - 1e-4f);
    const int ix = int(x), iz = int(z);
    const float u = x - float(ix), v = z - float(iz);
    const float h00 = corner(ix, iz), h10 = corner(ix + 1, iz), h11 = corner(ix + 1, iz + 1), h01 = corner(ix, iz + 1);
    // Same two triangles as the rendered ground: (00,10,11) and (00,11,01).
    return u >= v ? h00 + u * (h10 - h00) + v * (h11 - h10) : h00 + u * (h11 - h01) + v * (h01 - h00);
}

bool GameMap::move(glm::vec2& pos, glm::vec2 delta) const {
    bool moved = false;
    const float len = glm::length(delta);
    const int steps = std::max(1, int(std::ceil(len / 0.1f)));
    const glm::vec2 d = delta / float(steps);
    const float max_rise = 1.2f * glm::length(d);  // steeper than ~50 degrees uphill is a cliff
    auto ok = [&](glm::vec2 from, glm::vec2 to) {
        return !blocked(to) && ground_height(to.x, to.y) - ground_height(from.x, from.y) <= max_rise;
    };
    for (int i = 0; i < steps; ++i) {
        const glm::vec2 t = pos + d;
        if (ok(pos, t)) { pos = t; moved = true; continue; }
        const glm::vec2 tx(pos.x + d.x, pos.y), tz(pos.x, pos.y + d.y);  // slide along the wall
        if (ok(pos, tx)) { pos = tx; moved = true; }
        else if (ok(pos, tz)) { pos = tz; moved = true; }
    }
    return moved;
}

// ---- mesh --------------------------------------------------------------------------------------------

bool GameMap::wall_extent(int x, int z, float& bottom, float& top) const {
    const Glyph& g = glyph(x, z);
    if (g.kind != "wall" || x < 0 || z < 0 || x >= w_ || z >= h_) return false;
    bottom = std::min(std::min(corner(x, z), corner(x + 1, z)), std::min(corner(x, z + 1), corner(x + 1, z + 1)));
    top = ground_height(float(x) + 0.5f, float(z) + 0.5f) + g.height;
    return true;
}

std::vector<MapVertex> GameMap::build_mesh() const {
    Mesh m;
    for (int z = 0; z < h_; ++z) {
        for (int x = 0; x < w_; ++x) {
            const Glyph& g = glyph(x, z);
            const float shade = 0.90f + 0.14f * hash01(x, z);
            const float x0 = float(x), x1 = x0 + 1.0f, z0 = float(z), z1 = z0 + 1.0f, cx = x0 + 0.5f, cz = z0 + 0.5f;
            const float h00 = corner(x, z), h10 = corner(x + 1, z), h11 = corner(x + 1, z + 1), h01 = corner(x, z + 1);
            const float base = ground_height(cx, cz);  // objects stand on the ground at the cell centre
            const float low = std::min(std::min(h00, h10), std::min(h11, h01));

            // Ground under the cell (water: a flat surface just below the shore level).
            if (g.kind != "wall" && g.has_floor && g.kind != "empty") {
                const bool water = g.kind == "water";
                glm::vec3 ground = g.floor_color;
                if (!g.floor_set && g.kind != "floor" && g.kind != "water") {
                    // Objects stand on the ground colour of the nearest plain floor cell (so a tree on sand has sand under it).
                    static const int nb[8][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
                    for (const auto& d : nb) {
                        const Glyph& n = glyph(x + d[0], z + d[1]);
                        if (n.kind == "floor") { ground = n.color; break; }
                    }
                }
                glm::vec3 col = (g.kind == "floor" || water) ? g.color : ground;
                if (g.kind == "start") col = glm::mix(ground, g.color, 0.8f);
                const glm::vec3 hint(cx, low - 1.0f, cz);  // so floor normals point up
                if (water) {
                    const float y = cell_h_[size_t(z) * size_t(w_) + size_t(x)] - 0.04f;
                    m.quad({x0, y, z0}, {x1, y, z0}, {x1, y, z1}, {x0, y, z1}, col * shade, hint);
                } else {
                    m.quad({x0, h00, z0}, {x1, h10, z0}, {x1, h11, z1}, {x0, h01, z1}, col * shade, hint);
                }
            }
            if (opt_.ceiling && g.kind != "wall" && g.kind != "empty") {
                const float y = base + opt_.ceiling_height;
                m.quad({x0, y, z0}, {x1, y, z0}, {x1, y, z1}, {x0, y, z1}, glm::vec3(0.22f, 0.22f, 0.25f) * shade, glm::vec3(cx, y + 1.0f, cz));
            }

            if (g.kind == "wall") {
                float bot, top;
                wall_extent(x, z, bot, top);
                int skip = 8;
                float nb, nt;
                if (wall_extent(x + 1, z, nb, nt) && nt >= top - 1e-3f && nb <= bot + 1e-3f) skip |= 1;
                if (wall_extent(x - 1, z, nb, nt) && nt >= top - 1e-3f && nb <= bot + 1e-3f) skip |= 2;
                if (wall_extent(x, z + 1, nb, nt) && nt >= top - 1e-3f && nb <= bot + 1e-3f) skip |= 16;
                if (wall_extent(x, z - 1, nb, nt) && nt >= top - 1e-3f && nb <= bot + 1e-3f) skip |= 32;
                m.box({x0, bot, z0}, {x1, top, z1}, g.color * shade, skip);
            } else if (g.kind == "tree") {
                const float h = g.height;
                m.box({cx - 0.12f, base, cz - 0.12f}, {cx + 0.12f, base + h * 0.45f, cz + 0.12f}, g.color2 * shade, 8 | 4);
                m.frustum(cx, cz, 0.48f, 0.0f, base + h * 0.28f, base + h, 8, g.color * shade);
            } else if (g.kind == "rock") {
                m.frustum(cx, cz, 0.42f, 0.24f, low, base + g.height, 6, g.color * shade);
            } else if (g.kind == "pillar") {
                m.frustum(cx, cz, 0.28f, 0.28f, low, base + g.height, 8, g.color * shade);
            } else if (g.kind == "door") {
                const float h = g.height, top = base + std::max(h + 0.3f, 2.5f);
                const bool we = !walkable(x - 1, z) && !walkable(x + 1, z), ns = !walkable(x, z - 1) && !walkable(x, z + 1);
                m.box({x0, base + h, z0}, {x1, top, z1}, g.color * shade, 8);  // lintel fills the opening up to wall height
                if (we) {
                    m.box({x0, low, z0}, {x0 + 0.12f, base + h, z1}, g.color2 * shade, 8 | 4);
                    m.box({x1 - 0.12f, low, z0}, {x1, base + h, z1}, g.color2 * shade, 8 | 4);
                } else if (ns) {
                    m.box({x0, low, z0}, {x1, base + h, z0 + 0.12f}, g.color2 * shade, 8 | 4);
                    m.box({x0, low, z1 - 0.12f}, {x1, base + h, z1}, g.color2 * shade, 8 | 4);
                }
            } else if (g.kind == "item") {
                m.octahedron({cx, base + 0.85f, cz}, 0.2f, g.color);
            }
        }
    }
    return m.v;
}
