// Text maps ("roguelike ASCII": one character per 1 m cell, top of the file = north) and their 3D mesh.
//
// File layout. A file may be just the grid. Or it may use sections, each starting with a line `@glossary` / `@map`:
//   @glossary        lines `<symbol> <kind> [key=value ...] [| label]`, `option key=value ...`; `;` starts a comment line
//   @map             every following line is a row of the grid ('#' is a wall here, not a comment)
// Symbols not defined by the file or by --glossary come from the built-in glossary (see kBuiltinGlossary in mapdata.cpp,
// printed by `explore --print-glossary`). Kinds: wall floor water tree rock pillar door item start empty model.
// Custom models: `<symbol> model file=data/objects/thing.glb [scale=1] [rotate=0] [yoffset=0] [walk=no]` draws a glTF 2.0 file (e.g. exported
// from Blender, Y up, origin at the foot) at the cell centre. `file` is looked up as given, then relative to the map file and its parent folders.
// Sound keys: sound=none|water|leaves|bird|hum|fire (the cell is a 3D sound source), step=none|stone|grass|sand|wood (footsteps).
// Actions: `action=spin [duration=4]` makes the object turn one full circle about its vertical axis when used (see explore).
// Keys: interact=yes|no (the crosshair in explore can pick it)  color=#rrggbb  color2=#rrggbb (trunk/posts)  floor=#rrggbb|no (ground under the object)  height=metres  walk=yes|no.
// Options: sky=#rrggbb  fog=density  ceiling=yes|no  ceiling_height=metres  height_step=metres
// Terrain: an optional `@height` section holds a second grid, parallel to the map, with the elevation of each cell as a digit
// (0-9, then a-z = 10-35; '.' and space = 0), in units of `height_step` (default 0.5 m). The ground is a smooth surface through those
// heights, so slopes appear between cells of different elevation. Walking uphill steeper than about 50 degrees is blocked.
#pragma once
#include <glm/glm.hpp>

#include "mesh_types.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

struct Glyph {
    char sym = '?';
    std::string kind = "floor", label;
    glm::vec3 color{0.5f}, color2{0.4f}, floor_color{0.42f, 0.45f, 0.42f};
    bool has_floor = true;
    bool floor_set = false;  // floor= given explicitly (otherwise objects take the ground colour of their neighbours)
    float height = 1.0f;
    bool walk = true;
    std::string model_file;      // kind "model": glTF file (.glb/.gltf) drawn at the cell centre
    std::string action;          // what interacting does: "" (nothing) or "spin"; setting it also makes the object interactable
    float duration = 4.0f;       // seconds the action takes (spin: one full turn about the vertical axis)
    bool interact = false;       // can be picked with the crosshair (default: door item tree rock pillar model; key interact=yes|no)
    float scale = 1.0f, rotate = 0.0f, yoffset = 0.0f;  // model: uniform scale, rotation about Y in degrees, lift in metres
    std::string sound = "none";  // positional sound this cell emits: none water leaves bird hum fire
    std::string step = "none";   // footstep sound when walking on it: none stone grass sand wood
};

struct MapOptions {
    glm::vec3 sky{0.45f, 0.62f, 0.85f};
    float fog = 0.035f;
    bool ceiling = false;
    float ceiling_height = 2.5f;
    float height_step = 0.5f;
};

// Triangles of placed models that share one texture (image == nullptr: untextured). Valid while the GameMap lives.
struct ModelBatch {
    const ModelImage* image = nullptr;
    std::vector<MapVertex> verts;
};

// Result of a line-of-sight test (GameMap::pick).
struct PickResult {
    bool hit = false;    // an interactable object was reached first
    int x = 0, z = 0;    // its cell (when hit)
    float dist = 0.0f;   // metres along the ray to whatever stopped it (object, wall or ground), or the maximum distance
};

class GameMap {
public:
    // Loads a map. `glossary_file` (optional, may be empty) is applied before the map file's own glossary section.
    bool load(const std::string& map_file, const std::string& glossary_file, std::string& error);

    int width() const { return w_; }
    int height() const { return h_; }
    char symbol(int x, int z) const;                    // ' ' outside the grid
    const Glyph& glyph(int x, int z) const;             // glossary entry of the cell (void outside the grid)
    bool walkable(int x, int z) const { return glyph(x, z).walk; }
    glm::vec2 start() const { return start_; }          // x, z in metres
    const MapOptions& options() const { return opt_; }
    const std::vector<std::string>& warnings() const { return warnings_; }
    std::map<char, Glyph> used_glyphs() const;          // glossary entries actually appearing in the map

    // Moves `pos` (x,z metres) by `delta`, sliding along walls. Radius 0.25 m. Returns true if it moved at all.
    bool move(glm::vec2& pos, glm::vec2 delta) const;
    bool blocked(glm::vec2 pos) const;
    float ground_height(float x, float z) const;        // terrain elevation in metres at a point
    bool has_heights() const { return has_heights_; }
    float min_height() const { return min_h_; }
    float max_height() const { return max_h_; }

    // build_mesh() is the static mesh: ground and objects, except objects of cells with an `action` (those animate on their own).
    // With (only_x, only_z) it builds just the object geometry of that one cell (no ground), in world coordinates.
    std::vector<MapVertex> build_mesh(int only_x = -1, int only_z = -1) const;
    std::vector<glm::ivec2> action_cells() const;        // cells whose glossary entry has an action
    // Line of sight from `origin` along `dir` (need not be normalised) for at most `max_dist` metres. Walls, solid objects and the
    // ground stop the ray; the result says whether what it reached first is interactable. Objects are approximated by a square
    // footprint and a height range, so it is a generous but cheap test.
    PickResult pick(glm::vec3 origin, glm::vec3 dir, float max_dist) const;
    std::vector<ModelBatch> build_models(int only_x = -1, int only_z = -1) const;   // placed glTF models, batched by texture (same cell rule as build_mesh)
    const Model* model_of(const Glyph& g) const;         // the loaded model of a `model` glyph (nullptr if none)

private:
    int w_ = 0, h_ = 0;
    std::vector<std::string> rows_;
    std::map<char, Glyph> glossary_;
    Glyph void_, unknown_;
    MapOptions opt_;
    glm::vec2 start_{1.5f, 1.5f};
    std::vector<std::string> warnings_;
    std::map<char, bool> unknown_seen_;
    std::string map_dir_;
    std::map<std::string, std::shared_ptr<const Model>> models_;  // by glossary `file=` value
    bool load_models(std::string& error);

    bool apply_glossary_text(const std::string& text, const std::string& origin, std::string& error);
    bool wall_extent(int x, int z, float& bottom, float& top) const;
    void build_terrain();
    float corner(int i, int j) const { return corner_h_[size_t(j) * size_t(w_ + 1) + size_t(i)]; }
    std::vector<float> cell_h_, corner_h_;   // cell elevations (metres) and smoothed corner elevations
    bool has_heights_ = false;
    float min_h_ = 0, max_h_ = 0;
};

// The glossary compiled into the program, in file syntax.
extern const char* kBuiltinGlossary;
