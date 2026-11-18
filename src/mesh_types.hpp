// Vertex and model types shared by the map mesh builder (mapdata) and the glTF loader (gltf_model).
#pragma once
#include <glm/glm.hpp>

#include <vector>

// One vertex of the map mesh. `uv` is only used by textured model parts (it is 0 elsewhere).
struct MapVertex { float pos[3], normal[3], color[3], uv[2]; };

struct ModelImage {
    int w = 0, h = 0;
    std::vector<unsigned char> rgba;  // w*h*4, first row = top (glTF uv origin is top-left)
};

// Triangles of one model that share a material texture (`image` indexes Model::images, -1 = untextured).
// Vertices are in model space in metres, Y up, already flattened through the glTF node hierarchy.
struct ModelPart {
    std::vector<MapVertex> verts;
    int image = -1;
};

struct Model {
    std::vector<ModelPart> parts;
    std::vector<ModelImage> images;
    glm::vec3 lo{0.0f}, hi{0.0f};  // bounding box
    size_t triangles() const {
        size_t n = 0;
        for (const ModelPart& p : parts) n += p.verts.size() / 3;
        return n;
    }
};
