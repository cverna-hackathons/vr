#include "gltf_model.hpp"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_INCLUDE_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE_WRITE
#include <tiny_gltf.h>

namespace {
float srgb_from_linear(float c) {
    c = std::min(1.0f, std::max(0.0f, c));
    return c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

size_t component_size(int type) {
    switch (type) {
        case TINYGLTF_COMPONENT_TYPE_BYTE: case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: return 1;
        case TINYGLTF_COMPONENT_TYPE_SHORT: case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: return 2;
        default: return 4;
    }
}

// Pointer to element `i` of an accessor, or nullptr when the accessor has no readable data.
const unsigned char* element(const tinygltf::Model& m, const tinygltf::Accessor& a, size_t i) {
    if (a.bufferView < 0 || a.bufferView >= int(m.bufferViews.size()) || i >= a.count) return nullptr;
    const tinygltf::BufferView& bv = m.bufferViews[size_t(a.bufferView)];
    if (bv.buffer < 0 || bv.buffer >= int(m.buffers.size())) return nullptr;
    const tinygltf::Buffer& buf = m.buffers[size_t(bv.buffer)];
    const int comps = tinygltf::GetNumComponentsInType(uint32_t(a.type));
    const size_t esize = size_t(comps) * component_size(a.componentType);
    const int stride = a.ByteStride(bv);
    const size_t step = stride > 0 ? size_t(stride) : esize;
    const size_t off = bv.byteOffset + a.byteOffset + i * step;
    if (off + esize > buf.data.size()) return nullptr;
    return buf.data.data() + off;
}

float read_component(const unsigned char* p, int type, bool normalized) {
    switch (type) {
        case TINYGLTF_COMPONENT_TYPE_FLOAT: { float f; std::memcpy(&f, p, 4); return f; }
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: return normalized ? float(*p) / 255.0f : float(*p);
        case TINYGLTF_COMPONENT_TYPE_BYTE: return normalized ? std::max(float(int8_t(*p)) / 127.0f, -1.0f) : float(int8_t(*p));
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: { uint16_t s; std::memcpy(&s, p, 2); return normalized ? float(s) / 65535.0f : float(s); }
        case TINYGLTF_COMPONENT_TYPE_SHORT: { int16_t s; std::memcpy(&s, p, 2); return normalized ? std::max(float(s) / 32767.0f, -1.0f) : float(s); }
        default: return 0.0f;
    }
}

// Reads an accessor as `n` floats per element (missing components are 0). False if the data cannot be read.
bool read_floats(const tinygltf::Model& m, int accessor, int n, std::vector<float>& out) {
    if (accessor < 0 || accessor >= int(m.accessors.size())) return false;
    const tinygltf::Accessor& a = m.accessors[size_t(accessor)];
    const int comps = tinygltf::GetNumComponentsInType(uint32_t(a.type));
    const size_t csize = component_size(a.componentType);
    out.assign(a.count * size_t(n), 0.0f);
    for (size_t i = 0; i < a.count; ++i) {
        const unsigned char* p = element(m, a, i);
        if (!p) return false;
        for (int c = 0; c < std::min(comps, n); ++c) out[i * size_t(n) + size_t(c)] = read_component(p + size_t(c) * csize, a.componentType, a.normalized);
    }
    return true;
}

bool read_indices(const tinygltf::Model& m, int accessor, std::vector<uint32_t>& out) {
    if (accessor < 0 || accessor >= int(m.accessors.size())) return false;
    const tinygltf::Accessor& a = m.accessors[size_t(accessor)];
    out.resize(a.count);
    for (size_t i = 0; i < a.count; ++i) {
        const unsigned char* p = element(m, a, i);
        if (!p) return false;
        if (a.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) out[i] = *p;
        else if (a.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) { uint16_t s; std::memcpy(&s, p, 2); out[i] = s; }
        else if (a.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) { uint32_t u; std::memcpy(&u, p, 4); out[i] = u; }
        else return false;
    }
    return true;
}

glm::mat4 node_matrix(const tinygltf::Node& n) {
    if (n.matrix.size() == 16) {
        float f[16];
        for (int i = 0; i < 16; ++i) f[i] = float(n.matrix[size_t(i)]);
        return glm::make_mat4(f);
    }
    glm::mat4 t(1.0f);
    if (n.translation.size() == 3) t = glm::translate(t, glm::vec3(float(n.translation[0]), float(n.translation[1]), float(n.translation[2])));
    if (n.rotation.size() == 4) t *= glm::mat4_cast(glm::quat(float(n.rotation[3]), float(n.rotation[0]), float(n.rotation[1]), float(n.rotation[2])));
    if (n.scale.size() == 3) t = glm::scale(t, glm::vec3(float(n.scale[0]), float(n.scale[1]), float(n.scale[2])));
    return t;
}

struct Builder {
    const tinygltf::Model& gm;
    Model& out;
    std::map<int, size_t> part_of_image;       // glTF image index (or -1) -> index into out.parts
    std::map<int, int> out_image_of;           // glTF image index -> index into out.images
    std::string warning;
    bool any_bounds = false;

    int image_for(int gltf_image) {
        auto it = out_image_of.find(gltf_image);
        if (it != out_image_of.end()) return it->second;
        const tinygltf::Image& im = gm.images[size_t(gltf_image)];
        int idx = -1;
        if (im.width > 0 && im.height > 0 && im.bits == 8 && im.component >= 1 && im.component <= 4 &&
            im.image.size() >= size_t(im.width) * size_t(im.height) * size_t(im.component)) {
            ModelImage mi;
            mi.w = im.width;
            mi.h = im.height;
            mi.rgba.resize(size_t(mi.w) * size_t(mi.h) * 4);
            for (size_t p = 0; p < size_t(mi.w) * size_t(mi.h); ++p) {
                const unsigned char* s = &im.image[p * size_t(im.component)];
                unsigned char* d = &mi.rgba[p * 4];
                switch (im.component) {
                    case 1: d[0] = d[1] = d[2] = s[0]; d[3] = 255; break;
                    case 2: d[0] = d[1] = d[2] = s[0]; d[3] = s[1]; break;
                    case 3: d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = 255; break;
                    default: d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3]; break;
                }
            }
            out.images.push_back(std::move(mi));
            idx = int(out.images.size()) - 1;
        } else {
            warning = "a texture image could not be decoded (8-bit PNG/JPEG expected); it is drawn untextured";
        }
        out_image_of[gltf_image] = idx;
        return idx;
    }

    ModelPart& part(int image) {
        auto it = part_of_image.find(image);
        if (it != part_of_image.end()) return out.parts[it->second];
        out.parts.emplace_back();
        out.parts.back().image = image;
        part_of_image[image] = out.parts.size() - 1;
        return out.parts.back();
    }

    void primitive(const tinygltf::Primitive& prim, const glm::mat4& xf) {
        if (prim.mode != TINYGLTF_MODE_TRIANGLES) { warning = "only triangle primitives are supported; others skipped"; return; }
        auto pos_it = prim.attributes.find("POSITION");
        if (pos_it == prim.attributes.end()) return;
        std::vector<float> P, N, T;
        if (!read_floats(gm, pos_it->second, 3, P)) { warning = "unreadable POSITION data (sparse or missing buffer); primitive skipped"; return; }
        const size_t nv = P.size() / 3;
        auto n_it = prim.attributes.find("NORMAL");
        const bool has_n = n_it != prim.attributes.end() && read_floats(gm, n_it->second, 3, N) && N.size() == P.size();
        auto t_it = prim.attributes.find("TEXCOORD_0");
        const bool has_t = t_it != prim.attributes.end() && read_floats(gm, t_it->second, 2, T) && T.size() / 2 == nv;

        std::vector<uint32_t> idx;
        if (prim.indices >= 0) { if (!read_indices(gm, prim.indices, idx)) { warning = "unreadable index data; primitive skipped"; return; } }
        else { idx.resize(nv); for (size_t i = 0; i < nv; ++i) idx[i] = uint32_t(i); }

        glm::vec3 colour(1.0f);
        int image = -1;
        if (prim.material >= 0 && prim.material < int(gm.materials.size())) {
            const tinygltf::PbrMetallicRoughness& pbr = gm.materials[size_t(prim.material)].pbrMetallicRoughness;
            if (pbr.baseColorFactor.size() >= 3)
                colour = glm::vec3(srgb_from_linear(float(pbr.baseColorFactor[0])), srgb_from_linear(float(pbr.baseColorFactor[1])), srgb_from_linear(float(pbr.baseColorFactor[2])));
            const int ti = pbr.baseColorTexture.index;
            if (ti >= 0 && ti < int(gm.textures.size())) {
                const int src = gm.textures[size_t(ti)].source;
                if (src >= 0 && src < int(gm.images.size())) image = image_for(src);
            }
        }
        // Textured parts: the texture supplies the colour; baseColorFactor still tints it.
        const glm::mat3 nrm = glm::inverseTranspose(glm::mat3(xf));
        const bool mirrored = glm::determinant(glm::mat3(xf)) < 0.0f;
        ModelPart& dst = part(image);
        for (size_t i = 0; i + 2 < idx.size(); i += 3) {
            uint32_t tri[3] = {idx[i], idx[i + 1], idx[i + 2]};
            if (tri[0] >= nv || tri[1] >= nv || tri[2] >= nv) continue;
            if (mirrored) std::swap(tri[1], tri[2]);
            glm::vec3 p[3];
            for (int k = 0; k < 3; ++k) p[k] = glm::vec3(xf * glm::vec4(P[tri[k] * 3], P[tri[k] * 3 + 1], P[tri[k] * 3 + 2], 1.0f));
            glm::vec3 flat = glm::cross(p[1] - p[0], p[2] - p[0]);
            if (glm::dot(flat, flat) < 1e-14f) continue;
            flat = glm::normalize(flat);
            for (int k = 0; k < 3; ++k) {
                glm::vec3 n = flat;
                if (has_n) {
                    n = nrm * glm::vec3(N[tri[k] * 3], N[tri[k] * 3 + 1], N[tri[k] * 3 + 2]);
                    n = glm::dot(n, n) > 1e-12f ? glm::normalize(n) : flat;
                }
                MapVertex v;
                v.pos[0] = p[k].x; v.pos[1] = p[k].y; v.pos[2] = p[k].z;
                v.normal[0] = n.x; v.normal[1] = n.y; v.normal[2] = n.z;
                v.color[0] = colour.x; v.color[1] = colour.y; v.color[2] = colour.z;
                v.uv[0] = has_t ? T[tri[k] * 2] : 0.0f;
                v.uv[1] = has_t ? T[tri[k] * 2 + 1] : 0.0f;
                dst.verts.push_back(v);
                if (!any_bounds) { out.lo = out.hi = p[k]; any_bounds = true; }
                out.lo = glm::min(out.lo, p[k]);
                out.hi = glm::max(out.hi, p[k]);
            }
        }
    }

    void node(int index, const glm::mat4& parent, int depth) {
        if (index < 0 || index >= int(gm.nodes.size()) || depth > 64) return;
        const tinygltf::Node& n = gm.nodes[size_t(index)];
        const glm::mat4 xf = parent * node_matrix(n);
        if (n.mesh >= 0 && n.mesh < int(gm.meshes.size()))
            for (const tinygltf::Primitive& prim : gm.meshes[size_t(n.mesh)].primitives) primitive(prim, xf);
        for (int c : n.children) node(c, xf, depth + 1);
    }
};
}  // namespace

bool load_gltf_model(const std::string& path, Model& out, std::string& error) {
    tinygltf::TinyGLTF loader;
    tinygltf::Model gm;
    std::string err, warn;
    const bool glb = path.size() >= 4 && (path.compare(path.size() - 4, 4, ".glb") == 0 || path.compare(path.size() - 4, 4, ".GLB") == 0);
    const bool ok = glb ? loader.LoadBinaryFromFile(&gm, &err, &warn, path) : loader.LoadASCIIFromFile(&gm, &err, &warn, path);
    if (!ok) { error = path + ": " + (err.empty() ? "cannot load glTF" : err); return false; }
    out = Model();
    Builder b{gm, out, {}, {}, {}, false};
    const int scene = gm.defaultScene >= 0 ? gm.defaultScene : 0;
    if (scene < int(gm.scenes.size())) {
        for (int n : gm.scenes[size_t(scene)].nodes) b.node(n, glm::mat4(1.0f), 0);
    } else {
        for (size_t i = 0; i < gm.nodes.size(); ++i) b.node(int(i), glm::mat4(1.0f), 0);  // no scene: draw every node
    }
    if (out.parts.empty() || out.triangles() == 0) { error = path + ": the file contains no triangles"; return false; }
    if (!b.warning.empty()) std::fprintf(stderr, "Warning: %s: %s\n", path.c_str(), b.warning.c_str());
    return true;
}
