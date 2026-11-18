// Loads a glTF 2.0 file (.gltf or .glb, e.g. exported from Blender) into a flat triangle list for the explore map renderer.
// Supported: scene node hierarchy (matrix or TRS), triangle primitives (indexed or not), POSITION / NORMAL / TEXCOORD_0,
// material base colour factor and base colour texture (PNG/JPEG), alphaMode MASK/BLEND treated as a cut-out.
// Not supported: skins, animation, morph targets, sparse accessors, other texture maps (normal, roughness, emissive).
#pragma once
#include <string>

#include "mesh_types.hpp"

bool load_gltf_model(const std::string& path, Model& out, std::string& error);
