#pragma once

// Helpers shared by the asset loaders (not part of the public API).

#include "kinetic/animation/Skinning.h"
#include "kinetic/assets/GltfLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <optional>
#include <string>

struct ufbx_scene;

namespace kinetic::assets::detail {

inline std::string lowerExtension(const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

std::optional<anim::SkinnedModel> loadSkinnedModelGltf(const std::filesystem::path& path, std::string& error);
std::optional<anim::SkinnedModel> loadSkinnedModelFbx(const std::filesystem::path& path, std::string& error);
std::optional<SkeletalAnimationAsset> loadAnimationFbx(const std::filesystem::path& path, std::string& error);

// Merges vertices that are identical in every attribute (position, normal, uv, joints, weights)
// and rewrites the indices. FBX corners come out one vertex per triangle corner; welding them
// cuts the vertex count (and the per-frame skinning cost) several times.
void weldVertices(anim::SkinnedMesh& skin);

// First material of an FBX scene with a texture (paths as written in the file, unresolved).
MaterialDesc readFbxMaterial(const ufbx_scene& scene);

// Smooth per-vertex normals from the triangles (area weighted), for meshes that have none.
void computeNormals(MeshData& mesh);

} // namespace kinetic::assets::detail
