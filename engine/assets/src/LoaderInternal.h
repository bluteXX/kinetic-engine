#pragma once

// Helpers shared by the asset loaders (not part of the public API).

#include "kinetic/animation/Skinning.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <optional>
#include <string>

namespace kinetic::assets::detail {

inline std::string lowerExtension(const std::filesystem::path& path)
{
    std::string ext = path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

std::optional<anim::SkinnedModel> loadSkinnedModelGltf(const std::filesystem::path& path, std::string& error);
std::optional<anim::SkinnedModel> loadSkinnedModelFbx(const std::filesystem::path& path, std::string& error);

// Smooth per-vertex normals from the triangles (area weighted), for meshes that have none.
void computeNormals(MeshData& mesh);

} // namespace kinetic::assets::detail
