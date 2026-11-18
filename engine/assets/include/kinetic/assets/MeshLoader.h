#pragma once

#include "kinetic/core/MeshData.h"

#include <filesystem>
#include <optional>
#include <string>

namespace kinetic::assets {

// Loads every mesh in an .fbx or .obj file (via ufbx) and merges them into one MeshData.
// Geometry is converted to the engine's space: right-handed, +Y up, meters; node transforms are
// baked in; polygons are triangulated; missing normals are generated.
// On failure returns std::nullopt and, if `error` is given, a human-readable reason.
[[nodiscard]] std::optional<MeshData> loadMesh(const std::filesystem::path& path, std::string* error = nullptr);

// File extensions loadMesh understands (lower case, without the dot).
[[nodiscard]] bool isSupportedMeshFile(const std::filesystem::path& path);

} // namespace kinetic::assets
