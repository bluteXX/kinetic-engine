#pragma once

#include "kinetic/animation/Skinning.h"

#include <filesystem>
#include <optional>
#include <string>

namespace kinetic::assets {

// Loads an animated model: skinned mesh (bind pose), skeleton and every animation clip.
//
//   .glb / .gltf — the first skin, every mesh bound to it, and all animations targeting its joints.
//   .fbx         — every mesh; skinned meshes keep their bone weights, other meshes are bound
//                  rigidly to their own node. All animation stacks are baked to linear keys.
//
// Output space matches loadMesh: right-handed, +Y up, meters. Fails (std::nullopt + `error`)
// for files with neither a skin nor animation — use loadMesh for static meshes.
[[nodiscard]] std::optional<anim::SkinnedModel> loadSkinnedModel(const std::filesystem::path& path,
                                                                 std::string* error = nullptr);

// .glb, .gltf, .fbx (lower or upper case).
[[nodiscard]] bool isSupportedModelFile(const std::filesystem::path& path);

} // namespace kinetic::assets
