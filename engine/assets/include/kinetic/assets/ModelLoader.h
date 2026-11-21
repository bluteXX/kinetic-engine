#pragma once

#include "kinetic/animation/Skinning.h"
#include "kinetic/assets/GltfLoader.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

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

// Skeleton + animation clips of a file, mesh or not (e.g. Mixamo "without skin" FBX downloads).
// Clips are named after the file when it holds a single one with a generic take name. Use
// anim::retargetClip to play them on another model's skeleton.
[[nodiscard]] std::optional<SkeletalAnimationAsset> loadAnimationFile(const std::filesystem::path& path,
                                                                      std::string* error = nullptr);

// Loads the animation file and adds its clips to `model`, retargeted onto the model's skeleton
// (anim::retargetClip: bones matched by name, rest poses in the same stance). Returns the names of
// the clips added (empty on failure, with `error`).
std::vector<std::string> importAnimationsInto(anim::SkinnedModel& model, const std::filesystem::path& file,
                                              std::string* error = nullptr);

// .glb, .gltf, .fbx (lower or upper case).
[[nodiscard]] bool isSupportedModelFile(const std::filesystem::path& path);

} // namespace kinetic::assets
