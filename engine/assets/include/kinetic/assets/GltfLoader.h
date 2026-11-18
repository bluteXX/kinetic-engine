#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"

#include <filesystem>
#include <optional>
#include <vector>

namespace kinetic::assets {

struct SkeletalAnimationAsset {
    anim::Skeleton skeleton;
    std::vector<anim::AnimationClip> clips; // tracks remapped to skeleton joint order
};

// Loads the first skin of a .gltf/.glb file plus every animation that targets its joints.
// Meshes are not loaded yet (milestone 2).
[[nodiscard]] std::optional<SkeletalAnimationAsset> loadSkeletalAnimation(const std::filesystem::path& path);

} // namespace kinetic::assets
