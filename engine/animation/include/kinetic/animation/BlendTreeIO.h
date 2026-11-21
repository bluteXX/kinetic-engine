#pragma once

#include "kinetic/animation/BlendTree.h"

#include <kinetic/core/Json.h>

#include <optional>
#include <span>
#include <string>

namespace kinetic::anim {

// Animation graph <-> JSON. Clips are stored by name and joints (physics bones) by name, so a
// saved graph survives re-importing the model or adding clips; references that no longer resolve
// are dropped (listed in `warnings`).
[[nodiscard]] Json blendTreeToJson(const BlendTree& tree, const Skeleton& skeleton, std::span<const AnimationClip> clips);
[[nodiscard]] std::optional<BlendTree> blendTreeFromJson(const Json& json, const Skeleton& skeleton,
                                                         std::span<const AnimationClip> clips, std::string* warnings = nullptr);

} // namespace kinetic::anim
