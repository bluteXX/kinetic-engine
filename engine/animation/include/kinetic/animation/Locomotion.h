#pragma once

#include "kinetic/animation/BlendTree.h"
#include "kinetic/animation/IK.h"

#include <optional>
#include <span>
#include <string>
#include <vector>

namespace kinetic::anim {

// A locomotion clip found in a model: which clip, its role and the ground speed it was
// authored for (m/s at `scale`).
struct LocomotionClip {
    i32 clip = -1;
    std::string role; // "idle", "walk", "run"
    f32 speed = 0.0f;
};

// Finds idle / walk / run clips by name (idle, stand, breath / walk / run, jog, sprint; the
// shortest matching name wins) and measures each one's ground speed from its feet, times `scale`
// (the entity's scale, so thresholds are in world m/s).
[[nodiscard]] std::vector<LocomotionClip> findLocomotionClips(const Skeleton& skeleton, std::span<const AnimationClip> clips,
                                                              std::span<const LegChain> legs, f32 scale);

// Builds a locomotion animation graph:
//   parameters  speed (m/s), turn (rad/s, + = left), grounded (bool), jump (trigger)
//   motion      "Locomotion": Blend2D (turn x speed) when the model has in-place turn clips
//               (clips with rootYawRate, see ClipTools::prepareLocomotionClips), else Blend1D on
//               speed; idle at 0, walk / run at their measured ground speeds, turns at their rates
//   states      Locomotion (default) and, with a jump clip, Jump (on the jump trigger; back after
//               80% of the clip once grounded)
// nullopt without a walk or run clip.
[[nodiscard]] std::optional<BlendTree> makeLocomotionTree(const Skeleton& skeleton, std::span<const AnimationClip> clips,
                                                          std::span<const LegChain> legs, f32 scale);

} // namespace kinetic::anim
