#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"

#include <string>
#include <vector>

namespace kinetic::anim {

// Animation retargeting: plays a clip made for one skeleton on another with the same bones (by
// name) but different proportions and bone axes - e.g. a Mixamo download on a Tripo3D / Mixamo
// auto-rigged character, or clips shared between characters.
//
// Both skeletons' rest poses must be the same kind of pose (both T-pose, or both A-pose) and face
// the same way. Per mapped bone, the clip's rotation change from the source rest pose (in model
// space) is applied to the target's rest pose, so differing local bone axes do not matter. The
// root (hips) translation is scaled by the ratio of the hip heights.

// Target joint -> source joint (-1 = not animated). Names match exactly, or after removing a
// namespace prefix ("mixamorig:Hips", "mixamorig1:Hips", "Armature|Hips" -> "hips"), ignoring case.
[[nodiscard]] std::vector<i32> mapJointsByName(const Skeleton& source, const Skeleton& target);

struct RetargetOptions {
    f32 sampleRate = 30.0f;   // keys per second of the result
    bool translateRoot = true; // move the hips like the source (scaled); false keeps them at rest
};

[[nodiscard]] AnimationClip retargetClip(const Skeleton& source, const AnimationClip& clip, const Skeleton& target,
                                         std::string name, const RetargetOptions& options = {});

} // namespace kinetic::anim
