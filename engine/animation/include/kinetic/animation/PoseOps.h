#pragma once

#include "kinetic/animation/Pose.h"
#include "kinetic/animation/Skeleton.h"

#include <span>

namespace kinetic::anim {

// out = lerp(a, b, weight) per joint. Rotations use normalized lerp along the shortest arc.
// `out` may alias `a` or `b`.
void blendPoses(const Pose& a, const Pose& b, f32 weight, Pose& out);

// Additive layer: out = base + (additive - reference) * weight.
void addPose(const Pose& base, const Pose& additive, const Pose& reference, f32 weight, Pose& out);

// Local (parent-relative) pose -> model-space matrices. `out.size()` must equal jointCount.
void localToModel(const Skeleton& skeleton, const Pose& local, std::span<glm::mat4> out);

// Final matrices sent to the GPU: model * inverseBind.
void computeSkinningMatrices(const Skeleton& skeleton, std::span<const glm::mat4> model,
                             std::span<glm::mat4> out);

[[nodiscard]] glm::mat4 toMatrix(const glm::vec3& t, const glm::quat& r, const glm::vec3& s);

} // namespace kinetic::anim
