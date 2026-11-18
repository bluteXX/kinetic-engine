#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

namespace kinetic::anim {

// Local-space joint transform (relative to the parent joint).
struct JointTransform {
    glm::vec3 translation{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
    glm::vec3 scale{1.0f};
};

// A pose stored as Structure-of-Arrays: one array per channel, indexed by joint.
// Blending, sampling and IK walk these arrays linearly, which is cache/SIMD friendly.
struct Pose {
    std::vector<glm::vec3> translations;
    std::vector<glm::quat> rotations;
    std::vector<glm::vec3> scales;

    Pose() = default;
    explicit Pose(usize count) { resize(count); }

    void resize(usize count);
    void setIdentity();

    [[nodiscard]] usize jointCount() const { return rotations.size(); }

    [[nodiscard]] JointTransform get(usize joint) const;
    void set(usize joint, const JointTransform& t);
};

} // namespace kinetic::anim
