#pragma once

#include "kinetic/animation/Pose.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kinetic::anim {

inline constexpr i32 kNoParent = -1;

// Joint hierarchy. Invariant: parents[i] < i for every joint (parents come first),
// so model-space transforms can be computed in a single forward pass.
class Skeleton {
public:
    Skeleton() = default;
    Skeleton(std::vector<std::string> names, std::vector<i32> parents, Pose restPose,
             std::vector<glm::mat4> inverseBindMatrices = {});

    [[nodiscard]] usize jointCount() const { return m_parents.size(); }
    [[nodiscard]] const std::vector<std::string>& names() const { return m_names; }
    [[nodiscard]] const std::vector<i32>& parents() const { return m_parents; }
    [[nodiscard]] const Pose& restPose() const { return m_restPose; }
    [[nodiscard]] const std::vector<glm::mat4>& inverseBindMatrices() const { return m_inverseBind; }

    [[nodiscard]] std::optional<usize> findJoint(std::string_view name) const;

    // True when the parents-before-children invariant holds and all arrays match.
    [[nodiscard]] bool isValid() const;

private:
    std::vector<std::string> m_names;
    std::vector<i32> m_parents;
    Pose m_restPose;
    std::vector<glm::mat4> m_inverseBind;
};

} // namespace kinetic::anim
