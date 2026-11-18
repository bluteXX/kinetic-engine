#include "kinetic/animation/Skeleton.h"

#include "kinetic/core/Assert.h"

namespace kinetic::anim {

Skeleton::Skeleton(std::vector<std::string> names, std::vector<i32> parents, Pose restPose,
                   std::vector<glm::mat4> inverseBindMatrices)
    : m_names(std::move(names))
    , m_parents(std::move(parents))
    , m_restPose(std::move(restPose))
    , m_inverseBind(std::move(inverseBindMatrices))
{
    if (m_inverseBind.empty()) {
        m_inverseBind.assign(m_parents.size(), glm::mat4(1.0f));
    }
    KN_ASSERT(isValid(), "Skeleton violates the parents-before-children invariant");
}

std::optional<usize> Skeleton::findJoint(std::string_view name) const
{
    for (usize i = 0; i < m_names.size(); ++i) {
        if (m_names[i] == name) {
            return i;
        }
    }
    return std::nullopt;
}

bool Skeleton::isValid() const
{
    const usize n = m_parents.size();
    if (m_names.size() != n || m_restPose.jointCount() != n || m_inverseBind.size() != n) {
        return false;
    }
    for (usize i = 0; i < n; ++i) {
        const i32 p = m_parents[i];
        if (p != kNoParent && (p < 0 || static_cast<usize>(p) >= i)) {
            return false;
        }
    }
    return true;
}

} // namespace kinetic::anim
