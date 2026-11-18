#include "kinetic/animation/PoseOps.h"

#include "kinetic/core/Assert.h"

#include <glm/gtc/matrix_transform.hpp>

namespace kinetic::anim {

namespace {

glm::quat nlerpShortest(const glm::quat& a, glm::quat b, f32 t)
{
    if (glm::dot(a, b) < 0.0f) {
        b = -b; // q and -q are the same rotation; take the short way round
    }
    return glm::normalize(a * (1.0f - t) + b * t);
}

} // namespace

void blendPoses(const Pose& a, const Pose& b, f32 weight, Pose& out)
{
    KN_ASSERT(a.jointCount() == b.jointCount());
    const usize n = a.jointCount();
    out.resize(n);
    for (usize i = 0; i < n; ++i) {
        out.translations[i] = glm::mix(a.translations[i], b.translations[i], weight);
        out.rotations[i] = nlerpShortest(a.rotations[i], b.rotations[i], weight);
        out.scales[i] = glm::mix(a.scales[i], b.scales[i], weight);
    }
}

void addPose(const Pose& base, const Pose& additive, const Pose& reference, f32 weight, Pose& out)
{
    KN_ASSERT(base.jointCount() == additive.jointCount() && base.jointCount() == reference.jointCount());
    const usize n = base.jointCount();
    out.resize(n);
    const glm::quat identity(1.0f, 0.0f, 0.0f, 0.0f);
    for (usize i = 0; i < n; ++i) {
        const glm::vec3 dt = additive.translations[i] - reference.translations[i];
        const glm::quat dr = additive.rotations[i] * glm::inverse(reference.rotations[i]);
        const glm::vec3 ds = additive.scales[i] / reference.scales[i];

        out.translations[i] = base.translations[i] + dt * weight;
        out.rotations[i] = glm::normalize(nlerpShortest(identity, dr, weight) * base.rotations[i]);
        out.scales[i] = base.scales[i] * glm::mix(glm::vec3(1.0f), ds, weight);
    }
}

glm::mat4 toMatrix(const glm::vec3& t, const glm::quat& r, const glm::vec3& s)
{
    glm::mat4 m = glm::mat4_cast(r);
    m[0] *= s.x;
    m[1] *= s.y;
    m[2] *= s.z;
    m[3] = glm::vec4(t, 1.0f);
    return m;
}

void localToModel(const Skeleton& skeleton, const Pose& local, std::span<glm::mat4> out)
{
    KN_ASSERT(local.jointCount() == skeleton.jointCount() && out.size() == skeleton.jointCount());
    const auto& parents = skeleton.parents();
    for (usize i = 0; i < skeleton.jointCount(); ++i) {
        const glm::mat4 localMatrix = toMatrix(local.translations[i], local.rotations[i], local.scales[i]);
        const i32 p = parents[i];
        out[i] = (p == kNoParent) ? localMatrix : out[static_cast<usize>(p)] * localMatrix;
    }
}

void computeSkinningMatrices(const Skeleton& skeleton, std::span<const glm::mat4> model,
                             std::span<glm::mat4> out)
{
    KN_ASSERT(model.size() == skeleton.jointCount() && out.size() == skeleton.jointCount());
    const auto& inverseBind = skeleton.inverseBindMatrices();
    for (usize i = 0; i < skeleton.jointCount(); ++i) {
        out[i] = model[i] * inverseBind[i];
    }
}

} // namespace kinetic::anim
