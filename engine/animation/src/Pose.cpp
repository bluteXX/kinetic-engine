#include "kinetic/animation/Pose.h"

namespace kinetic::anim {

void Pose::resize(usize count)
{
    translations.resize(count, glm::vec3(0.0f));
    rotations.resize(count, glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    scales.resize(count, glm::vec3(1.0f));
}

void Pose::setIdentity()
{
    for (usize i = 0; i < jointCount(); ++i) {
        translations[i] = glm::vec3(0.0f);
        rotations[i] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        scales[i] = glm::vec3(1.0f);
    }
}

JointTransform Pose::get(usize joint) const
{
    return {translations[joint], rotations[joint], scales[joint]};
}

void Pose::set(usize joint, const JointTransform& t)
{
    translations[joint] = t.translation;
    rotations[joint] = t.rotation;
    scales[joint] = t.scale;
}

} // namespace kinetic::anim
