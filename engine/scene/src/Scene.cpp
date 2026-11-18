#include "kinetic/scene/Scene.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace kinetic::scene {

glm::mat4 TransformComponent::matrix() const
{
    return glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(rotation) * glm::scale(glm::mat4(1.0f), scale);
}

glm::mat4 CameraComponent::projection(f32 aspect) const
{
    return glm::perspective(fovY, aspect > 0.0f ? aspect : 1.0f, nearPlane, farPlane);
}

glm::mat4 CameraComponent::view(const TransformComponent& transform)
{
    // Inverse of the camera's rigid transform (translation + rotation, no scale).
    const glm::mat4 rotationInv = glm::mat4_cast(glm::conjugate(transform.rotation));
    return glm::translate(rotationInv, -transform.position);
}

glm::quat lookAtRotation(const glm::vec3& position, const glm::vec3& target)
{
    const glm::vec3 dir = target - position;
    if (glm::dot(dir, dir) < 1e-12f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    // glm::quatLookAtRH returns the orientation whose -Z points along `dir`.
    const glm::vec3 up = std::abs(glm::normalize(dir).y) > 0.999f ? glm::vec3(0.0f, 0.0f, -1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::quatLookAtRH(glm::normalize(dir), up);
}

glm::vec3 eulerDegreesNear(const glm::quat& q, const glm::vec3& hint)
{
    // Shift every angle by a multiple of 360 so it lands as close to the hint as possible.
    const auto wrapNear = [&](glm::vec3 e) {
        for (int i = 0; i < 3; ++i) {
            e[i] += 360.0f * std::round((hint[i] - e[i]) / 360.0f);
        }
        return e;
    };
    const auto represents = [&](const glm::vec3& e) {
        return std::abs(glm::dot(glm::quat(glm::radians(e)), q)) > 0.9999f; // q and -q are the same rotation
    };

    const glm::vec3 a = glm::degrees(glm::eulerAngles(q));
    // The other solution of the XYZ decomposition; both sign variants are checked, only a valid one is kept.
    const glm::vec3 candidates[] = {
        a,
        {a.x + 180.0f, 180.0f - a.y, a.z + 180.0f},
        {a.x + 180.0f, -180.0f - a.y, a.z + 180.0f},
    };

    glm::vec3 best = wrapNear(a);
    f32 bestDistance = glm::dot(best - hint, best - hint);
    for (const glm::vec3& candidate : candidates) {
        const glm::vec3 e = wrapNear(candidate);
        const f32 distance = glm::dot(e - hint, e - hint);
        if (distance < bestDistance && represents(e)) {
            best = e;
            bestDistance = distance;
        }
    }
    return best;
}

entt::entity Scene::createEntity(std::string name)
{
    const entt::entity entity = m_registry.create();
    m_registry.emplace<NameComponent>(entity, std::move(name));
    m_registry.emplace<TransformComponent>(entity);
    return entity;
}

void Scene::destroyEntity(entt::entity entity)
{
    m_registry.destroy(entity);
}

entt::entity Scene::createCamera(std::string name, const glm::vec3& position, const glm::vec3& target)
{
    const entt::entity entity = createEntity(std::move(name));
    auto& transform = m_registry.get<TransformComponent>(entity);
    transform.position = position;
    transform.rotation = lookAtRotation(position, target);
    m_registry.emplace<CameraComponent>(entity);
    return entity;
}

entt::entity Scene::createMesh(std::string name, MeshId mesh, const glm::vec4& tint)
{
    const entt::entity entity = createEntity(std::move(name));
    m_registry.emplace<MeshRendererComponent>(entity, mesh, tint);
    return entity;
}

entt::entity Scene::activeCamera() const
{
    auto view = m_registry.view<const CameraComponent, const TransformComponent>();
    return view.begin() != view.end() ? *view.begin() : entt::null;
}

} // namespace kinetic::scene
