#include "kinetic/scene/Scene.h"

#include <glm/gtc/matrix_transform.hpp>

namespace kinetic::scene {

glm::mat4 TransformComponent::matrix() const
{
    return glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(rotation) * glm::scale(glm::mat4(1.0f), scale);
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

} // namespace kinetic::scene
