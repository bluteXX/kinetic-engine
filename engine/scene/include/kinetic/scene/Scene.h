#pragma once

#include "kinetic/scene/Components.h"

#include <entt/entity/registry.hpp>

#include <string>

namespace kinetic::scene {

// Thin owner of the ECS registry. Systems (animation, rendering) iterate views over it.
class Scene {
public:
    entt::entity createEntity(std::string name);
    void destroyEntity(entt::entity entity);

    [[nodiscard]] entt::registry& registry() { return m_registry; }
    [[nodiscard]] const entt::registry& registry() const { return m_registry; }

private:
    entt::registry m_registry;
};

} // namespace kinetic::scene
