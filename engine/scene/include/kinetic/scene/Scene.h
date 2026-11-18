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

    // Convenience factories: entity + Transform + the matching component.
    entt::entity createCamera(std::string name, const glm::vec3& position, const glm::vec3& target);
    entt::entity createMesh(std::string name, MeshId mesh, const glm::vec4& tint);

    // The camera the scene is rendered from: the first entity with a CameraComponent, or entt::null.
    [[nodiscard]] entt::entity activeCamera() const;

    [[nodiscard]] entt::registry& registry() { return m_registry; }
    [[nodiscard]] const entt::registry& registry() const { return m_registry; }

private:
    entt::registry m_registry;
};

// Rotation that makes an object at `position` look at `target` (local -Z towards the target, +Y up).
[[nodiscard]] glm::quat lookAtRotation(const glm::vec3& position, const glm::vec3& target);

// Euler angles (degrees, glm's XYZ convention: glm::quat(glm::radians(e)) == q) for `q`, choosing
// the representation closest to `hint`. Every orientation has two Euler solutions (plus ±360°
// wraps); glm::eulerAngles keeps Y in [-90, 90] and flips the others, e.g. (0, 100, 0) -> (180, 80, 180).
// Editors pass the previously shown angles as `hint` so the values stay continuous.
[[nodiscard]] glm::vec3 eulerDegreesNear(const glm::quat& q, const glm::vec3& hint);

} // namespace kinetic::scene
