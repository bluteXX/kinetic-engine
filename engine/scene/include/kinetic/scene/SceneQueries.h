#pragma once

#include "kinetic/scene/Scene.h"

#include <kinetic/animation/IK.h>
#include <kinetic/core/Ray.h>

#include <optional>

namespace kinetic::scene {

struct ColliderHit {
    entt::entity entity = entt::null;
    f32 distance = 0.0f; // along the (normalised) ray
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Closest ColliderComponent box (oriented and scaled by the entity's transform) hit by the ray
// within `maxDistance`. `ray.direction` must be normalised. Rays starting inside a box ignore it,
// and so does `ignore` (e.g. the character asking, which may carry a collider of its own).
[[nodiscard]] std::optional<ColliderHit> raycastColliders(const Scene& scene, const Ray& ray, f32 maxDistance,
                                                          entt::entity ignore = entt::null);

// The same against one collider entity only.
[[nodiscard]] std::optional<ColliderHit> raycastCollider(const Scene& scene, entt::entity entity, const Ray& ray, f32 maxDistance);

[[nodiscard]] bool hasColliders(const Scene& scene);

// Ground probe for foot IK / character controllers: a downward ray against the colliders. Scenes
// without any collider get an infinite floor at y = 0 instead. The query keeps a reference to the
// scene. Colliders on `ignore` (the asking character) are skipped.
[[nodiscard]] anim::GroundQuery colliderGroundQuery(const Scene& scene, entt::entity ignore = entt::null);

} // namespace kinetic::scene
