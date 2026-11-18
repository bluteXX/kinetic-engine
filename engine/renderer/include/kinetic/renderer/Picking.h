#pragma once

#include "kinetic/core/Ray.h"
#include "kinetic/renderer/Renderer.h"
#include "kinetic/scene/Scene.h"

#include <optional>

namespace kinetic::renderer {

struct PickHit {
    entt::entity entity = entt::null;
    f32 distance = 0.0f; // along the ray, in ray units
};

// Ray through a point of the camera's image. `ndc` is in [-1, 1] (x right, y up).
[[nodiscard]] Ray cameraRay(const scene::CameraComponent& camera, const scene::TransformComponent& transform,
                            f32 aspect, const glm::vec2& ndc);

// Closest visible mesh the ray hits, tested against its triangles. Animated entities are tested
// in their current pose (the AnimatorComponent's skinned vertices).
[[nodiscard]] std::optional<PickHit> pickEntity(const scene::Scene& scene, const Renderer& renderer, const Ray& ray);

// World-space axis-aligned bounds of an entity's mesh as currently shown (posed for animated
// entities). False if the entity has no visible mesh.
bool worldBounds(const scene::Scene& scene, const Renderer& renderer, entt::entity entity, glm::vec3& outMin,
                 glm::vec3& outMax);

// The 12 edges of a box as line segments.
void appendBoxLines(std::vector<DebugVertex>& out, const glm::vec3& boxMin, const glm::vec3& boxMax,
                    const glm::vec3& color);

} // namespace kinetic::renderer
