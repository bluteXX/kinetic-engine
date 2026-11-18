#pragma once

#include "kinetic/renderer/Renderer.h"
#include "kinetic/scene/AnimatorComponent.h"
#include "kinetic/scene/Scene.h"

#include <memory>

namespace kinetic::renderer {

// Makes an entity with a MeshRendererComponent play `model`: gives it its own dynamic mesh (every
// instance is deformed differently) and an AnimatorComponent playing the first clip. The current
// mesh id is kept as AnimatorComponent::restMesh. Returns false if the mesh could not be uploaded.
bool enableAnimation(scene::Scene& scene, Renderer& renderer, entt::entity entity,
                     std::shared_ptr<const anim::SkinnedModel> model);

// Removes the AnimatorComponent, releases the per-instance mesh and restores the rest mesh.
void disableAnimation(scene::Scene& scene, Renderer& renderer, entt::entity entity);

// Removes an entity's MeshRendererComponent (and animation, which needs it).
void removeMesh(scene::Scene& scene, Renderer& renderer, entt::entity entity);

// Destroys an entity and, for animated entities, releases their per-instance mesh.
void destroyEntity(scene::Scene& scene, Renderer& renderer, entt::entity entity);

// For every animated entity: advances its player by dt, evaluates the pose, skins the mesh on
// the CPU and stages the new vertices for upload. Call once per frame before rendering.
//
// CPU skinning keeps the existing mesh shader working; GPU skinning (joint matrices in a storage
// buffer, skinned in the vertex shader) is the planned replacement for heavier characters.
void updateAnimations(scene::Scene& scene, Renderer& renderer, f32 dt);

// Bones of every animated entity as world-space line segments (for Renderer::setOverlayLines).
// `highlightEntity` / `highlightJoint` select one joint drawn in a different colour.
void appendSkeletonLines(const scene::Scene& scene, std::vector<DebugVertex>& out,
                         entt::entity highlightEntity = entt::null, i32 highlightJoint = -1);

// Bones of one animator placed with `world` (e.g. in a preview scene).
void appendSkeletonLines(const scene::AnimatorComponent& animator, const glm::mat4& world,
                         std::vector<DebugVertex>& out, i32 highlightJoint = -1);

} // namespace kinetic::renderer
