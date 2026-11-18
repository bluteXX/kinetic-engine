#pragma once

#include "kinetic/renderer/Renderer.h"
#include "kinetic/scene/AnimatorComponent.h"
#include "kinetic/scene/Scene.h"

#include <memory>

namespace kinetic::renderer {

// Creates an entity that plays `model`: Transform + MeshRenderer (with its own dynamic mesh,
// since every instance is deformed differently) + Animator, playing the first clip.
// Returns entt::null if the mesh could not be uploaded.
entt::entity createAnimatedEntity(scene::Scene& scene, Renderer& renderer,
                                  std::shared_ptr<const anim::SkinnedModel> model, std::string name,
                                  const glm::vec4& tint);

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

} // namespace kinetic::renderer
