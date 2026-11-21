#pragma once

#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/Scene.h>

#include <vector>

namespace kinetic::editor {

// Wireframes that show an actor's components in a viewport:
//   Box Collider          green box (oriented and scaled like the actor)
//   Character Controller  cyan capsule (radius, height), step-height ring, facing arrow
//   Animator              foot IK ground hits / ankle targets (the skeleton is drawn separately)
//
// `world` places the actor: its TransformComponent matrix in the scene, or wherever a preview
// shows it (the preview shows the model unscaled, so sizes in metres are divided by the actor's
// scale to stay true to the model).
struct GizmoOptions {
    bool collider = true;
    bool controller = true;
    bool footIK = true;
    bool highlight = false; // brighter colours (the actor's component selected in the Actor Editor)
};
void appendComponentGizmos(const scene::Scene& scene, entt::entity actor, const glm::mat4& world,
                           std::vector<renderer::DebugVertex>& out, const GizmoOptions& options = {});

// Box Collider wireframes of every actor (View > All Colliders).
void appendAllColliders(const scene::Scene& scene, std::vector<renderer::DebugVertex>& out);

} // namespace kinetic::editor
