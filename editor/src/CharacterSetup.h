#pragma once

#include <kinetic/core/Json.h>
#include <kinetic/gameplay/CharacterController.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/AnimatorComponent.h>
#include <kinetic/scene/Scene.h>

#include <kinetic/animation/Combat.h>

#include <optional>
#include <string>
#include <vector>

namespace kinetic::editor {

// Turns an animated actor (any humanoid with idle / walk / run clips, e.g. a Tripo3D or Mixamo
// export) into a playable hero, using only engine features:
//   - uniform scale so the model is `height` metres tall,
//   - a "Locomotion" blend tree (idle / walk / run on "speed", thresholds measured from the feet),
//   - foot IK on,
//   - a CharacterControllerComponent whose walk / run speeds match the clips (no foot sliding).
// Returns a message for the UI (what was found / what is missing).
struct HeroSetup {
    bool ok = false;
    std::string message;
};
HeroSetup makePlayableHero(scene::Scene& scene, entt::entity entity, f32 height);

// Rebuilds the entity's locomotion tree from its clips at its current scale.
HeroSetup rebuildLocomotionTree(scene::Scene& scene, entt::entity entity);

// Actor settings saved next to the model file (<model>.actor, JSON): the animation graph (with
// each clip's physics bones) and the weapon sockets. "Make Playable Hero" loads it when present;
// the editor saves it whenever something in it changes.
[[nodiscard]] std::string actorFilePath(const anim::SkinnedModel& model); // empty: model not from a file
[[nodiscard]] Json actorToJson(const scene::Scene& scene, entt::entity entity);
bool saveActor(const scene::Scene& scene, entt::entity entity, std::string* message = nullptr);
// Applies the saved graph (if any); returns false when there is no usable file.
bool loadActorGraph(scene::Scene& scene, entt::entity entity, std::string* message = nullptr);
// Applies the saved weapon sockets to the actor's weapons (attachments with a WeaponComponent),
// then the saved clip edits that need them (blade edge alignment).
void loadActorWeapons(scene::Scene& scene, entt::entity entity);

// Turns the weapon hand in clip `clip` so the actor's weapon cuts edge first (anim::alignBladeEdge
// with the weapon's socket). The actor gets its own copy of the model (other instances keep
// theirs). Returns nullopt without a weapon / for an already aligned clip.
std::optional<anim::EdgeAlignment> alignWeaponEdge(scene::Scene& scene, entt::entity entity, i32 clip);

// Model height in its rest pose (model units).
[[nodiscard]] f32 restHeight(const anim::SkinnedModel& model);

// Component settings shown in the Actor Editor (property grids, no headers).
void drawMaterialSection(scene::MeshRendererComponent& mesh, const renderer::Renderer& renderer);
void drawFootIKSettings(scene::Scene& scene, entt::entity entity);
void drawCharacterControllerDetails(scene::Scene& scene, entt::entity entity);

// Foot IK debug: ground hits and ankle targets of an animator.
void appendFootIKLines(const scene::AnimatorComponent& animator, std::vector<renderer::DebugVertex>& out);

} // namespace kinetic::editor
