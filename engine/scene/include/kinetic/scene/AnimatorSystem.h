#pragma once

#include "kinetic/scene/AnimatorComponent.h"
#include "kinetic/scene/Scene.h"

namespace kinetic::scene {

// Pose stage of animation for every AnimatorComponent (no GPU work, runs headless):
//   1. local pose: bind pose / external (physics) / blend tree / clip player, advanced by dt
//   2. foot IK against the scene's colliders (AnimatorComponent::footIK)
//   3. model-space joint matrices and skinning matrices
// The renderer's animation system calls this, then skins the meshes.
void updateAnimatorPoses(Scene& scene, f32 dt);

// The same in two halves, for simulations that need this frame's animated pose before the final
// one (physics animation): evaluate (steps 1-2 and AnimatorComponent::animatedModel), then - after
// the simulation wrote its bone overrides - finish (overrides, jointModel, skinning matrices).
void evaluateAnimatorPoses(Scene& scene, f32 dt);
void finishAnimatorPoses(Scene& scene);

// Moves every AttachmentComponent to its parent's joint (call after updateAnimatorPoses).
void updateAttachments(Scene& scene);

} // namespace kinetic::scene
