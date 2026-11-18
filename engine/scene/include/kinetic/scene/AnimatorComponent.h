#pragma once

#include <kinetic/animation/AnimationPlayer.h>
#include <kinetic/animation/Skinning.h>

#include <memory>
#include <vector>

namespace kinetic::scene {

// Plays a SkinnedModel's clips on an entity. The entity's MeshRendererComponent points at its own
// (per-instance) mesh, which the animation system re-skins every frame.
struct AnimatorComponent {
    std::shared_ptr<const anim::SkinnedModel> model; // shared by every instance of the asset
    anim::AnimationPlayer player;
    bool bindPose = false; // true: show the rest pose instead of the animation

    // Results of the last evaluation (written by the animation system, read by UI / debug draw).
    anim::Pose localPose;
    std::vector<glm::mat4> jointModel; // model-space joint matrices (relative to the entity)

    // Scratch memory reused every frame.
    anim::Pose scratchPose;
    std::vector<glm::mat4> skinMatrices;
    std::vector<glm::vec3> skinnedPositions;
    std::vector<glm::vec3> skinnedNormals;

    [[nodiscard]] std::span<const anim::AnimationClip> clips() const
    {
        return model ? std::span<const anim::AnimationClip>(model->clips) : std::span<const anim::AnimationClip>();
    }
};

} // namespace kinetic::scene
