#pragma once

#include "kinetic/scene/Components.h"

#include <kinetic/animation/AnimationPlayer.h>
#include <kinetic/animation/BlendTree.h>
#include <kinetic/animation/IK.h>
#include <kinetic/animation/Skinning.h>

#include <memory>
#include <vector>

namespace kinetic::scene {

// Plays a SkinnedModel's clips on an entity. While it is attached, the entity's
// MeshRendererComponent points at its own (per-instance) mesh, which the animation system
// re-skins every frame; `restMesh` is the shared static mesh restored when animation is turned off.
struct AnimatorComponent {
    std::shared_ptr<const anim::SkinnedModel> model; // shared by every instance of the asset
    MeshId restMesh = kTriangleMesh;
    anim::AnimationPlayer player;
    bool bindPose = false; // true: show the rest pose instead of the animation
    // true: localPose is written by someone else (e.g. a physics simulation) and only skinned;
    // the player is not evaluated.
    bool externalPose = false;

    // Blend tree (e.g. locomotion: idle / walk / run by speed). Used instead of the player while
    // `useBlendTree` is set. Parameters are set by gameplay (CharacterController) or the editor.
    std::shared_ptr<anim::BlendTree> blendTree;
    anim::BlendTreeInstance treeState;
    bool useBlendTree = false;

    // Foot IK after the animation: feet follow the colliders under them.
    anim::FootIKSettings footIK;
    anim::FootIKState footIKState;
    std::vector<anim::LegChain> legs; // found automatically (anim::findLegChains) when empty

    [[nodiscard]] bool blendTreeActive() const { return useBlendTree && blendTree && blendTree->validNode(blendTree->root); }

    // Bones driven by something else after the animation (e.g. bones simulated with muscles,
    // kinetic::physanim). Each override is a model-space joint matrix (relative to
    // `overrideAnchor`'s matrix when that is >= 0); their descendants (fingers, a weapon) follow.
    // `overrideWeight` < 1 blends them with the animation (e.g. getting back up). The owner fills
    // and clears them.
    struct BoneOverride {
        i32 joint = -1;
        glm::mat4 relative{1.0f}; // joint model matrix = (anchor ? jointModel[overrideAnchor] : I) * relative
    };
    i32 overrideAnchor = -1;
    f32 overrideWeight = 1.0f;
    std::vector<BoneOverride> boneOverrides;

    // Results of the last evaluation (written by the animation system, read by UI / debug draw).
    anim::Pose localPose;
    std::vector<glm::mat4> jointModel;    // model-space joint matrices (relative to the entity)
    std::vector<glm::mat4> animatedModel; // the same before boneOverrides (what the animation wants)

    // Scratch memory reused every frame.
    anim::Pose scratchPose;
    std::vector<glm::mat4> skinMatrices;
    std::vector<glm::vec3> skinnedPositions;
    std::vector<glm::vec3> skinnedNormals;
    std::vector<glm::vec4> skinnedTangents;

    [[nodiscard]] std::span<const anim::AnimationClip> clips() const
    {
        return model ? std::span<const anim::AnimationClip>(model->clips) : std::span<const anim::AnimationClip>();
    }
};

} // namespace kinetic::scene
