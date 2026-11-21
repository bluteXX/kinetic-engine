#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"
#include "kinetic/core/MeshData.h"

#include <span>
#include <string>
#include <vector>

namespace kinetic::anim {

// Joint influences per vertex (linear blend skinning).
inline constexpr usize kMaxInfluences = 4;

// A mesh in its bind pose plus, per vertex, up to four joints and their weights.
// Joint indices refer to the owning SkinnedModel's skeleton; weights sum to 1.
struct SkinnedMesh {
    MeshData mesh;
    std::vector<glm::uvec4> joints;
    std::vector<glm::vec4> weights;

    [[nodiscard]] bool isValid(usize jointCount) const;
};

// Everything needed to play an animated character: skinned mesh, skeleton and its clips.
struct SkinnedModel {
    std::string name;
    std::string sourcePath; // file it was loaded from (actor settings are saved next to it)
    SkinnedMesh skin;
    Skeleton skeleton;
    std::vector<AnimationClip> clips;
    // Per joint: true for real bones, false for helper nodes (FBX scene root, armature object).
    // Only bones are drawn in skeleton overlays. Empty = every joint is a bone.
    std::vector<u8> isBone;
    // Bounds of the mesh skinned in the skeleton's rest pose (see computeRestBounds). This is
    // where the model actually appears; the bind mesh itself may be in another space (e.g. Z-up
    // glTF meshes under a rotated parent node).
    glm::vec3 restBoundsMin{0.0f};
    glm::vec3 restBoundsMax{0.0f};
    // Clips were processed by anim::prepareLocomotionClips (centred roots, loops, in-place turns).
    bool locomotionPrepared = false;

    [[nodiscard]] bool jointIsBone(usize joint) const { return isBone.empty() || isBone[joint] != 0; }
};

// Scales `weights` so they sum to 1. All-zero weights stay zero.
[[nodiscard]] glm::vec4 normalizeWeights(const glm::vec4& weights);

// CPU linear blend skinning: out = sum(weight_i * skinMatrix[joint_i]) * bindVertex.
// Normals use the upper 3x3 of the blended matrix and are renormalized (assumes no strong
// non-uniform scale in the joints). Tangents (if the mesh has them and `outTangents` is not
// empty) are rotated the same way and keep their handedness (w). Output spans must have one
// entry per mesh vertex.
void skinVertices(const SkinnedMesh& skin, std::span<const glm::mat4> skinMatrices, std::span<glm::vec3> outPositions,
                  std::span<glm::vec3> outNormals, std::span<glm::vec4> outTangents = {});

// The model's mesh skinned in the skeleton's rest pose: what a static (non-animated) instance of
// the model shows.
[[nodiscard]] MeshData skinnedRestMesh(const SkinnedModel& model);

// Skins the mesh in the rest pose and stores its bounds in model.restBoundsMin/Max.
void computeRestBounds(SkinnedModel& model);

} // namespace kinetic::anim
