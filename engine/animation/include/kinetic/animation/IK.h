#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/Skeleton.h"

#include <functional>
#include <optional>
#include <span>
#include <vector>

namespace kinetic::anim {

// ---------------------------------------------------------------------------------------------- //
// Two-bone IK (analytic)

// Chain a (root, e.g. hip) -> b (mid, knee) -> c (end, ankle), world positions. Returns the
// world-space rotations to apply to the upper bone (about a) and, after it, to the lower bone
// (about the new b) so that c reaches `target` (or gets as close as the bone lengths allow). The
// bend stays in the plane of the current mid joint (the knee keeps pointing where the animation
// points it); `poleHint` is used only when the chain is straight.
struct TwoBoneSolution {
    glm::quat upper{1.0f, 0.0f, 0.0f, 0.0f};
    glm::quat lower{1.0f, 0.0f, 0.0f, 0.0f};
    bool reached = false;
};
[[nodiscard]] TwoBoneSolution solveTwoBone(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                           const glm::vec3& target, const glm::vec3& poleHint);

// Rotation of a (possibly uniformly scaled) transform matrix.
[[nodiscard]] glm::quat rotationOf(const glm::mat4& m);

// Sets joint `joint`'s rotation so that its world rotation becomes `worldRotation`, given the
// current world matrices of the pose (`world`, e.g. entity * model). Only `local` changes;
// recompute `world` afterwards when children are needed.
void setWorldRotation(const Skeleton& skeleton, Pose& local, std::span<const glm::mat4> world, usize joint,
                      const glm::quat& worldRotation);

// ---------------------------------------------------------------------------------------------- //
// Legs

struct LegChain {
    usize upper = 0; // thigh / up leg
    usize lower = 0; // shin / leg
    usize foot = 0;  // ankle
    i32 toe = -1;
};

// Finds the legs of a humanoid / animal skeleton by name: every joint called *foot* or *ankle*
// (not toe / end / ball / twist / ik / helper) whose parent and grandparent exist, ordered by
// their rest position along X. Works for Mixamo, Unreal (thigh / calf / foot), Blender rigs, ...
[[nodiscard]] std::vector<LegChain> findLegChains(const Skeleton& skeleton);

// The joint all legs hang from (pelvis / hips), or -1.
[[nodiscard]] i32 findPelvis(const Skeleton& skeleton, std::span<const LegChain> legs);

// Ground speed a locomotion clip was authored for: how fast the planted foot slides back
// relative to the pelvis (in place clips) / the pelvis moves over the planted foot (root motion),
// in model units per second. 0 for idle clips. Uses the lowest ~third of each foot's height range
// as "planted".
[[nodiscard]] f32 estimateGroundSpeed(const Skeleton& skeleton, const AnimationClip& clip,
                                      std::span<const LegChain> legs);

// ---------------------------------------------------------------------------------------------- //
// Foot IK: feet follow the ground (slopes, steps, stones) instead of the flat plane the clips
// were authored on.

struct GroundHit {
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
};
// Casts straight down from `from` (world) up to `maxDistance`; nullopt when nothing is hit.
using GroundQuery = std::function<std::optional<GroundHit>(const glm::vec3& from, f32 maxDistance)>;

struct FootIKSettings {
    bool enabled = false;
    f32 probeHeight = 0.5f;   // rays start this far above the character's ground level (m)
    f32 maxStepDown = 0.5f;   // how far below its ground level a foot may reach (m)
    f32 maxPelvisDrop = 0.45f; // the pelvis lowers so the lower foot can reach (m)
    f32 footAlign = 1.0f;     // 0..1: rotate planted feet to the slope
    f32 smoothing = 14.0f;    // 1/s; higher = snappier
};

struct FootIKState {
    std::vector<f32> offsets;          // per leg, smoothed ground height - character ground level
    std::vector<GroundHit> hits;       // per leg, last ground under the foot (debug)
    std::vector<u8> grounded;          // per leg: ground found
    std::vector<glm::vec3> targets;    // per leg, ankle target (world, debug)
    f32 pelvisOffset = 0.0f;
    f32 restAnkleHeight = -1.0f;       // ankle height above the soles in the rest pose (model units)
    bool initialized = false;
};

// Adjusts `local` after animation: lowers the pelvis when a foot must reach lower ground, then
// solves each leg with two-bone IK so its ankle sits at (ground + the clip's own lift), and tilts
// planted feet to the ground normal. `world` is the entity's transform; its translation's Y is
// the character's ground level. dt = 0 snaps without smoothing.
void applyFootIK(const Skeleton& skeleton, std::span<const LegChain> legs, const glm::mat4& world,
                 const GroundQuery& ground, const FootIKSettings& settings, FootIKState& state, f32 dt, Pose& local);

} // namespace kinetic::anim
