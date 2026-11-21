#pragma once

#include "kinetic/animation/AnimationClip.h"
#include "kinetic/animation/IK.h"
#include "kinetic/animation/Skinning.h"

#include <optional>
#include <string>
#include <vector>

namespace kinetic::anim {

// Clip processing, the kind of thing an importer's "animation settings" do (Unity: Loop Pose,
// Root Transform Position / Rotation "Bake Into Pose", clip start / end, Mirror). All of them
// rewrite the clip's keys; sampling stays as cheap as before.

// Moves `joint` (usually the pelvis) so its average horizontal (model XZ) position over the clip
// equals its rest position. Fixes "in place" clips authored off-centre, which otherwise make the
// body slide while blending with other clips. Returns the shift applied (model units).
f32 centerRootXZ(const Skeleton& skeleton, AnimationClip& clip, usize joint);

// Loop Pose: spreads the difference between the last and the first frame over the clip, so the
// loop closes without a pop. Returns the largest joint gap closed (model units).
f32 makeLoopPose(const Skeleton& skeleton, AnimationClip& clip);

// The part of `clip` between `start` and `end` seconds as a new clip starting at 0.
[[nodiscard]] AnimationClip makeSubClip(const AnimationClip& clip, f32 start, f32 end, std::string name);

// Removes the turning of `joint` (pelvis) about the vertical axis from the clip ("bake root
// rotation into motion"): the body keeps facing its first-frame direction; the caller rotates the
// character instead. Returns the average yaw rate removed (rad/s, positive = counter-clockwise
// seen from above, i.e. turning left for a character facing +Z).
f32 extractRootYaw(const Skeleton& skeleton, AnimationClip& clip, usize joint);

// Removes the horizontal travel of `joint` (pelvis) from the clip ("bake root position into
// motion"): the hips stay over the character's origin and the travel is stored as the clip's root
// motion (AnimationClip::rootMotion), which the character controller applies. Returns the
// distance travelled from start to end (model units).
f32 extractRootMotion(const Skeleton& skeleton, AnimationClip& clip, usize joint);

// Yaw of `joint` over time (model space, radians, unwrapped), sampled `samples` + 1 times.
[[nodiscard]] std::vector<f32> rootYawCurve(const Skeleton& skeleton, const AnimationClip& clip, usize joint,
                                            usize samples = 60);

// Time range where the root turns (yaw rate above 25% of its peak), or nullopt for clips that do
// not turn at least `minAngle` radians.
[[nodiscard]] std::optional<std::pair<f32, f32>> findTurnSegment(const Skeleton& skeleton, const AnimationClip& clip,
                                                                 usize joint, f32 minAngle = 0.8f);

// Left / right counterpart of every joint by name (Left/Right, left/right, L/R, _l/_r, .L/.R);
// joints without one map to themselves.
[[nodiscard]] std::vector<usize> findMirrorJoints(const Skeleton& skeleton);

// Mirrored copy of a clip (across the model's YZ plane: left and right swap). Uses the rest pose
// to map rotations between the two sides, so it works with asymmetric bone axes (Mixamo, ...).
[[nodiscard]] AnimationClip mirrorClip(const Skeleton& skeleton, const AnimationClip& clip, std::string name);

// One-call preparation of a model's locomotion clips (what "Make Playable Hero" runs):
//   idle / walk / run   centred root, loop pose
//   turn (if any)       the turning part cut out, root yaw extracted, loop pose, plus a mirrored
//                       copy - appended as "<turn> (in place)" and "<turn> (mirror)"
// Returns a line per change for the UI / log.
struct LocomotionPrepReport {
    std::vector<std::string> lines;
    i32 turnClip = -1;        // index of the in-place turn clip, -1 if none
    i32 turnMirrorClip = -1;  // its mirrored copy
    f32 turnRate = 0.0f;      // rad/s of turnClip (sign: see extractRootYaw)
};
LocomotionPrepReport prepareLocomotionClips(SkinnedModel& model);

} // namespace kinetic::anim
