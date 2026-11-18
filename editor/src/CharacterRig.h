#pragma once

#include <kinetic/animation/Skinning.h>
#include <kinetic/physics/PhysicsWorld.h>

#include <memory>
#include <vector>

namespace kinetic::editor::character {

// The playable character: a 1.8 m, 75 kg humanoid made of capsules and boxes, holding a sword in
// its right hand. The same definition builds
//   - the SkinnedModel (rigidly skinned: every vertex follows one bone) with its clips, and
//   - the physics ragdoll: one body per bone, joints with limits, muscles on the sword arm.
// Model space: feet on y = 0, facing +Z, the character's right is -X.

enum Joint : u32 {
    kRoot,
    kPelvis,
    kChest,
    kHead,
    kUpperArmR,
    kForearmR,
    kHandR,
    kSword,
    kUpperArmL,
    kForearmL,
    kHandL,
    kThighR,
    kShinR,
    kFootR,
    kThighL,
    kShinL,
    kFootL,
    kJointCount,
};

enum Clip : i32 { kIdle = 0, kWalk = 1, kStrike = 2 };

enum class JointType : u8 { None, Hinge, SwingTwist, Fixed };

struct Part {
    const char* name;
    i32 parent;
    glm::vec3 restPosition; // joint position, model space (rest pose: arms and sword hanging down)
    physics::ShapeDesc shape; // relative to the joint (body origin = joint, rest rotation = identity)
    f32 mass = 0.0f;        // kg (0 = no body: the root)
    glm::vec3 centerOfMass{0.0f};
    JointType joint = JointType::None; // to the parent
    // Hinges turn about the model's -X axis (positive = forward, "sagittal"); limits in degrees.
    f32 limitMin = 0.0f;
    f32 limitMax = 0.0f;
    // Swing-twist: cone half angles and twist range, degrees.
    f32 cone = 0.0f;
    f32 twist = 0.0f;
    physics::Muscle muscle{}; // hinges of the sword arm
    bool swordArm = false;    // simulated with muscles during physics clips
};

// Index = skeleton joint (see Joint).
[[nodiscard]] const std::vector<Part>& parts();

// Model (mesh + skeleton + clips Idle, Walk [kinematic], Strike [physics]).
[[nodiscard]] std::shared_ptr<anim::SkinnedModel> buildModel();

// Rotation about the sagittal axis (-X): positive swings a hanging limb forward.
[[nodiscard]] glm::quat sagittal(f32 degrees);
// Signed angle of `q` about the sagittal axis, radians.
[[nodiscard]] f32 sagittalAngle(const glm::quat& q);

inline constexpr glm::vec3 kSagittalAxis{-1.0f, 0.0f, 0.0f};
inline constexpr f32 kWalkClipSpeed = 1.6f; // m/s at playback speed 1

} // namespace kinetic::editor::character
