#pragma once

#include "kinetic/core/Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <memory>
#include <span>
#include <string>
#include <vector>

namespace kinetic::physics {

// Thin wrapper around Jolt Physics (kept out of the public headers).
//
// Built for measuring how forces travel through articulated bodies (e.g. a sword arm hitting a
// stone): every joint reports the forces it transmits and the torque its "muscle" (a position
// motor with a torque limit) produces, and contact forces are reconstructed exactly from the
// change of momentum of the bodies. Use one collision step per step() and a small fixed dt
// (240 Hz or more) for accurate per-step forces.

struct BodyId {
    u32 index = ~0u;
    [[nodiscard]] bool valid() const { return index != ~0u; }
    friend bool operator==(BodyId a, BodyId b) { return a.index == b.index; }
};

struct JointId {
    u32 index = ~0u;
    [[nodiscard]] bool valid() const { return index != ~0u; }
};

enum class MotionType : u8 { Static, Kinematic, Dynamic };
enum class ShapeType : u8 { Box, Sphere, Capsule };

// Collision shape in body space. Capsules are aligned with the local Y axis.
struct ShapeDesc {
    ShapeType type = ShapeType::Box;
    glm::vec3 halfExtents{0.5f}; // box
    f32 radius = 0.5f;           // sphere, capsule
    f32 halfHeight = 0.5f;       // capsule: half length of the cylinder part
    glm::vec3 offset{0.0f};      // shape centre relative to the body origin
};

struct BodyDesc {
    std::string name;
    ShapeDesc shape;
    glm::vec3 position{0.0f}; // body origin, world space
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    MotionType motion = MotionType::Dynamic;
    f32 mass = 0.0f;                  // kg; 0 = from shape volume with density 1000
    glm::vec3 centerOfMass{0.0f};     // local offset from the shape centre (e.g. a sword's balance point)
    f32 friction = 0.6f;
    f32 restitution = 0.0f;
    f32 linearDamping = 0.0f;         // 0 keeps force reconstruction exact
    f32 angularDamping = 0.0f;
    bool continuousCollision = false; // CCD for fast, thin bodies (blades)
    // Bodies of one rig: same non-zero group = never collide with each other.
    u32 collisionGroup = 0;
};

// Ball-and-socket joint with a cone and twist limits (hips, shoulders, spine, neck).
// The twist axis runs along the child bone; the cone opens around it.
struct SwingTwistDesc {
    std::string name;
    BodyId parent;
    BodyId child;
    glm::vec3 anchor{0.0f};                     // world space
    glm::vec3 twistAxis{0.0f, -1.0f, 0.0f};     // world space, along the child bone
    glm::vec3 planeAxis{1.0f, 0.0f, 0.0f};      // world space, perpendicular to twistAxis
    f32 normalHalfCone = 0.8f;                  // rad
    f32 planeHalfCone = 0.8f;                   // rad
    f32 twistMin = -0.5f;                       // rad
    f32 twistMax = 0.5f;                        // rad
    f32 friction = 0.0f;                        // N m, joint friction (limp ragdolls look less floppy)
};

// Muscle model v1: position motor with a spring and a torque limit.
//   torque = clamp(stiffness * (target - angle) - damping * angularVelocity, -maxTorque, maxTorque)
struct Muscle {
    f32 stiffness = 300.0f; // N m / rad
    f32 damping = 20.0f;    // N m s / rad
    f32 maxTorque = 50.0f;  // N m (muscle strength)
    bool enabled = true;
};

// One-axis joint (elbow-like). The angle is the child's rotation about `axis` relative to the
// parent, measured so that 0 is the pose at creation in which `referenceParent` (a direction
// fixed to the parent) and `referenceChild` (fixed to the child) coincide.
struct HingeDesc {
    std::string name;
    BodyId parent;
    BodyId child;
    glm::vec3 anchor{0.0f};          // world space
    glm::vec3 axis{0.0f, 0.0f, 1.0f}; // world space
    glm::vec3 referenceParent{0.0f, -1.0f, 0.0f}; // world space, perpendicular to axis
    glm::vec3 referenceChild{0.0f, -1.0f, 0.0f};
    f32 limitMin = -glm::pi<f32>(); // rad, in [-pi, 0]
    f32 limitMax = glm::pi<f32>();  // rad, in [0, pi]
    Muscle muscle;
    f32 friction = 0.0f; // N m, joint friction while the muscle is off
};

// Rigid weld (a hand gripping a sword).
struct FixedDesc {
    std::string name;
    BodyId parent;
    BodyId child;
    glm::vec3 anchor{0.0f}; // world space
};

struct BodyState {
    glm::vec3 position{0.0f}; // body origin
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 centerOfMass{0.0f}; // world
    glm::vec3 linearVelocity{0.0f};
    glm::vec3 angularVelocity{0.0f};
    f32 mass = 0.0f;
    // Sum of contact forces on this body during the last step and their mean application point,
    // reconstructed from the momentum balance (exact when damping is 0).
    glm::vec3 contactForce{0.0f};
    glm::vec3 contactPoint{0.0f};
    bool inContact = false;
};

struct JointState {
    bool hinge = true;
    glm::vec3 anchor{0.0f}; // world
    glm::vec3 axis{0.0f};   // world (hinges)
    f32 angle = 0.0f;       // rad (hinges)
    f32 targetAngle = 0.0f;
    f32 angularVelocity = 0.0f; // child relative to parent about the axis, rad/s
    glm::vec3 force{0.0f};      // N: force the parent applies to the child at the anchor
    glm::vec3 torque{0.0f};     // N m: rotational constraint torque on the child (fixed joints)
    f32 muscleTorque = 0.0f;    // N m about the axis, on the child
    f32 limitTorque = 0.0f;     // N m about the axis from the joint limits
};

struct ContactPoint {
    BodyId bodyA;
    BodyId bodyB;
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f}; // from A towards B
    f32 depth = 0.0f;
};

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    bool init(u32 maxBodies = 1024);
    void shutdown();
    // Removes every body and joint.
    void clear();

    BodyId addBody(const BodyDesc& desc);
    JointId addHinge(const HingeDesc& desc);
    JointId addFixed(const FixedDesc& desc);
    JointId addSwingTwist(const SwingTwistDesc& desc);

    // Kinematic / dynamic switching (static bodies stay static).
    void setMotionType(BodyId body, MotionType motion);
    // Kinematic bodies: reach this transform at the end of the next step of length dt (gives the
    // body the velocity needed, so it pushes dynamic bodies correctly).
    void moveKinematic(BodyId body, const glm::vec3& position, const glm::quat& rotation, f32 dt);
    // Instantly place a body (no velocity), e.g. when a ragdoll is reset.
    void teleport(BodyId body, const glm::vec3& position, const glm::quat& rotation);

    void setMuscle(JointId joint, const Muscle& muscle);
    // Contact material. Jolt combines two bodies' restitution with max() and friction with sqrt(a b).
    void setRestitution(BodyId body, f32 restitution);
    void setFriction(BodyId body, f32 friction);
    void setTargetAngle(JointId joint, f32 angle);
    void setGravity(const glm::vec3& gravity);
    [[nodiscard]] glm::vec3 gravity() const;

    // One simulation step of dt seconds (one collision step), then refreshes every state.
    void step(f32 dt);

    [[nodiscard]] usize bodyCount() const;
    [[nodiscard]] usize jointCount() const;
    [[nodiscard]] const BodyDesc& bodyDesc(BodyId body) const;
    [[nodiscard]] const BodyState& body(BodyId body) const;
    [[nodiscard]] const std::string& jointName(JointId joint) const;
    [[nodiscard]] const Muscle& muscle(JointId joint) const;
    [[nodiscard]] const JointState& joint(JointId joint) const;
    [[nodiscard]] std::span<const ContactPoint> contacts() const;
    [[nodiscard]] f32 lastStep() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace kinetic::physics
