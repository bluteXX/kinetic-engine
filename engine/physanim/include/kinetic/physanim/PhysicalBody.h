#pragma once

#include "kinetic/physanim/HitTracker.h"

#include <kinetic/animation/Skinning.h>
#include <kinetic/physics/PhysicsWorld.h>
#include <kinetic/scene/AnimatorComponent.h>

#include <memory>
#include <span>
#include <string>
#include <vector>

namespace kinetic::physanim {

// --- Body layout --------------------------------------------------------------------------------
//
// The rigid bodies a skinned model gets for physics animation, found from any skeleton: one
// capsule per bone long enough to matter (spine pieces, limbs, hands, feet, head); shorter bones
// (fingers, toes, end bones) ride on the body above them. Each capsule runs from its joint to the
// next joint along the chain (stretched over its riders, e.g. a hand over its fingers) and is as
// thick as the skin around it (median distance of the vertices bound to it).
struct BodySegment {
    i32 joint = -1;
    i32 parent = -1;          // parent segment, -1 for the root (the pelvis)
    glm::vec3 start{0.0f};    // model space, rest pose
    glm::vec3 end{0.0f};
    f32 radius = 0.05f;       // model units
    std::vector<i32> riders;  // joints carried by this body (not simulated on their own)
};

struct BodyLayout {
    std::vector<BodySegment> segments; // parents first
    std::vector<i32> segmentOfJoint;   // per joint: its body (own or the one it rides on), -1 = none
    f32 height = 1.0f;                 // model units

    [[nodiscard]] i32 segmentOf(i32 joint) const
    {
        return joint >= 0 && static_cast<usize>(joint) < segmentOfJoint.size() ? segmentOfJoint[static_cast<usize>(joint)] : -1;
    }
};

// `minSegment`: bones shorter than this fraction of the model's height ride on their parent.
[[nodiscard]] BodyLayout layoutBody(const anim::SkinnedModel& model, f32 minSegment = 0.04f);

// Presets for the "physics bones" of a clip (joint indices; see anim::BlendNode::physicsJoints).
[[nodiscard]] std::vector<i32> weaponArmJoints(const BodyLayout& layout, i32 hand); // hand, forearm, upper arm
[[nodiscard]] std::vector<i32> fullBodyJoints(const BodyLayout& layout);

// --- Simulation -----------------------------------------------------------------------------------

struct BodySettings {
    f32 bodyMass = 75.0f;   // kg, spread over the bodies by volume (0: use `density`)
    f32 density = 1050.0f;  // kg / m^3
    // Muscles, per joint from the inertia I of everything it moves (kg m^2): stiffness = I w^2,
    // damping = 2 zeta I w, strength = `strength` * sqrt(I). w = 2 pi frequency, higher for light
    // limbs (up to 2.5x for a hand) so they keep up with fast swings.
    f32 frequency = 8.0f;   // Hz
    f32 dampingRatio = 1.0f;
    f32 strength = 250.0f;  // N m per sqrt(kg m^2): about 110 N m at a sword hand, 300 at the shoulder
    // The muscles aim this far ahead along the animation (s): a spring lags behind a moving target.
    f32 lead = 0.03f;
    // A hard hit (rebound / stop) or a stuck blade: the weapon arm yields for `reflexTime` (at
    // reflexStrength), then pulls back onto the animation over `recoverTime`, starting `hold`
    // of the way there.
    f32 reflexTime = 0.06f;
    f32 reflexStrength = 0.25f;
    f32 recoverTime = 0.35f;
    f32 hold = 0.3f;
    // A bone leaving the physics bones keeps tracking the animation this long, then turns
    // kinematic (no pop).
    f32 releaseTime = 0.2f;
    // Full body (the pelvis simulated): a balance assist pulls the pelvis towards the animation,
    // with limited force. When something holds the body back too far (a wall, a stone) it gives
    // up: the character is knocked down (ragdoll), lies for `downTime`, then gets back up.
    f32 assistFrequency = 6.0f;      // Hz
    f32 assistMaxAcceleration = 25.0f; // m/s^2 (on top of carrying the weight)
    f32 knockdownDistance = 0.16f;   // fraction of the height between pelvis and its animated place
    f32 knockdownAngle = 0.9f;       // rad of the pelvis off its animated orientation
    f32 ragdollStrength = 0.06f;     // muscles while down
    f32 downTime = 1.5f;             // s
    f32 getUpTime = 0.6f;            // s blending from the lying pose back to the animation
};

// A weapon held by the model: welded to the body of `joint` at `offset` (AttachmentComponent).
struct WeaponMount {
    i32 joint = -1;
    glm::mat4 offset{1.0f};
    scene::WeaponComponent weapon;
};

// A character's physics body for physics animation: every segment of its BodyLayout as a rigid
// body; joints between them are ball joints with 3-axis muscles. Each frame the animation names
// the simulated bones (the leading clip's physics bones): those are dynamic and their muscles pull
// them towards the animated pose, so they move like the animation until they hit something; the
// rest follow the animation kinematically (riding on any simulated parent). The result goes back
// into the animator as bone overrides.
class PhysicalBody {
public:
    enum class Phase : u8 { Animated, Down, GettingUp };

    bool build(physics::PhysicsWorld& world, const anim::SkinnedModel& model, std::span<const glm::mat4> pose,
               const glm::mat4& entityWorld, std::span<const WeaponMount> weapons, const std::string& name, u32 collisionGroup,
               const BodySettings& settings = {});

    // Once per frame, before the physics steps: where the animation wants the body (this frame's
    // animated pose), which bones are simulated and how strong their muscles are.
    void setTargets(const glm::mat4& entityWorld, std::span<const glm::mat4> animatedModel, f32 frameTime,
                    std::span<const i32> physicsJoints, f32 strength);
    // Before each physics step. `alpha` (0..1]: how far through the frame this step is.
    void step(physics::PhysicsWorld& world, f32 dt, f32 alpha, f32 time);
    // After each physics step: weapon hits (appended to `events`) and the reactions to them.
    void afterStep(physics::PhysicsWorld& world, const SurfaceMap& surfaces, f32 time, std::vector<HitEvent>* events);
    // Writes the simulated bones into the animator (clears the overrides when nothing is simulated).
    void writeOverrides(const physics::PhysicsWorld& world, const glm::mat4& entityWorld, scene::AnimatorComponent& animator) const;
    static void clearOverrides(scene::AnimatorComponent& animator);

    [[nodiscard]] bool built() const { return !m_segments.empty(); }
    [[nodiscard]] Phase phase() const { return m_phase; }
    [[nodiscard]] bool down() const { return m_phase == Phase::Down; }
    [[nodiscard]] bool simulated() const; // any dynamic body
    [[nodiscard]] const BodyLayout& layout() const { return m_layout; }
    [[nodiscard]] usize segmentCount() const { return m_segments.size(); }
    [[nodiscard]] physics::BodyId segmentBody(usize i) const { return m_segments[i].body; }
    [[nodiscard]] bool segmentDynamic(usize i) const { return m_segments[i].dynamic; }
    [[nodiscard]] glm::vec3 segmentEnd(const physics::PhysicsWorld& world, usize i) const;
    // Muscle load (|torque| / strength) of the joint above segment i (0 for the root).
    [[nodiscard]] f32 load(const physics::PhysicsWorld& world, usize i) const;
    [[nodiscard]] usize weaponCount() const { return m_weapons.size(); }
    [[nodiscard]] physics::BodyId weapon(usize i = 0) const { return m_weapons[i].body; }
    [[nodiscard]] const std::vector<Hit>& hits(usize weapon = 0) const { return m_weapons[weapon].tracker.hits(); }
    // Angle (rad) between the simulated and the animated orientation of `joint`'s body.
    [[nodiscard]] f32 trackingError(const physics::PhysicsWorld& world, i32 joint) const;
    // World position of the pelvis body.
    [[nodiscard]] glm::vec3 rootPosition(const physics::PhysicsWorld& world) const;
    [[nodiscard]] f32 totalMass() const { return m_totalMass; }

private:
    struct Target {
        glm::vec3 position{0.0f};
        glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    struct Segment {
        i32 joint = -1;
        i32 parent = -1;
        physics::BodyId body;
        physics::JointId constraint; // to the parent (invalid for the root)
        physics::Muscle muscle;      // full strength
        glm::vec3 scale{1.0f};       // joint world scale (written back unchanged)
        glm::vec3 localEnd{0.0f};    // capsule end in body space
        f32 mass = 0.0f;
        bool wanted = false;         // in this frame's physics bones
        bool dynamic = false;
        f32 releaseStart = -1.0f;    // >= 0: leaving physics, still tracking
        f32 reflexStart = -1e9f;
        Target target, previous;     // animated world frame (this / last frame)
        Target driven;               // kinematic frame this step
    };
    struct Weapon {
        physics::BodyId body;
        i32 owner = -1;      // segment
        Target inOwner;      // weapon frame relative to the owner body
        std::vector<i32> chain; // segments that react to its hits (the arm)
        HitTracker tracker;
    };

    [[nodiscard]] Target interpolated(const Segment& s, f32 alpha) const;
    void setSegmentDynamic(physics::PhysicsWorld& world, usize i, bool dynamic);
    void syncWeapons(physics::PhysicsWorld& world);
    void knockDown(physics::PhysicsWorld& world, f32 time);
    void getUp(physics::PhysicsWorld& world, f32 time);
    void assist(physics::PhysicsWorld& world, f32 dt, f32 alpha);

    BodySettings m_settings;
    BodyLayout m_layout;
    std::vector<Segment> m_segments;
    std::vector<Weapon> m_weapons;
    f32 m_strength = 1.0f;
    f32 m_frameTime = 1.0f / 60.0f;
    bool m_hasTargets = false;
    f32 m_totalMass = 0.0f;
    f32 m_worldHeight = 1.8f;
    Phase m_phase = Phase::Animated;
    f32 m_phaseStart = 0.0f;
    f32 m_offBalance = 0.0f; // s the pelvis has been too far from the animation
    std::vector<glm::mat4> m_frozen; // world frames of the lying body (getting up)
    f32 m_getUpWeight = 1.0f;
};

// --- Strike analysis --------------------------------------------------------------------------------

// Where an attack clip's blade strikes, from the clip alone: the moment its middle moves fastest
// at the height of a standing opponent's body (25-85% of the skeleton's height; wind-ups over the
// head do not count). Model space with the clip's root motion added.
struct StrikePoint {
    f32 time = 0.5f;             // normalised clip time
    glm::vec3 bladeCenter{0.0f}; // model space
    glm::vec3 velocity{0.0f};    // of the blade centre, model units / s
};
[[nodiscard]] StrikePoint findStrikePoint(const anim::Skeleton& skeleton, const anim::AnimationClip& clip, i32 hand,
                                          const glm::mat4& weaponOffset, const scene::WeaponComponent& weapon);

// A test target (box standing on the ground at y = 0) for a strike: just before the strike
// point along the swing, a bit taller than it, so the blade meets its side at full speed but a wind-up over the
// head does not clip its top. World space.
struct TargetBox {
    glm::vec3 center{0.0f};
    glm::vec3 halfExtents{0.2f, 1.1f, 0.2f};
};
[[nodiscard]] TargetBox strikeTarget(const StrikePoint& strike, const glm::mat4& entityWorld, f32 radius = 0.2f);

} // namespace kinetic::physanim
