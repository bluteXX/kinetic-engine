#pragma once

#include <kinetic/animation/AnimationClip.h>
#include <kinetic/animation/AnimationPlayer.h>
#include <kinetic/animation/Skeleton.h>
#include <kinetic/physics/PhysicsWorld.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/Scene.h>

#include <imgui.h>

#include <array>
#include <deque>
#include <vector>

namespace kinetic::editor {

// Physics prototype: a sword arm strikes a stone, and every force on the way is measured.
//
// Rig (planar, all hinges about +Z): static torso -> upper arm -(shoulder)- forearm -(elbow)-
// hand -(wrist)- sword (welded at the grip). Each hinge has a muscle: a position motor with
// stiffness, damping and a torque limit (strength). The swing is an AnimationClip whose joint
// angles are the muscles' targets, so the arm only follows the animation as far as its muscles can.
// The stone is placed on the blade's path, computed from the clip.
//
// Measured each 240 Hz step: muscle torque and load (torque / strength), joint reaction forces,
// the grip force and torque, the contact force on the blade (exact, from the momentum balance),
// and the quasi-static estimate J^T F of the torque each joint needs to resist the contact. Over
// a hit: peaks, angular impulse and the energy each muscle absorbed.
class SwordStrikeTest {
public:
    SwordStrikeTest();
    ~SwordStrikeTest();

    [[nodiscard]] bool active() const { return m_active; }
    void create(scene::Scene& scene, renderer::Renderer& renderer);
    // Simulation without entities / meshes (tests, tools).
    bool createHeadless();
    // Runs fixed steps for `seconds` of simulated time (ignores pause / freeze-on-hit).
    void advance(f32 seconds);
    void setStepRate(f32 hz) { m_stepHz = hz; }
    void destroy(scene::Scene& scene, renderer::Renderer& renderer);

    // Advances the simulation by frameDt * timeScale in fixed steps and moves the entities.
    void update(f32 frameDt, scene::Scene& scene);

    // "Physics" window.
    void drawWindow(bool* open);
    // Debug geometry for the main viewport overlay.
    void appendDebugLines(std::vector<renderer::DebugVertex>& out) const;
    // Text labels over the viewport (joint torques, contact force).
    void drawViewportLabels(ImDrawList* draw, const glm::mat4& viewProjection, const ImVec2& p0, const ImVec2& size) const;

    // Read-only access for tests and tools.
    // Per joint over one hit. Rigid contacts are instantaneous, so per-step peaks scale with the
    // step rate; the impulses and energies are the step-independent measures.
    struct HitStats {
        f32 peakTorque = 0.0f;     // per step (rate dependent)
        f32 peakLoad = 0.0f;       // muscle torque / strength, 1 = saturated
        f32 peakForce = 0.0f;      // per step (rate dependent)
        glm::vec3 impulse{0.0f};   // integral of the joint force, N s
        f32 angularImpulse = 0.0f; // integral of muscle torque, N m s
        f32 absorbedEnergy = 0.0f; // work done against the muscle (eccentric), J
    };
    [[nodiscard]] const physics::PhysicsWorld& world() const { return m_world; }
    // One contact episode of the blade (contacts less than 50 ms apart belong to the same hit).
    struct HitRecord {
        f32 start = 0.0f;    // simulation time, s
        f32 duration = 0.0f; // s
        f32 peakContact = 0.0f; // N
        glm::vec3 impulse{0.0f}; // N s, on the blade
        std::array<HitStats, 4> joints{}; // shoulder, elbow, wrist, grip
    };
    [[nodiscard]] const std::vector<HitRecord>& hits() const { return m_hits; }
    // Hit with the largest contact impulse (the actual strike, not a graze), or null.
    [[nodiscard]] const HitRecord* strongestHit() const;
    [[nodiscard]] physics::JointId joint(usize index) const { return m_joints[index].id; }
    [[nodiscard]] physics::BodyId sword() const { return m_sword; }
    [[nodiscard]] glm::vec3 stoneCenter() const { return m_stoneCenter; }
    [[nodiscard]] glm::vec3 stoneHalfExtents() const { return m_stoneHalf; }
    [[nodiscard]] f32 jointToForceEstimate(usize hinge) const { return jToFEstimate(hinge); }

private:
    struct JointInfo {
        physics::JointId id;
        const char* name = "";
        bool hinge = true;
    };
    struct DebugOptions {
        bool shapes = true;
        bool centerOfMass = true;
        bool jointAxes = true;
        bool jointForces = true;
        bool muscleTorques = true;
        bool contactForces = true;
        bool gripForce = true;
        bool velocities = false;
        bool gravity = false;
        bool targetPose = true;
        bool bladeTrail = true;
        bool labels = true;
        f32 forceScale = 0.0015f; // m per N
        f32 velocityScale = 0.05f; // m per m/s
    };

    void buildClip();
    void buildWorld();
    void placeStone();
    void simulateStep(f32 dt);
    void resetAnalysis();
    // Forward kinematics of the arm for given joint angles (shoulder, elbow, wrist):
    // joint positions and the blade tip.
    struct ArmPose {
        glm::vec3 shoulder, elbow, wrist, grip, guard, tip;
    };
    [[nodiscard]] ArmPose forwardKinematics(const std::array<f32, 3>& angles) const;
    [[nodiscard]] std::array<f32, 3> clipAngles(f32 time) const;
    [[nodiscard]] f32 jToFEstimate(usize hinge) const;

    bool m_active = false;
    physics::PhysicsWorld m_world;
    std::vector<entt::entity> m_entities; // one per body, same index
    std::vector<scene::MeshId> m_meshes;

    // Bodies and joints.
    physics::BodyId m_torso, m_upperArm, m_forearm, m_hand, m_sword, m_stone, m_floor;
    std::array<JointInfo, 4> m_joints{}; // shoulder, elbow, wrist (hinges), grip (fixed)
    std::array<physics::Muscle, 3> m_muscles{};

    // Animation driving the muscles.
    anim::Skeleton m_skeleton;
    std::vector<anim::AnimationClip> m_clips;
    anim::AnimationPlayer m_player;
    std::array<f32, 3> m_targets{};

    // Stone placement on the blade's path.
    glm::vec3 m_stoneCenter{1.0f, 0.5f, 0.0f};
    glm::vec3 m_stoneHalf{0.2f, 0.5f, 0.3f};

    // Simulation control.
    f32 m_stepHz = 960.0f; // rigid contacts resolve in ~1 step: small steps keep penetration small
    f32 m_contactTimeMs = 1.0f; // assumed contact duration for the peak-force estimate
    f32 m_timeScale = 1.0f;
    f32 m_accumulator = 0.0f;
    f32 m_simTime = 0.0f;
    bool m_paused = false;
    bool m_stepOnce = false;
    bool m_freezeOnHit = true;
    bool m_loop = true;

    // Analysis.
    bool m_inHit = false;
    std::vector<HitRecord> m_hits;
    i32 m_selectedHit = -1; // -1: follow the latest hit

    static constexpr usize kHistory = 1920; // samples (2 s at 960 Hz)
    std::array<std::vector<f32>, 3> m_torqueHistory;
    std::array<std::vector<f32>, 3> m_loadHistory;
    std::vector<f32> m_contactHistory;
    usize m_historyHead = 0;

    std::deque<std::pair<glm::vec3, f32>> m_trail; // blade tip positions and speeds

    DebugOptions m_debug;
};

} // namespace kinetic::editor
