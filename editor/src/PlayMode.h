#pragma once

#include <kinetic/animation/AnimationPlayer.h>
#include <kinetic/animation/Skinning.h>
#include <kinetic/physics/PhysicsWorld.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/Scene.h>

#include <imgui.h>

#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

namespace kinetic::editor {

// Input for one frame of Play mode (already mapped from keys / mouse).
struct PlayInput {
    glm::vec2 move{0.0f}; // x = right, y = forward, relative to the camera; length <= 1
    bool run = false;
    bool attack = false;        // pressed this frame
    bool toggleRagdoll = false; // pressed this frame
    f32 cameraYaw = 0.0f;       // radians; forward = (sin yaw, 0, cos yaw)
};

// One playable character (an entity with CharacterComponent + AnimatorComponent).
//
//   Locomotion  every body is kinematic and follows the Idle / Walk clips (regular animation)
//   Attack      the sword arm (upper arm, forearm, hand, sword) becomes dynamic: its muscles
//               follow the Strike clip (physics animation), hits rebound, then a reaction
//               (reflex -> recover) brings the arm back; the rest of the body keeps walking
//   Ragdoll     every body is dynamic with the muscles off: the character collapses
class PlayCharacter {
public:
    enum class Mode : u8 { Locomotion, Attack, Ragdoll };
    enum class AttackPhase : u8 { Swing, Reflex, Recover };

    struct Hit {
        f32 impulse = 0.0f;     // N s
        f32 peakLoad[3] = {};   // shoulder, elbow, wrist
        f32 time = 0.0f;
    };

    bool spawn(physics::PhysicsWorld& world, scene::Scene& scene, entt::entity entity, u32 collisionGroup);
    void step(f32 dt, physics::PhysicsWorld& world, const PlayInput& input, const scene::Scene& scene,
              const std::vector<std::pair<glm::vec3, glm::vec3>>& obstacles);
    // After each physics step: contacts of the blade (starts the hit reaction), hit statistics.
    void afterStep(const physics::PhysicsWorld& world);
    // Writes the simulated pose into the entity's AnimatorComponent and Transform.
    void writePose(scene::Scene& scene, const physics::PhysicsWorld& world);
    void appendDebugLines(std::vector<renderer::DebugVertex>& out, const physics::PhysicsWorld& world, bool shapes) const;

    [[nodiscard]] entt::entity entity() const { return m_entity; }
    [[nodiscard]] Mode mode() const { return m_mode; }
    [[nodiscard]] AttackPhase attackPhase() const { return m_phase; }
    [[nodiscard]] glm::vec3 rootPosition() const { return m_root; }
    [[nodiscard]] f32 yaw() const { return m_yaw; }
    [[nodiscard]] f32 speed() const { return m_speed; }
    [[nodiscard]] const std::vector<Hit>& hits() const { return m_hits; }
    [[nodiscard]] physics::BodyId body(u32 joint) const { return m_bodies[joint]; }

private:
    void startAttack(physics::PhysicsWorld& world);
    void endAttack(physics::PhysicsWorld& world);
    void setRagdoll(physics::PhysicsWorld& world, bool on);
    void locomotionPose(anim::Pose& out) const;
    [[nodiscard]] glm::mat4 rootMatrix() const;
    // World transforms of every joint for a local pose.
    void worldTransforms(const anim::Pose& pose, std::vector<glm::mat4>& out) const;

    entt::entity m_entity = entt::null;
    std::shared_ptr<const anim::SkinnedModel> m_model;
    std::vector<physics::BodyId> m_bodies; // per joint (invalid for the root)
    std::vector<physics::JointId> m_joints; // per joint, to its parent (invalid for root / pelvis)
    f32 m_strength = 1.0f;
    f32 m_walkSpeed = 1.6f;
    f32 m_runSpeed = 3.6f;

    Mode m_mode = Mode::Locomotion;
    glm::vec3 m_root{0.0f};
    f32 m_yaw = 0.0f;
    f32 m_speed = 0.0f;
    f32 m_time = 0.0f;

    anim::AnimationPlayer m_locomotion;
    anim::AnimationPlayer m_attack;
    AttackPhase m_phase = AttackPhase::Swing;
    f32 m_phaseStart = 0.0f;
    std::vector<Hit> m_hits;
    bool m_inContact = false;

    mutable anim::Pose m_pose;
    mutable anim::Pose m_scratch;
    mutable std::vector<glm::mat4> m_world;
};

// Play mode of the editor: builds a physics world from the scene (colliders + characters), runs
// it, drives the camera and restores the scene when stopped.
class PlayMode {
public:
    PlayMode();
    ~PlayMode();

    [[nodiscard]] bool active() const { return m_active; }
    void start(scene::Scene& scene, entt::entity camera);
    void stop(scene::Scene& scene);

    // Simulation (fixed steps) for one frame.
    void update(f32 dt, const PlayInput& input, scene::Scene& scene);
    // Third-person camera: orbit with `orbitDelta` (pixels), zoom with `wheel`.
    void updateCamera(scene::Scene& scene, const glm::vec2& orbitDelta, f32 wheel);
    [[nodiscard]] f32 cameraYaw() const { return m_cameraYaw; }

    void appendDebugLines(std::vector<renderer::DebugVertex>& out) const;
    void drawHud(ImDrawList* draw, const ImVec2& p0, const ImVec2& size) const;

    bool showDebug = true;
    bool showShapes = false;

    // For tests / tools.
    [[nodiscard]] const physics::PhysicsWorld& world() const { return *m_world; }
    [[nodiscard]] const std::vector<std::unique_ptr<PlayCharacter>>& characters() const { return m_characters; }

private:
    struct Snapshot {
        scene::TransformComponent transform;
    };

    bool m_active = false;
    std::unique_ptr<physics::PhysicsWorld> m_world;
    std::vector<std::unique_ptr<PlayCharacter>> m_characters;
    std::vector<std::pair<glm::vec3, glm::vec3>> m_obstacles; // static collider AABBs (world)
    std::unordered_map<entt::entity, Snapshot> m_snapshot;
    entt::entity m_camera = entt::null;
    f32 m_accumulator = 0.0f;
    f32 m_stepHz = 960.0f;

    f32 m_cameraYaw = 0.0f;
    f32 m_cameraPitch = 0.3f;
    f32 m_cameraDistance = 3.8f;
};

} // namespace kinetic::editor
