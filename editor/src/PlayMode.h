#pragma once

#include <kinetic/animation/AnimationPlayer.h>
#include <kinetic/animation/Skinning.h>
#include <kinetic/physanim/HitTracker.h>
#include <kinetic/physanim/PhysicalBody.h>
#include <kinetic/physics/PhysicsWorld.h>
#include <kinetic/renderer/Renderer.h>
#include <kinetic/scene/Scene.h>
#include <kinetic/vfx/Vfx.h>

#include <imgui.h>

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinetic::editor {

// Input for one frame of Play mode (already mapped from keys / mouse).
struct PlayInput {
    glm::vec2 move{0.0f}; // x = right, y = forward, relative to the camera; length <= 1
    bool run = false;
    bool attack = false;        // pressed this frame
    bool toggleRagdoll = false; // pressed this frame
    bool jump = false;          // pressed this frame (heroes)
    i32 playAttack = -1;        // pressed this frame: start the combo's attack N (0-based) directly (testing)
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

    using HitResult = physanim::HitResult;
    using Hit = physanim::Hit;
    using SurfaceMap = physanim::SurfaceMap;

    bool spawn(physics::PhysicsWorld& world, scene::Scene& scene, entt::entity entity, u32 collisionGroup);
    void step(f32 dt, physics::PhysicsWorld& world, const PlayInput& input, const scene::Scene& scene,
              const std::vector<std::pair<glm::vec3, glm::vec3>>& obstacles);
    // After each physics step: contacts of the blade (starts the hit reaction), hit statistics.
    void afterStep(const physics::PhysicsWorld& world, const SurfaceMap& surfaces = {}, std::vector<physanim::HitEvent>* events = nullptr);
    // Writes the simulated pose into the entity's AnimatorComponent and Transform.
    void writePose(scene::Scene& scene, const physics::PhysicsWorld& world);
    void appendDebugLines(std::vector<renderer::DebugVertex>& out, const physics::PhysicsWorld& world, bool shapes) const;

    [[nodiscard]] entt::entity entity() const { return m_entity; }
    [[nodiscard]] Mode mode() const { return m_mode; }
    [[nodiscard]] AttackPhase attackPhase() const { return m_phase; }
    [[nodiscard]] glm::vec3 rootPosition() const { return m_root; }
    [[nodiscard]] f32 yaw() const { return m_yaw; }
    [[nodiscard]] f32 speed() const { return m_speed; }
    [[nodiscard]] const std::vector<Hit>& hits() const { return m_tracker.hits(); }
    [[nodiscard]] physics::BodyId body(u32 joint) const { return m_bodies[joint]; }
    [[nodiscard]] static const char* resultName(HitResult result) { return physanim::hitResultName(result); }

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
    physanim::HitTracker m_tracker;

    mutable anim::Pose m_pose;
    mutable anim::Pose m_scratch;
    mutable std::vector<glm::mat4> m_world;
};

// Play mode of the editor: builds a physics world from the scene (colliders + characters), runs
// it, drives the camera and restores the scene when stopped. Actors with a
// gameplay::CharacterControllerComponent (heroes) are moved by the engine's character controller
// (their blend tree follows the speed, foot IK keeps the feet on the colliders).
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
    [[nodiscard]] const std::vector<entt::entity>& heroes() const { return m_heroes; }
    // The physics body of `hero` (nullptr: none / not built yet).
    [[nodiscard]] const physanim::PhysicalBody* heroBody(entt::entity hero) const;
    // "On hit" events of the last update (weapons striking things), and the effects they started.
    [[nodiscard]] const std::vector<physanim::HitEvent>& hitEvents() const { return m_hitEvents; }
    [[nodiscard]] const vfx::VfxSystem& vfx() const { return m_vfx; }
    // Effects played on hits (assets/vfx); without a library hits play none.
    void setVfxLibrary(const vfx::VfxLibrary* library) { m_vfxLibrary = library; }
    // The effect a hit plays: the struck collider's onHit, else "hit_<material>"; empty = none.
    [[nodiscard]] std::string hitEffect(const physanim::HitEvent& hit) const;

private:
    struct Snapshot {
        scene::TransformComponent transform;
    };

    bool m_active = false;
    std::unique_ptr<physics::PhysicsWorld> m_world;
    std::vector<std::unique_ptr<PlayCharacter>> m_characters;
    PlayCharacter::SurfaceMap m_surfaces; // static colliders' materials
    std::vector<entt::entity> m_heroes;
    // HUD info about the first hero (filled by update).
    f32 m_heroSpeed = 0.0f;
    bool m_heroGrounded = true;
    std::vector<std::pair<glm::vec3, glm::vec3>> m_obstacles; // static collider AABBs (world)
    std::unordered_map<entt::entity, Snapshot> m_snapshot;
    entt::entity m_camera = entt::null;
    f32 m_accumulator = 0.0f;
    f32 m_time = 0.0f; // simulated seconds since start
    // Heroes' physics bodies (physics animation): the bones the leading clip marks as physical
    // run on muscles (built on the first frame that has an animated pose).
    struct HeroBody {
        entt::entity hero = entt::null;
        u32 group = 0;
        bool failed = false;
        physanim::PhysicalBody body;
        u32 hardHits = 0;          // body.hardHits() already reacted to
        f32 hitStopEnd = -1.0f;    // physics time the hit stop ends
        i32 bounceNext = -1;       // after bouncing off a wall: the combo's next attack ...
        f32 bounceUntil = -1.0f;   // ... a press until then goes straight into it
    };
    // A hard hit on a wall during an attack: hit stop, then the rest of the swing becomes a move
    // into the next attack of the combo (when one is queued) or into the guard.
    void reactToHardHits(scene::Scene& scene);
    std::vector<std::unique_ptr<HeroBody>> m_bodies;
    void updateHeroBodies(scene::Scene& scene, f32 frameTime);
    std::vector<physanim::HitEvent> m_hitEvents;
    std::unordered_map<u32, entt::entity> m_bodyEntity; // static collider bodies -> entities
    std::unordered_map<u32, std::string> m_bodyEffect;  // ... -> their onHit effect
    vfx::VfxSystem m_vfx;
    const vfx::VfxLibrary* m_vfxLibrary = nullptr;
    f32 m_stepHz = 960.0f;

    f32 m_cameraYaw = 0.0f;
    f32 m_cameraPitch = 0.3f;
    f32 m_cameraDistance = 3.8f;
};

} // namespace kinetic::editor
