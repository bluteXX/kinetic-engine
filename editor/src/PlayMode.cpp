#include "PlayMode.h"

#include "CharacterRig.h"

#include <kinetic/animation/PoseOps.h>
#include <kinetic/scene/AnimatorComponent.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kinetic::editor {

namespace {

namespace ch = character;

constexpr f32 kPi = 3.14159265f;
constexpr f32 kCharacterRadius = 0.3f;
constexpr f32 kStepUp = 0.35f;
constexpr u32 kArmJoints[3] = {ch::kUpperArmR, ch::kForearmR, ch::kHandR};
constexpr u32 kArmBodies[4] = {ch::kUpperArmR, ch::kForearmR, ch::kHandR, ch::kSword};

// Reaction to a hit (same model as the sword test): reflex, then recover.
constexpr f32 kReflex = 0.05f;
constexpr f32 kRecoverTime = 0.4f;
constexpr f32 kHold = 0.3f;
constexpr f32 kRecoveryClipTime = 1.15f;
constexpr f32 kRestitution = 0.35f;

f32 wrapAngle(f32 a)
{
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

glm::quat yawRotation(f32 yaw) { return glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f)); }

glm::vec3 loadColor(f32 load)
{
    load = std::clamp(load, 0.0f, 1.0f);
    return load < 0.5f ? glm::mix(glm::vec3(0.25f, 0.85f, 0.35f), glm::vec3(1.0f, 0.85f, 0.2f), load * 2.0f)
                       : glm::mix(glm::vec3(1.0f, 0.85f, 0.2f), glm::vec3(1.0f, 0.2f, 0.15f), load * 2.0f - 1.0f);
}

void line(std::vector<renderer::DebugVertex>& out, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    out.push_back({a, c});
    out.push_back({b, c});
}

void arrow(std::vector<renderer::DebugVertex>& out, const glm::vec3& from, const glm::vec3& v, const glm::vec3& c)
{
    const f32 length = glm::length(v);
    if (length < 1e-5f) {
        return;
    }
    const glm::vec3 d = v / length;
    glm::vec3 side = glm::cross(d, glm::vec3(0, 1, 0));
    side = glm::dot(side, side) > 1e-6f ? glm::normalize(side) : glm::vec3(1, 0, 0);
    const f32 head = std::min(0.06f, length * 0.3f);
    const glm::vec3 tip = from + v;
    line(out, from, tip, c);
    line(out, tip, tip - d * head + side * head * 0.5f, c);
    line(out, tip, tip - d * head - side * head * 0.5f, c);
}

void arc(std::vector<renderer::DebugVertex>& out, const glm::vec3& center, const glm::vec3& axis, const glm::vec3& start,
         f32 radius, f32 angle, const glm::vec3& c)
{
    const int segments = std::max(4, static_cast<int>(std::abs(angle) / 0.15f));
    glm::vec3 previous = center + start * radius;
    for (int i = 1; i <= segments; ++i) {
        const glm::vec3 p = center + glm::angleAxis(angle * static_cast<f32>(i) / static_cast<f32>(segments), axis) * start * radius;
        line(out, previous, p, c);
        previous = p;
    }
}

} // namespace

// =============================================================================================== //
// PlayCharacter

glm::mat4 PlayCharacter::rootMatrix() const
{
    return glm::translate(glm::mat4(1.0f), m_root) * glm::mat4_cast(yawRotation(m_yaw));
}

void PlayCharacter::worldTransforms(const anim::Pose& pose, std::vector<glm::mat4>& out) const
{
    out.resize(m_model->skeleton.jointCount());
    anim::localToModel(m_model->skeleton, pose, out);
    const glm::mat4 root = rootMatrix();
    for (glm::mat4& m : out) {
        m = root * m;
    }
}

void PlayCharacter::locomotionPose(anim::Pose& out) const
{
    m_locomotion.evaluate(m_model->skeleton, m_model->clips, out, m_scratch);
}

bool PlayCharacter::spawn(physics::PhysicsWorld& world, scene::Scene& scene, entt::entity entity, u32 collisionGroup)
{
    auto& registry = scene.registry();
    const auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    const auto* settings = registry.try_get<scene::CharacterComponent>(entity);
    if (!animator || !animator->model || !settings || animator->model->skeleton.jointCount() != ch::kJointCount ||
        animator->model->clips.size() <= static_cast<usize>(ch::kStrike)) {
        return false;
    }
    m_entity = entity;
    m_model = animator->model;
    m_strength = settings->strength;
    m_walkSpeed = settings->walkSpeed;
    m_runSpeed = settings->runSpeed;

    const auto& transform = registry.get<scene::TransformComponent>(entity);
    m_root = transform.position;
    const glm::vec3 forward = transform.rotation * glm::vec3(0.0f, 0.0f, 1.0f);
    m_yaw = std::atan2(forward.x, forward.z);

    // Bodies in the rest pose (hinge angles are measured from it), then moved to the idle pose.
    const auto& parts = ch::parts();
    const glm::mat4 root = rootMatrix();
    const glm::quat rootRotation = yawRotation(m_yaw);
    m_bodies.assign(ch::kJointCount, physics::BodyId{});
    m_joints.assign(ch::kJointCount, physics::JointId{});
    for (u32 j = 0; j < ch::kJointCount; ++j) {
        const ch::Part& part = parts[j];
        if (part.mass <= 0.0f) {
            continue;
        }
        physics::BodyDesc d;
        d.name = part.name;
        d.shape = part.shape;
        d.position = glm::vec3(root * glm::vec4(part.restPosition, 1.0f));
        d.rotation = rootRotation;
        d.motion = physics::MotionType::Kinematic;
        d.mass = part.mass;
        d.centerOfMass = part.centerOfMass;
        d.collisionGroup = collisionGroup;
        d.friction = j == ch::kSword ? 0.4f : 0.7f;
        d.restitution = j == ch::kSword ? kRestitution : 0.05f;
        d.continuousCollision = j == ch::kSword;
        m_bodies[j] = world.addBody(d);
    }
    const glm::vec3 sagittalAxis = rootRotation * ch::kSagittalAxis;
    const glm::vec3 down = rootRotation * glm::vec3(0.0f, -1.0f, 0.0f);
    const glm::vec3 side = rootRotation * glm::vec3(1.0f, 0.0f, 0.0f);
    for (u32 j = 0; j < ch::kJointCount; ++j) {
        const ch::Part& part = parts[j];
        if (part.joint == ch::JointType::None || part.parent < 0) {
            continue;
        }
        const physics::BodyId parent = m_bodies[static_cast<usize>(part.parent)];
        const glm::vec3 anchor = glm::vec3(root * glm::vec4(part.restPosition, 1.0f));
        switch (part.joint) {
        case ch::JointType::Hinge: {
            physics::HingeDesc h;
            h.name = part.name;
            h.parent = parent;
            h.child = m_bodies[j];
            h.anchor = anchor;
            h.axis = sagittalAxis;
            h.referenceParent = down;
            h.referenceChild = down;
            h.limitMin = glm::radians(part.limitMin);
            h.limitMax = glm::radians(part.limitMax);
            h.muscle = part.muscle;
            h.muscle.enabled = false; // only while the arm is simulated
            h.friction = 2.0f;
            m_joints[j] = world.addHinge(h);
            break;
        }
        case ch::JointType::SwingTwist: {
            physics::SwingTwistDesc s;
            s.name = part.name;
            s.parent = parent;
            s.child = m_bodies[j];
            s.anchor = anchor;
            const bool upwards = j == ch::kChest || j == ch::kHead;
            s.twistAxis = upwards ? -down : down;
            s.planeAxis = side;
            s.normalHalfCone = s.planeHalfCone = glm::radians(part.cone);
            s.twistMin = -glm::radians(part.twist);
            s.twistMax = glm::radians(part.twist);
            s.friction = 3.0f;
            m_joints[j] = world.addSwingTwist(s);
            break;
        }
        case ch::JointType::Fixed: {
            physics::FixedDesc f;
            f.name = part.name;
            f.parent = parent;
            f.child = m_bodies[j];
            f.anchor = anchor;
            m_joints[j] = world.addFixed(f);
            break;
        }
        case ch::JointType::None: break;
        }
    }

    m_locomotion = {};
    m_locomotion.play(ch::kIdle);
    m_mode = Mode::Locomotion;
    locomotionPose(m_pose);
    worldTransforms(m_pose, m_world);
    for (u32 j = 0; j < ch::kJointCount; ++j) {
        if (m_bodies[j].valid()) {
            world.teleport(m_bodies[j], glm::vec3(m_world[j][3]), glm::quat_cast(glm::mat3(m_world[j])));
        }
    }
    return true;
}

void PlayCharacter::startAttack(physics::PhysicsWorld& world)
{
    m_mode = Mode::Attack;
    m_phase = AttackPhase::Swing;
    m_attack = {};
    m_attack.loop = false;
    m_attack.play(ch::kStrike);
    for (const u32 j : kArmBodies) {
        world.setMotionType(m_bodies[j], physics::MotionType::Dynamic);
    }
    for (const u32 j : kArmJoints) {
        physics::Muscle muscle = ch::parts()[j].muscle;
        muscle.maxTorque *= m_strength;
        muscle.enabled = true;
        world.setMuscle(m_joints[j], muscle);
    }
}

void PlayCharacter::endAttack(physics::PhysicsWorld& world)
{
    m_mode = Mode::Locomotion;
    for (const u32 j : kArmBodies) {
        world.setMotionType(m_bodies[j], physics::MotionType::Kinematic);
    }
    for (const u32 j : kArmJoints) {
        physics::Muscle muscle = ch::parts()[j].muscle;
        muscle.enabled = false;
        world.setMuscle(m_joints[j], muscle);
    }
}

void PlayCharacter::setRagdoll(physics::PhysicsWorld& world, bool on)
{
    if (on) {
        m_mode = Mode::Ragdoll;
        for (const physics::BodyId body : m_bodies) {
            if (body.valid()) {
                world.setMotionType(body, physics::MotionType::Dynamic);
            }
        }
        for (const u32 j : kArmJoints) {
            physics::Muscle muscle = ch::parts()[j].muscle;
            muscle.enabled = false;
            world.setMuscle(m_joints[j], muscle);
        }
        return;
    }
    // Stand up where the pelvis lies (no get-up animation yet: the pose snaps to idle).
    const glm::vec3 pelvis = world.body(m_bodies[ch::kPelvis]).position;
    m_root.x = pelvis.x;
    m_root.z = pelvis.z;
    m_mode = Mode::Locomotion;
    m_speed = 0.0f;
    m_locomotion.play(ch::kIdle);
    locomotionPose(m_pose);
    worldTransforms(m_pose, m_world);
    for (u32 j = 0; j < ch::kJointCount; ++j) {
        if (m_bodies[j].valid()) {
            world.setMotionType(m_bodies[j], physics::MotionType::Kinematic);
            world.teleport(m_bodies[j], glm::vec3(m_world[j][3]), glm::quat_cast(glm::mat3(m_world[j])));
        }
    }
}

void PlayCharacter::step(f32 dt, physics::PhysicsWorld& world, const PlayInput& input, const scene::Scene& /*scene*/,
                         const std::vector<std::pair<glm::vec3, glm::vec3>>& obstacles)
{
    m_time += dt;
    if (input.toggleRagdoll) {
        setRagdoll(world, m_mode != Mode::Ragdoll);
    }
    if (m_mode == Mode::Ragdoll) {
        return; // bodies are simulated; afterStep-like bookkeeping happens in writePose
    }
    if (input.attack && m_mode == Mode::Locomotion) {
        startAttack(world);
    }

    // --- Locomotion: move the root, turn towards the movement ---
    const glm::vec3 forward(std::sin(input.cameraYaw), 0.0f, std::cos(input.cameraYaw));
    const glm::vec3 right(-forward.z, 0.0f, forward.x);
    glm::vec3 direction = right * input.move.x + forward * input.move.y;
    const f32 amount = std::min(glm::length(direction), 1.0f);
    const f32 targetSpeed = amount * (input.run ? m_runSpeed : m_walkSpeed);
    const f32 accel = 8.0f * dt;
    m_speed += std::clamp(targetSpeed - m_speed, -accel, accel);
    if (amount > 0.05f) {
        direction = glm::normalize(direction);
        const f32 targetYaw = std::atan2(direction.x, direction.z);
        const f32 turn = wrapAngle(targetYaw - m_yaw);
        m_yaw = wrapAngle(m_yaw + std::clamp(turn, -10.0f * dt, 10.0f * dt));
    }
    // Walk where the character faces (it turns quickly), so the steps match the movement.
    const glm::vec3 facing(std::sin(m_yaw), 0.0f, std::cos(m_yaw));
    m_root += facing * m_speed * dt;

    // Static obstacles: push out in XZ; stand on the highest top within step-up reach.
    f32 ground = -1e9f;
    for (const auto& [lo, hi] : obstacles) {
        const bool under = m_root.x >= lo.x && m_root.x <= hi.x && m_root.z >= lo.z && m_root.z <= hi.z;
        if (hi.y <= m_root.y + kStepUp) {
            if (under) {
                ground = std::max(ground, hi.y);
            }
            continue;
        }
        if (lo.y > m_root.y + 1.8f) {
            continue;
        }
        const glm::vec2 minXZ(lo.x - kCharacterRadius, lo.z - kCharacterRadius);
        const glm::vec2 maxXZ(hi.x + kCharacterRadius, hi.z + kCharacterRadius);
        if (m_root.x > minXZ.x && m_root.x < maxXZ.x && m_root.z > minXZ.y && m_root.z < maxXZ.y) {
            const f32 pushes[4] = {m_root.x - minXZ.x, maxXZ.x - m_root.x, m_root.z - minXZ.y, maxXZ.y - m_root.z};
            const usize smallest = static_cast<usize>(std::min_element(pushes, pushes + 4) - pushes);
            switch (smallest) {
            case 0: m_root.x = minXZ.x; break;
            case 1: m_root.x = maxXZ.x; break;
            case 2: m_root.z = minXZ.y; break;
            default: m_root.z = maxXZ.y; break;
            }
        }
    }
    if (ground > -1e8f) {
        m_root.y = ground;
    }

    // Idle <-> walk, walk cadence follows the speed.
    const bool walking = m_locomotion.clip() == ch::kWalk;
    if (m_speed > 0.15f && !walking) {
        m_locomotion.play(ch::kWalk, 0.2f);
    } else if (m_speed < 0.1f && walking) {
        m_locomotion.play(ch::kIdle, 0.25f);
    }
    m_locomotion.speed = m_locomotion.clip() == ch::kWalk ? std::max(m_speed / ch::kWalkClipSpeed, 0.3f) : 1.0f;
    m_locomotion.update(dt, m_model->clips);

    // --- Attack: muscles follow the Strike clip, with the hit reaction ---
    if (m_mode == Mode::Attack) {
        m_attack.update(dt, m_model->clips);
        anim::Pose attackPose = m_model->skeleton.restPose();
        m_model->clips[ch::kStrike].sample(m_attack.time(), false, attackPose);
        const f32 since = m_time - m_phaseStart;
        if (m_phase == AttackPhase::Reflex && since >= kReflex) {
            m_phase = AttackPhase::Recover;
            m_phaseStart = m_time;
            m_attack.setTime(kRecoveryClipTime, m_model->clips);
            m_attack.playing = true;
        }
        for (const u32 j : kArmJoints) {
            const f32 animation = ch::sagittalAngle(attackPose.rotations[j]);
            f32 target = animation;
            if (m_phase == AttackPhase::Recover) {
                const f32 x = std::clamp((m_time - m_phaseStart) / kRecoverTime, 0.0f, 1.0f);
                const f32 w = kHold + (1.0f - kHold) * x * x * (3.0f - 2.0f * x);
                const f32 current = world.joint(m_joints[j]).angle;
                target = current + w * (animation - current);
            }
            world.setTargetAngle(m_joints[j], target);
        }
        if (m_phase == AttackPhase::Recover && m_time - m_phaseStart >= kRecoverTime) {
            m_phase = AttackPhase::Swing;
        }
        if (!m_attack.playing && m_phase == AttackPhase::Swing) {
            endAttack(world);
        }
    }

    // --- Kinematic bodies follow the regular animation ---
    locomotionPose(m_pose);
    worldTransforms(m_pose, m_world);
    for (u32 j = 0; j < ch::kJointCount; ++j) {
        if (!m_bodies[j].valid()) {
            continue;
        }
        const bool simulated = m_mode == Mode::Attack && ch::parts()[j].swordArm;
        if (!simulated) {
            world.moveKinematic(m_bodies[j], glm::vec3(m_world[j][3]), glm::quat_cast(glm::mat3(m_world[j])), dt);
        }
    }
}

void PlayCharacter::afterStep(const physics::PhysicsWorld& world)
{
    if (m_mode == Mode::Ragdoll) {
        // The root follows the pelvis on the ground.
        const glm::vec3 pelvis = world.body(m_bodies[ch::kPelvis]).position;
        m_root.x = pelvis.x;
        m_root.z = pelvis.z;
        return;
    }
    if (m_mode != Mode::Attack) {
        m_inContact = false;
        return;
    }
    const physics::BodyState& sword = world.body(m_bodies[ch::kSword]);
    const bool touching = sword.inContact && glm::length(sword.contactForce) > 1.0f;
    if (touching && !m_inContact) {
        m_hits.push_back(Hit{.time = m_time});
        if (m_hits.size() > 16) {
            m_hits.erase(m_hits.begin());
        }
    }
    if (touching && m_phase == AttackPhase::Swing) {
        m_phase = AttackPhase::Reflex; // the reaction starts at the impact
        m_phaseStart = m_time;
    }
    m_inContact = touching;
    if (!m_hits.empty() && m_time - m_hits.back().time < 0.3f) {
        Hit& hit = m_hits.back();
        if (touching) {
            hit.impulse += glm::length(sword.contactForce) * world.lastStep();
        }
        for (usize i = 0; i < 3; ++i) {
            const f32 strength = ch::parts()[kArmJoints[i]].muscle.maxTorque * m_strength;
            hit.peakLoad[i] = std::max(hit.peakLoad[i], std::abs(world.joint(m_joints[kArmJoints[i]]).muscleTorque) / strength);
        }
    }
}

void PlayCharacter::writePose(scene::Scene& scene, const physics::PhysicsWorld& world)
{
    auto& registry = scene.registry();
    if (!registry.valid(m_entity)) {
        return;
    }
    auto* animator = registry.try_get<scene::AnimatorComponent>(m_entity);
    auto& transform = registry.get<scene::TransformComponent>(m_entity);
    transform.position = m_root;
    transform.rotation = yawRotation(m_yaw);
    if (!animator) {
        return;
    }

    // Local pose from the bodies: local = inverse(parent world) * world.
    const auto& parents = m_model->skeleton.parents();
    std::vector<glm::vec3> positions(ch::kJointCount);
    std::vector<glm::quat> rotations(ch::kJointCount);
    positions[ch::kRoot] = m_root;
    rotations[ch::kRoot] = yawRotation(m_yaw);
    animator->localPose = m_model->skeleton.restPose();
    for (u32 j = 1; j < ch::kJointCount; ++j) {
        const physics::BodyState& b = world.body(m_bodies[j]);
        positions[j] = b.position;
        rotations[j] = b.rotation;
        const usize p = static_cast<usize>(parents[j]);
        const glm::quat inverseParent = glm::conjugate(rotations[p]);
        animator->localPose.translations[j] = inverseParent * (positions[j] - positions[p]);
        animator->localPose.rotations[j] = glm::normalize(inverseParent * rotations[j]);
        animator->localPose.scales[j] = glm::vec3(1.0f);
    }
    animator->localPose.translations[ch::kRoot] = glm::vec3(0.0f);
    animator->localPose.rotations[ch::kRoot] = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    animator->externalPose = true;
    animator->bindPose = false;
}

void PlayCharacter::appendDebugLines(std::vector<renderer::DebugVertex>& out, const physics::PhysicsWorld& world,
                                     bool shapes) const
{
    if (shapes) {
        for (const physics::BodyId body : m_bodies) {
            if (body.valid()) {
                const physics::BodyState& s = world.body(body);
                const f32 size = 0.02f;
                const glm::vec3 c = s.centerOfMass;
                const glm::vec3 color = world.bodyDesc(body).motion == physics::MotionType::Dynamic
                                            ? glm::vec3(1.0f, 0.6f, 0.2f)
                                            : glm::vec3(0.5f, 0.7f, 1.0f);
                line(out, c - glm::vec3(size, 0, 0), c + glm::vec3(size, 0, 0), color);
                line(out, c - glm::vec3(0, size, 0), c + glm::vec3(0, size, 0), color);
                line(out, c - glm::vec3(0, 0, size), c + glm::vec3(0, 0, size), color);
            }
        }
    }
    if (m_mode != Mode::Attack) {
        return;
    }
    for (const u32 j : kArmJoints) {
        const physics::JointState& s = world.joint(m_joints[j]);
        const f32 strength = ch::parts()[j].muscle.maxTorque * m_strength;
        const f32 load = std::clamp(s.muscleTorque / strength, -1.0f, 1.0f);
        const glm::vec3 start = world.body(m_bodies[j]).rotation * glm::vec3(0.0f, -1.0f, 0.0f);
        arc(out, s.anchor, s.axis, start, 0.08f, load * glm::radians(270.0f), loadColor(std::abs(load)));
        arrow(out, s.anchor, s.force * 0.0015f, {1.0f, 0.55f, 0.15f});
    }
    const physics::BodyState& sword = world.body(m_bodies[ch::kSword]);
    if (sword.inContact) {
        arrow(out, sword.contactPoint, sword.contactForce * 0.0015f, {1.0f, 0.2f, 0.2f});
    }
}

// =============================================================================================== //
// PlayMode

PlayMode::PlayMode() = default;
PlayMode::~PlayMode() = default;

void PlayMode::start(scene::Scene& scene, entt::entity camera)
{
    if (m_active) {
        return;
    }
    auto& registry = scene.registry();
    m_snapshot.clear();
    for (const entt::entity e : registry.view<scene::TransformComponent>()) {
        m_snapshot[e] = Snapshot{registry.get<scene::TransformComponent>(e)};
    }
    m_camera = camera;

    m_world = std::make_unique<physics::PhysicsWorld>();
    m_world->init();
    m_world->setGravity({0.0f, -9.81f, 0.0f});

    // Static colliders (floor, stones): physics bodies + AABBs for the character movement.
    m_obstacles.clear();
    for (const entt::entity e : registry.view<scene::ColliderComponent, scene::TransformComponent>()) {
        const auto& collider = registry.get<scene::ColliderComponent>(e);
        const auto& t = registry.get<scene::TransformComponent>(e);
        physics::BodyDesc d;
        d.name = registry.all_of<scene::NameComponent>(e) ? registry.get<scene::NameComponent>(e).name : "Collider";
        d.shape.halfExtents = glm::max(collider.halfExtents * glm::abs(t.scale), glm::vec3(0.005f));
        d.position = t.position + t.rotation * (collider.offset * t.scale);
        d.rotation = t.rotation;
        d.motion = physics::MotionType::Static;
        d.friction = 0.8f;
        d.restitution = kRestitution;
        m_world->addBody(d);

        glm::vec3 lo(1e9f);
        glm::vec3 hi(-1e9f);
        const glm::mat3 r = glm::mat3_cast(t.rotation);
        for (int i = 0; i < 8; ++i) {
            const glm::vec3 c(i & 1 ? 1.0f : -1.0f, i & 2 ? 1.0f : -1.0f, i & 4 ? 1.0f : -1.0f);
            const glm::vec3 p = d.position + r * (c * d.shape.halfExtents);
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        m_obstacles.emplace_back(lo, hi);
    }

    m_characters.clear();
    u32 group = 100;
    for (const entt::entity e : registry.view<scene::CharacterComponent>()) {
        auto character = std::make_unique<PlayCharacter>();
        if (character->spawn(*m_world, scene, e, group++)) {
            m_characters.push_back(std::move(character));
        }
    }
    if (!m_characters.empty()) {
        m_cameraYaw = m_characters.front()->yaw();
    }
    m_accumulator = 0.0f;
    m_active = true;
    update(0.0f, PlayInput{.cameraYaw = m_cameraYaw}, scene);
}

void PlayMode::stop(scene::Scene& scene)
{
    if (!m_active) {
        return;
    }
    auto& registry = scene.registry();
    for (const auto& [entity, snapshot] : m_snapshot) {
        if (registry.valid(entity) && registry.all_of<scene::TransformComponent>(entity)) {
            registry.get<scene::TransformComponent>(entity) = snapshot.transform;
        }
    }
    for (const auto& character : m_characters) {
        if (registry.valid(character->entity())) {
            if (auto* animator = registry.try_get<scene::AnimatorComponent>(character->entity())) {
                animator->externalPose = false;
            }
        }
    }
    m_characters.clear();
    m_world.reset();
    m_snapshot.clear();
    m_active = false;
}

void PlayMode::update(f32 dt, const PlayInput& input, scene::Scene& scene)
{
    if (!m_active) {
        return;
    }
    const f32 h = 1.0f / m_stepHz;
    m_accumulator += std::min(dt, 0.1f);
    PlayInput stepInput = input;
    int steps = 0;
    while (m_accumulator >= h && steps < 200) {
        for (const auto& character : m_characters) {
            character->step(h, *m_world, stepInput, scene, m_obstacles);
        }
        m_world->step(h);
        for (const auto& character : m_characters) {
            character->afterStep(*m_world);
        }
        stepInput.attack = false; // edge-triggered: only the first step of the frame
        stepInput.toggleRagdoll = false;
        m_accumulator -= h;
        ++steps;
    }
    for (const auto& character : m_characters) {
        character->writePose(scene, *m_world);
    }
}

void PlayMode::updateCamera(scene::Scene& scene, const glm::vec2& orbitDelta, f32 wheel)
{
    if (!m_active || m_camera == entt::null || !scene.registry().valid(m_camera) || m_characters.empty()) {
        return;
    }
    m_cameraYaw -= orbitDelta.x * 0.006f;
    m_cameraPitch = std::clamp(m_cameraPitch + orbitDelta.y * 0.006f, -0.4f, 1.3f);
    m_cameraDistance = std::clamp(m_cameraDistance * std::pow(0.9f, wheel), 1.2f, 15.0f);

    const PlayCharacter& c = *m_characters.front();
    glm::vec3 target = c.rootPosition() + glm::vec3(0.0f, 1.4f, 0.0f);
    if (c.mode() == PlayCharacter::Mode::Ragdoll) {
        target = m_world->body(c.body(character::kPelvis)).position + glm::vec3(0.0f, 0.3f, 0.0f);
    }
    const glm::vec3 look(std::sin(m_cameraYaw) * std::cos(m_cameraPitch), -std::sin(m_cameraPitch),
                         std::cos(m_cameraYaw) * std::cos(m_cameraPitch));
    auto& t = scene.registry().get<scene::TransformComponent>(m_camera);
    t.position = target - look * m_cameraDistance;
    t.rotation = scene::lookAtRotation(t.position, target);
}

void PlayMode::appendDebugLines(std::vector<renderer::DebugVertex>& out) const
{
    if (!m_active || !showDebug) {
        return;
    }
    for (const auto& character : m_characters) {
        character->appendDebugLines(out, *m_world, showShapes);
    }
}

void PlayMode::drawHud(ImDrawList* draw, const ImVec2& p0, const ImVec2& /*size*/) const
{
    if (!m_active) {
        return;
    }
    std::vector<std::string> lines;
    lines.emplace_back("PLAY   WASD move   Shift run   LMB attack (physics)   R ragdoll   RMB drag camera   Esc stop");
    if (m_characters.empty()) {
        lines.emplace_back("No character in the scene: drop the Character preset from Assets (and a Floor).");
    } else {
        const PlayCharacter& c = *m_characters.front();
        const char* mode = c.mode() == PlayCharacter::Mode::Locomotion ? "walking (regular animation)"
                         : c.mode() == PlayCharacter::Mode::Attack     ? "attack (physics animation)"
                                                                       : "ragdoll (physics)";
        char text[160];
        std::snprintf(text, sizeof(text), "%s   speed %.1f m/s", mode, static_cast<double>(c.speed()));
        lines.emplace_back(text);
        if (c.mode() == PlayCharacter::Mode::Attack) {
            const char* phase = c.attackPhase() == PlayCharacter::AttackPhase::Swing    ? "swing"
                              : c.attackPhase() == PlayCharacter::AttackPhase::Reflex ? "REFLEX"
                                                                                      : "RECOVER";
            std::snprintf(text, sizeof(text), "attack phase: %s", phase);
            lines.emplace_back(text);
        }
        if (!c.hits().empty()) {
            const auto& hit = c.hits().back();
            std::snprintf(text, sizeof(text), "last hit: %.2f N s   muscle load  shoulder %.0f%%  elbow %.0f%%  wrist %.0f%%",
                          static_cast<double>(hit.impulse), static_cast<double>(hit.peakLoad[0] * 100.0f),
                          static_cast<double>(hit.peakLoad[1] * 100.0f), static_cast<double>(hit.peakLoad[2] * 100.0f));
            lines.emplace_back(text);
        }
    }
    const f32 lineHeight = ImGui::GetTextLineHeightWithSpacing();
    f32 width = 0.0f;
    for (const auto& l : lines) {
        width = std::max(width, ImGui::CalcTextSize(l.c_str()).x);
    }
    const ImVec2 a(p0.x + 10.0f, p0.y + 10.0f);
    draw->AddRectFilled(a, ImVec2(a.x + width + 16.0f, a.y + lineHeight * static_cast<f32>(lines.size()) + 10.0f),
                        IM_COL32(12, 14, 18, 200), 4.0f);
    for (usize i = 0; i < lines.size(); ++i) {
        draw->AddText(ImVec2(a.x + 8.0f, a.y + 5.0f + lineHeight * static_cast<f32>(i)),
                      i == 0 ? IM_COL32(120, 220, 140, 255) : IM_COL32(220, 222, 228, 255), lines[i].c_str());
    }
}

} // namespace kinetic::editor
