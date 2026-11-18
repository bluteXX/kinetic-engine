#include "SwordStrikeTest.h"

#include <kinetic/animation/PoseOps.h>
#include <kinetic/core/Primitives.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kinetic::editor {

namespace {

constexpr glm::vec3 kShoulder{0.0f, 1.45f, 0.0f};
constexpr glm::vec3 kAxis{0.0f, 0.0f, 1.0f}; // every hinge: the swing happens in the XY plane
constexpr f32 kUpperArm = 0.30f;
constexpr f32 kForearm = 0.27f;
constexpr f32 kWristToGrip = 0.06f;
constexpr f32 kGripToGuard = 0.10f;
constexpr f32 kGripToTip = 1.00f;
constexpr u32 kRigGroup = 1;

// Segment direction for an absolute angle about +Z, measured from straight down.
glm::vec3 direction(f32 angle) { return {std::sin(angle), -std::cos(angle), 0.0f}; }
glm::quat rotationZ(f32 angle) { return glm::angleAxis(angle, kAxis); }
f32 angleAboutZ(const glm::quat& q) { return 2.0f * std::atan2(q.z, q.w); }

glm::vec3 loadColor(f32 load)
{
    load = std::clamp(load, 0.0f, 1.0f);
    return load < 0.5f ? glm::mix(glm::vec3(0.25f, 0.85f, 0.35f), glm::vec3(1.0f, 0.85f, 0.2f), load * 2.0f)
                       : glm::mix(glm::vec3(1.0f, 0.85f, 0.2f), glm::vec3(1.0f, 0.2f, 0.15f), load * 2.0f - 1.0f);
}

ImU32 toImColor(const glm::vec3& c) { return ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, 1.0f)); }

MeshData merge(std::initializer_list<MeshData> parts)
{
    MeshData out;
    for (const MeshData& part : parts) {
        const auto base = static_cast<u32>(out.positions.size());
        out.positions.insert(out.positions.end(), part.positions.begin(), part.positions.end());
        out.normals.insert(out.normals.end(), part.normals.begin(), part.normals.end());
        for (const u32 i : part.indices) {
            out.indices.push_back(base + i);
        }
    }
    return out;
}

// --- Debug line helpers ------------------------------------------------------------------------------

using Lines = std::vector<renderer::DebugVertex>;

void line(Lines& out, const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
{
    out.push_back({a, color});
    out.push_back({b, color});
}

void cross(Lines& out, const glm::vec3& p, f32 size, const glm::vec3& color)
{
    for (int axis = 0; axis < 3; ++axis) {
        glm::vec3 d(0.0f);
        d[axis] = size;
        line(out, p - d, p + d, color);
    }
}

glm::vec3 anyPerpendicular(const glm::vec3& d)
{
    const glm::vec3 c = glm::cross(d, kAxis);
    return glm::dot(c, c) > 1e-8f ? glm::normalize(c) : glm::normalize(glm::cross(d, glm::vec3(0, 1, 0)));
}

void arrow(Lines& out, const glm::vec3& from, const glm::vec3& vector, const glm::vec3& color)
{
    const f32 length = glm::length(vector);
    if (length < 1e-5f) {
        return;
    }
    const glm::vec3 tip = from + vector;
    const glm::vec3 d = vector / length;
    const glm::vec3 side = anyPerpendicular(d);
    const f32 head = std::min(0.05f, length * 0.3f);
    line(out, from, tip, color);
    line(out, tip, tip - d * head + side * head * 0.5f, color);
    line(out, tip, tip - d * head - side * head * 0.5f, color);
}

// Arc around `axis` through `start` (perpendicular to the axis), sweeping `angle` radians, with an
// arrowhead showing the direction of rotation.
void arc(Lines& out, const glm::vec3& center, const glm::vec3& axis, const glm::vec3& start, f32 radius, f32 angle,
         const glm::vec3& color)
{
    const int segments = std::max(4, static_cast<int>(std::abs(angle) / 0.15f));
    glm::vec3 previous = center + start * radius;
    for (int i = 1; i <= segments; ++i) {
        const f32 a = angle * static_cast<f32>(i) / static_cast<f32>(segments);
        const glm::vec3 p = center + glm::angleAxis(a, axis) * start * radius;
        line(out, previous, p, color);
        previous = p;
    }
    if (std::abs(angle) > 0.05f) {
        const glm::vec3 radial = glm::normalize(previous - center);
        const glm::vec3 tangent = glm::cross(axis, radial) * (angle > 0.0f ? 1.0f : -1.0f);
        const f32 head = radius * 0.35f;
        line(out, previous, previous - tangent * head + radial * head * 0.5f, color);
        line(out, previous, previous - tangent * head - radial * head * 0.5f, color);
    }
}

void circle(Lines& out, const glm::vec3& center, const glm::vec3& u, const glm::vec3& v, f32 radius, const glm::vec3& color,
            f32 from = 0.0f, f32 to = 6.2831853f)
{
    constexpr int kSegments = 16;
    glm::vec3 previous = center + (u * std::cos(from) + v * std::sin(from)) * radius;
    for (int i = 1; i <= kSegments; ++i) {
        const f32 a = from + (to - from) * static_cast<f32>(i) / kSegments;
        const glm::vec3 p = center + (u * std::cos(a) + v * std::sin(a)) * radius;
        line(out, previous, p, color);
        previous = p;
    }
}

void shapeWireframe(Lines& out, const physics::BodyDesc& desc, const physics::BodyState& state, const glm::vec3& color)
{
    const glm::mat3 r = glm::mat3_cast(state.rotation);
    const glm::vec3 c = state.position + r * desc.shape.offset;
    const glm::vec3 x = r[0];
    const glm::vec3 y = r[1];
    const glm::vec3 z = r[2];
    switch (desc.shape.type) {
    case physics::ShapeType::Box: {
        const glm::vec3 h = desc.shape.halfExtents;
        const auto corner = [&](int i) {
            return c + x * (i & 1 ? h.x : -h.x) + y * (i & 2 ? h.y : -h.y) + z * (i & 4 ? h.z : -h.z);
        };
        for (int a = 0; a < 8; ++a) {
            for (int bit = 1; bit < 8; bit <<= 1) {
                if (!(a & bit)) {
                    line(out, corner(a), corner(a | bit), color);
                }
            }
        }
        break;
    }
    case physics::ShapeType::Sphere:
        circle(out, c, x, y, desc.shape.radius, color);
        circle(out, c, y, z, desc.shape.radius, color);
        circle(out, c, z, x, desc.shape.radius, color);
        break;
    case physics::ShapeType::Capsule: {
        const f32 rad = desc.shape.radius;
        const f32 hh = desc.shape.halfHeight;
        const glm::vec3 top = c + y * hh;
        const glm::vec3 bottom = c - y * hh;
        circle(out, top, x, z, rad, color);
        circle(out, bottom, x, z, rad, color);
        for (const glm::vec3& side : {x, -x, z, -z}) {
            line(out, top + side * rad, bottom + side * rad, color);
        }
        const f32 pi = 3.14159265f;
        circle(out, top, x, y, rad, color, 0.0f, pi);
        circle(out, top, z, y, rad, color, 0.0f, pi);
        circle(out, bottom, x, y, rad, color, pi, 2.0f * pi);
        circle(out, bottom, z, y, rad, color, pi, 2.0f * pi);
        break;
    }
    }
}

void tooltip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

} // namespace

SwordStrikeTest::SwordStrikeTest()
{
    // Strength from biomechanics (flexion, adult): shoulder ~90-100, elbow ~70, wrist ~10-15 N m.
    m_muscles[0] = {.stiffness = 900.0f, .damping = 60.0f, .maxTorque = 95.0f};
    m_muscles[1] = {.stiffness = 450.0f, .damping = 30.0f, .maxTorque = 70.0f};
    m_muscles[2] = {.stiffness = 120.0f, .damping = 6.0f, .maxTorque = 15.0f};
    for (usize i = 0; i < 3; ++i) {
        m_torqueHistory[i].assign(kHistory, 0.0f);
        m_loadHistory[i].assign(kHistory, 0.0f);
    }
    m_contactHistory.assign(kHistory, 0.0f);
    buildClip();
}

SwordStrikeTest::~SwordStrikeTest() = default;

// --- Setup -----------------------------------------------------------------------------------------------

void SwordStrikeTest::buildClip()
{
    m_skeleton = anim::Skeleton({"Shoulder", "Elbow", "Wrist"}, {anim::kNoParent, 0, 1}, anim::Pose(3));

    // Overhead strike, joint angles in degrees (shoulder from straight down, elbow / wrist relative).
    struct Key {
        f32 time;
        f32 shoulder, elbow, wrist;
    };
    const Key keys[] = {
        {0.00f, 45.0f, 75.0f, 25.0f},   // guard: sword up and forward
        {0.45f, 160.0f, 105.0f, 45.0f}, // wind-up: blade cocked behind the head
        {0.62f, 150.0f, 60.0f, 20.0f},  // acceleration
        {0.78f, 95.0f, 15.0f, -5.0f},   // strike
        {0.95f, 40.0f, 10.0f, -15.0f},  // follow-through
        {1.50f, 45.0f, 75.0f, 25.0f},   // back to guard
        {2.00f, 45.0f, 75.0f, 25.0f},   // hold
    };
    std::vector<anim::JointTracks> tracks(3);
    for (const Key& k : keys) {
        const f32 angles[3] = {k.shoulder, k.elbow, k.wrist};
        for (usize j = 0; j < 3; ++j) {
            tracks[j].rotation.times.push_back(k.time);
            tracks[j].rotation.values.push_back(rotationZ(glm::radians(angles[j])));
        }
    }
    m_clips = {anim::AnimationClip("Overhead Strike", 2.0f, std::move(tracks))};
}

std::array<f32, 3> SwordStrikeTest::clipAngles(f32 time) const
{
    anim::Pose pose = m_skeleton.restPose();
    m_clips[0].sample(time, false, pose);
    return {angleAboutZ(pose.rotations[0]), angleAboutZ(pose.rotations[1]), angleAboutZ(pose.rotations[2])};
}

SwordStrikeTest::ArmPose SwordStrikeTest::forwardKinematics(const std::array<f32, 3>& angles) const
{
    const f32 a1 = angles[0];
    const f32 a2 = a1 + angles[1];
    const f32 a3 = a2 + angles[2];
    ArmPose p{};
    p.shoulder = kShoulder;
    p.elbow = p.shoulder + direction(a1) * kUpperArm;
    p.wrist = p.elbow + direction(a2) * kForearm;
    p.grip = p.wrist + direction(a3) * kWristToGrip;
    p.guard = p.grip + direction(a3) * kGripToGuard;
    p.tip = p.grip + direction(a3) * kGripToTip;
    return p;
}

void SwordStrikeTest::placeStone()
{
    // Follow the blade through the strike; put the stone's top where 70 % of the blade passes
    // 70 degrees below the vertical, so the edge comes down onto it.
    for (f32 t = 0.6f; t < 1.0f; t += 0.002f) {
        const auto angles = clipAngles(t);
        if (angles[0] + angles[1] + angles[2] <= glm::radians(70.0f)) {
            const ArmPose pose = forwardKinematics(angles);
            const glm::vec3 point = glm::mix(pose.guard, pose.tip, 0.7f);
            const f32 top = std::max(point.y - 0.005f, 0.15f);
            m_stoneHalf = {0.18f, top * 0.5f, 0.25f};
            m_stoneCenter = {point.x, top * 0.5f, 0.0f};
            return;
        }
    }
}

void SwordStrikeTest::buildWorld()
{
    m_world.clear();
    m_world.setGravity({0.0f, -9.81f, 0.0f});

    const auto start = clipAngles(0.0f);
    const ArmPose pose = forwardKinematics(start);
    const f32 a1 = start[0];
    const f32 a2 = a1 + start[1];
    const f32 a3 = a2 + start[2];

    physics::BodyDesc torso;
    torso.name = "Torso";
    torso.shape.halfExtents = {0.16f, 0.32f, 0.10f};
    torso.position = {0.0f, 1.2f, -0.25f};
    torso.motion = physics::MotionType::Static;
    torso.collisionGroup = kRigGroup;
    m_torso = m_world.addBody(torso);

    // Segments: body origin at the proximal joint, capsule hanging along local -Y.
    const auto segment = [&](const char* name, const glm::vec3& origin, f32 angle, f32 length, f32 radius, f32 mass) {
        physics::BodyDesc d;
        d.name = name;
        d.shape.type = physics::ShapeType::Capsule;
        d.shape.radius = radius;
        d.shape.halfHeight = length * 0.5f - radius;
        d.shape.offset = {0.0f, -length * 0.5f, 0.0f};
        d.position = origin;
        d.rotation = rotationZ(angle);
        d.mass = mass;
        d.collisionGroup = kRigGroup;
        return m_world.addBody(d);
    };
    // Segment masses for a 75 kg adult (anthropometric tables: 2.8 %, 1.6 %, 0.6 %).
    m_upperArm = segment("Upper arm", pose.shoulder, a1, kUpperArm, 0.045f, 2.1f);
    m_forearm = segment("Forearm", pose.elbow, a2, kForearm, 0.038f, 1.25f);

    physics::BodyDesc hand;
    hand.name = "Hand";
    hand.shape.halfExtents = {0.04f, 0.045f, 0.02f};
    hand.shape.offset = {0.0f, -0.045f, 0.0f};
    hand.position = pose.wrist;
    hand.rotation = rotationZ(a3);
    hand.mass = 0.45f;
    hand.collisionGroup = kRigGroup;
    m_hand = m_world.addBody(hand);

    // Longsword: 1.3 kg, 0.9 m blade; balance point ~12 cm in front of the guard.
    physics::BodyDesc sword;
    sword.name = "Sword";
    sword.shape.halfExtents = {0.022f, 0.45f, 0.004f};
    sword.shape.offset = {0.0f, -(kGripToGuard + 0.45f), 0.0f};
    sword.centerOfMass = {0.0f, 0.45f - 0.12f, 0.0f}; // from the blade centre back towards the guard
    sword.position = pose.grip;
    sword.rotation = rotationZ(a3);
    sword.mass = 1.3f;
    sword.friction = 0.4f;
    sword.restitution = 0.1f;
    sword.continuousCollision = true;
    sword.collisionGroup = kRigGroup;
    m_sword = m_world.addBody(sword);

    physics::BodyDesc stone;
    stone.name = "Stone";
    stone.shape.halfExtents = m_stoneHalf;
    stone.position = m_stoneCenter;
    stone.motion = physics::MotionType::Static;
    stone.friction = 0.7f;
    m_stone = m_world.addBody(stone);

    physics::BodyDesc floor;
    floor.name = "Floor";
    floor.shape.halfExtents = {20.0f, 0.5f, 20.0f};
    floor.position = {0.0f, -0.5f, 0.0f};
    floor.motion = physics::MotionType::Static;
    m_floor = m_world.addBody(floor);

    const auto hinge = [&](const char* name, physics::BodyId parent, physics::BodyId child, const glm::vec3& anchor,
                           f32 parentAngle, f32 childAngle, f32 minDeg, f32 maxDeg, const physics::Muscle& muscle) {
        physics::HingeDesc h;
        h.name = name;
        h.parent = parent;
        h.child = child;
        h.anchor = anchor;
        h.axis = kAxis;
        h.referenceParent = direction(parentAngle);
        h.referenceChild = direction(childAngle);
        h.limitMin = glm::radians(minDeg);
        h.limitMax = glm::radians(maxDeg);
        h.muscle = muscle;
        return m_world.addHinge(h);
    };
    m_joints[0] = {hinge("Shoulder", m_torso, m_upperArm, pose.shoulder, 0.0f, a1, -60.0f, 180.0f, m_muscles[0]), "Shoulder", true};
    m_joints[1] = {hinge("Elbow", m_upperArm, m_forearm, pose.elbow, a1, a2, -5.0f, 150.0f, m_muscles[1]), "Elbow", true};
    m_joints[2] = {hinge("Wrist", m_forearm, m_hand, pose.wrist, a2, a3, -70.0f, 80.0f, m_muscles[2]), "Wrist", true};

    physics::FixedDesc grip;
    grip.name = "Grip";
    grip.parent = m_hand;
    grip.child = m_sword;
    grip.anchor = pose.grip;
    m_joints[3] = {m_world.addFixed(grip), "Grip", false};

    for (usize i = 0; i < 3; ++i) {
        m_world.setTargetAngle(m_joints[i].id, start[i]);
    }
    applyMaterials();
    m_phase = Phase::Swing;
    m_phaseStart = 0.0f;
}

void SwordStrikeTest::create(scene::Scene& scene, renderer::Renderer& renderer)
{
    if (m_active) {
        return;
    }
    if (!m_world.init()) {
        return;
    }
    placeStone();
    buildWorld();

    // One entity per body with a mesh of its shape (floor excluded: the editor grid shows it).
    const struct {
        physics::BodyId body;
        glm::vec3 color;
    } visuals[] = {
        {m_torso, {0.45f, 0.48f, 0.55f}},   {m_upperArm, {0.85f, 0.65f, 0.5f}}, {m_forearm, {0.85f, 0.65f, 0.5f}},
        {m_hand, {0.85f, 0.65f, 0.5f}},     {m_sword, {0.8f, 0.82f, 0.88f}},     {m_stone, {0.55f, 0.52f, 0.48f}},
    };
    m_entities.assign(m_world.bodyCount(), entt::null);
    for (const auto& v : visuals) {
        const physics::BodyDesc& desc = m_world.bodyDesc(v.body);
        MeshData mesh;
        if (v.body == m_sword) {
            mesh = merge({makeBoxMesh(desc.shape.halfExtents, desc.shape.offset),
                          makeBoxMesh({0.09f, 0.012f, 0.015f}, {0.0f, -kGripToGuard, 0.0f}),        // cross-guard
                          makeCapsuleMesh(0.016f, 0.045f, {0.0f, -kGripToGuard * 0.4f, 0.0f}, 10, 3)}); // grip
        } else if (desc.shape.type == physics::ShapeType::Capsule) {
            mesh = makeCapsuleMesh(desc.shape.radius, desc.shape.halfHeight, desc.shape.offset);
        } else {
            mesh = makeBoxMesh(desc.shape.halfExtents, desc.shape.offset);
        }
        const scene::MeshId id = renderer.addMesh("Physics " + desc.name, mesh);
        m_meshes.push_back(id);
        m_entities[v.body.index] = scene.createMesh(desc.name, id, glm::vec4(v.color, 1.0f));
    }

    m_player = {};
    m_player.loop = m_loop;
    m_player.play(0);
    m_simTime = 0.0f;
    m_accumulator = 0.0f;
    resetAnalysis();
    m_trail.clear();
    m_active = true;
    update(0.0f, scene);
}

bool SwordStrikeTest::createHeadless()
{
    if (m_active || !m_world.init()) {
        return m_active;
    }
    placeStone();
    buildWorld();
    m_entities.assign(m_world.bodyCount(), entt::null);
    m_player = {};
    m_player.loop = m_loop;
    m_player.play(0);
    m_simTime = 0.0f;
    resetAnalysis();
    m_active = true;
    return true;
}

void SwordStrikeTest::advance(f32 seconds)
{
    const f32 h = 1.0f / m_stepHz;
    for (int i = 0; i < static_cast<int>(seconds / h + 0.5f); ++i) {
        simulateStep(h);
    }
}

void SwordStrikeTest::destroy(scene::Scene& scene, renderer::Renderer& renderer)
{
    if (!m_active) {
        return;
    }
    for (const entt::entity e : m_entities) {
        if (e != entt::null && scene.registry().valid(e)) {
            scene.destroyEntity(e);
        }
    }
    for (const scene::MeshId id : m_meshes) {
        renderer.removeMesh(id);
    }
    m_entities.clear();
    m_meshes.clear();
    m_world.clear();
    m_active = false;
}

// --- Simulation ------------------------------------------------------------------------------------------

void SwordStrikeTest::resetAnalysis()
{
    m_inHit = false;
    m_hits.clear();
    m_selectedHit = -1;
}

const SwordStrikeTest::HitRecord* SwordStrikeTest::strongestHit() const
{
    const HitRecord* best = nullptr;
    for (const HitRecord& hit : m_hits) {
        if (!best || glm::length(hit.impulse) > glm::length(best->impulse)) {
            best = &hit;
        }
    }
    return best;
}

f32 SwordStrikeTest::jToFEstimate(usize hinge) const
{
    // Quasi-static: the torque the joint must supply so the contact force does not rotate the
    // chain beyond it = -(r x F) . axis, with r from the joint to the contact point.
    const physics::BodyState& sword = m_world.body(m_sword);
    if (!sword.inContact) {
        return 0.0f;
    }
    const physics::JointState& j = m_world.joint(m_joints[hinge].id);
    return -glm::dot(j.axis, glm::cross(sword.contactPoint - j.anchor, sword.contactForce));
}

void SwordStrikeTest::applyMaterials()
{
    m_world.setRestitution(m_sword, m_reaction.restitution);
    m_world.setRestitution(m_stone, m_reaction.restitution);
}

void SwordStrikeTest::updateTargets(f32 dt)
{
    m_player.loop = m_loop;
    m_player.update(dt, m_clips);
    const std::array<f32, 3> animation = clipAngles(m_player.time());
    const f32 sincePhase = m_simTime - m_phaseStart;

    switch (m_phase) {
    case Phase::Swing:
    case Phase::Reflex:
        // Before the reflex kicks in the muscles keep following the swing they were executing.
        m_targets = animation;
        if (m_phase == Phase::Reflex && sincePhase >= m_reaction.reflexMs * 0.001f) {
            m_phase = Phase::Recover;
            m_phaseStart = m_simTime;
            m_player.setTime(m_reaction.recoveryClipTime, m_clips); // skip the follow-through
            m_player.playing = true;
        }
        break;
    case Phase::Recover: {
        // Target = current angle + w * (animation - current): with w < 1 the muscles only partly
        // pull towards the animation, so the arm keeps the rebound's momentum and is caught softly.
        const f32 x = std::clamp(sincePhase / std::max(m_reaction.recoverTime, 1e-3f), 0.0f, 1.0f);
        const f32 smooth = x * x * (3.0f - 2.0f * x);
        const f32 w = m_reaction.hold + (1.0f - m_reaction.hold) * smooth;
        for (usize i = 0; i < 3; ++i) {
            const f32 current = m_world.joint(m_joints[i].id).angle;
            m_targets[i] = current + w * (animation[i] - current);
        }
        if (x >= 1.0f) {
            m_phase = Phase::Swing;
        }
        break;
    }
    }
    for (usize i = 0; i < 3; ++i) {
        m_world.setTargetAngle(m_joints[i].id, m_targets[i]);
    }
}

void SwordStrikeTest::simulateStep(f32 dt)
{
    updateTargets(dt);
    m_world.step(dt);
    m_simTime += dt;

    // History.
    m_historyHead = (m_historyHead + 1) % kHistory;
    for (usize i = 0; i < 3; ++i) {
        const physics::JointState& s = m_world.joint(m_joints[i].id);
        m_torqueHistory[i][m_historyHead] = s.muscleTorque;
        m_loadHistory[i][m_historyHead] = std::abs(s.muscleTorque) / std::max(m_muscles[i].maxTorque, 1e-3f);
    }
    const physics::BodyState& sword = m_world.body(m_sword);
    const f32 contact = sword.inContact ? glm::length(sword.contactForce) : 0.0f;
    m_contactHistory[m_historyHead] = contact;

    // Blade trail.
    const glm::vec3 tip = sword.position + sword.rotation * glm::vec3(0.0f, -kGripToTip, 0.0f);
    const f32 speed = glm::length(sword.linearVelocity + glm::cross(sword.angularVelocity, tip - sword.centerOfMass));
    m_trail.emplace_back(tip, speed);
    while (m_trail.size() > static_cast<usize>(m_stepHz * 0.6f)) {
        m_trail.pop_front();
    }

    // Hit analysis: a hit starts with the first contact and ends 150 ms after the last one, so the
    // muscles' (delayed) reaction to the impulse is part of the hit. Contacts within that time
    // belong to the same hit.
    static constexpr f32 kContactThreshold = 1.0f; // N
    static constexpr f32 kHitTail = 0.15f;         // s
    const bool touching = contact > kContactThreshold;
    if (touching && m_phase == Phase::Swing && m_reaction.enabled) {
        m_phase = Phase::Reflex; // the reaction starts at the impact
        m_phaseStart = m_simTime;
    }
    if (touching && !m_inHit) {
        if (m_hits.size() >= 64) {
            m_hits.erase(m_hits.begin());
        }
        m_hits.push_back(HitRecord{.start = m_simTime});
        m_inHit = true;
        if (m_freezeOnHit) {
            m_paused = true;
        }
    }
    if (!m_inHit) {
        return;
    }
    HitRecord& hit = m_hits.back();
    if (touching) {
        hit.duration = m_simTime - hit.start;
        hit.impulse += sword.contactForce * dt;
        hit.peakContact = std::max(hit.peakContact, contact);
    } else if (m_simTime - (hit.start + hit.duration) > kHitTail) {
        m_inHit = false;
        return;
    }
    for (usize i = 0; i < 4; ++i) {
        const physics::JointState& s = m_world.joint(m_joints[i].id);
        HitStats& h = hit.joints[i];
        h.peakForce = std::max(h.peakForce, glm::length(s.force));
        h.impulse += s.force * dt;
        if (m_joints[i].hinge) {
            const f32 load = std::abs(s.muscleTorque) / std::max(m_muscles[i].maxTorque, 1e-3f);
            h.peakTorque = std::max(h.peakTorque, std::abs(s.muscleTorque));
            h.peakLoad = std::max(h.peakLoad, load);
            h.angularImpulse += s.muscleTorque * dt;
            h.absorbedEnergy += std::max(0.0f, -s.muscleTorque * s.angularVelocity) * dt;
        } else {
            h.peakTorque = std::max(h.peakTorque, glm::length(s.torque));
        }
    }
}

void SwordStrikeTest::update(f32 frameDt, scene::Scene& scene)
{
    if (!m_active) {
        return;
    }
    const f32 h = 1.0f / m_stepHz;
    if (m_stepOnce) {
        m_stepOnce = false;
        simulateStep(h);
    } else if (!m_paused) {
        m_accumulator += std::min(frameDt, 0.1f) * m_timeScale;
        int steps = 0;
        while (m_accumulator >= h && steps < 256 && !m_paused) {
            simulateStep(h);
            m_accumulator -= h;
            ++steps;
        }
        if (m_paused) {
            m_accumulator = 0.0f; // froze on a hit
        }
    }

    auto& registry = scene.registry();
    for (usize i = 0; i < m_entities.size(); ++i) {
        const entt::entity e = m_entities[i];
        if (e == entt::null || !registry.valid(e)) {
            continue;
        }
        const physics::BodyState& s = m_world.body(physics::BodyId{static_cast<u32>(i)});
        auto& transform = registry.get<scene::TransformComponent>(e);
        transform.position = s.position;
        transform.rotation = s.rotation;
    }
}

// --- Debug drawing ----------------------------------------------------------------------------------------

void SwordStrikeTest::appendDebugLines(std::vector<renderer::DebugVertex>& out) const
{
    if (!m_active) {
        return;
    }
    const DebugOptions& o = m_debug;
    const glm::vec3 g = m_world.gravity();

    for (usize i = 0; i < m_world.bodyCount(); ++i) {
        const physics::BodyId id{static_cast<u32>(i)};
        if (id == m_floor) {
            continue;
        }
        const physics::BodyDesc& desc = m_world.bodyDesc(id);
        const physics::BodyState& s = m_world.body(id);
        if (o.shapes) {
            shapeWireframe(out, desc, s, desc.motion == physics::MotionType::Static ? glm::vec3(0.45f) : glm::vec3(0.75f, 0.8f, 0.9f));
        }
        if (desc.motion != physics::MotionType::Dynamic) {
            continue;
        }
        if (o.centerOfMass) {
            cross(out, s.centerOfMass, 0.02f, glm::vec3(1.0f));
        }
        if (o.velocities) {
            arrow(out, s.centerOfMass, s.linearVelocity * o.velocityScale, {0.3f, 0.9f, 1.0f});
        }
        if (o.gravity) {
            arrow(out, s.centerOfMass, g * s.mass * o.forceScale, {0.6f, 0.6f, 0.65f});
        }
    }

    for (usize i = 0; i < 4; ++i) {
        const physics::JointState& s = m_world.joint(m_joints[i].id);
        if (o.jointAxes) {
            cross(out, s.anchor, 0.015f, {0.4f, 0.6f, 1.0f});
            if (m_joints[i].hinge) {
                line(out, s.anchor - s.axis * 0.08f, s.anchor + s.axis * 0.08f, {0.4f, 0.6f, 1.0f});
            }
        }
        if (m_joints[i].hinge) {
            if (o.jointForces) {
                arrow(out, s.anchor, s.force * o.forceScale, {1.0f, 0.55f, 0.15f});
            }
            if (o.muscleTorques) {
                // Arc around the joint: sweep = load (full strength = 270 degrees), colour = load.
                const f32 load = s.muscleTorque / std::max(m_muscles[i].maxTorque, 1e-3f);
                // Start the arc along the child segment so it reads as "pulling that bone".
                const physics::BodyId childBody = i == 0 ? m_upperArm : i == 1 ? m_forearm : m_hand;
                const glm::vec3 start = m_world.body(childBody).rotation * glm::vec3(0.0f, -1.0f, 0.0f);
                arc(out, s.anchor, s.axis, start, 0.075f, std::clamp(load, -1.0f, 1.0f) * glm::radians(270.0f), loadColor(std::abs(load)));
                arc(out, s.anchor, s.axis, start, 0.082f, std::clamp(load, -1.0f, 1.0f) * glm::radians(270.0f), loadColor(std::abs(load)));
            }
        } else if (o.gripForce) {
            arrow(out, s.anchor, s.force * o.forceScale, {1.0f, 0.55f, 0.15f});
            arrow(out, s.anchor, s.torque * 0.01f, {0.95f, 0.35f, 0.95f}); // torque vector, 1 cm per N m
        }
    }

    if (o.contactForces) {
        for (const physics::ContactPoint& c : m_world.contacts()) {
            cross(out, c.position, 0.012f, {1.0f, 0.95f, 0.3f});
        }
        const physics::BodyState& sword = m_world.body(m_sword);
        if (sword.inContact) {
            arrow(out, sword.contactPoint, sword.contactForce * o.forceScale, {1.0f, 0.2f, 0.2f});
        }
    }

    if (o.targetPose) {
        const ArmPose p = forwardKinematics(m_targets);
        const glm::vec3 ghost{0.35f, 0.55f, 1.0f};
        line(out, p.shoulder, p.elbow, ghost);
        line(out, p.elbow, p.wrist, ghost);
        line(out, p.wrist, p.grip, ghost);
        line(out, p.grip, p.tip, ghost);
        for (const glm::vec3& joint : {p.shoulder, p.elbow, p.wrist}) {
            cross(out, joint, 0.01f, ghost);
        }
    }

    if (o.bladeTrail && m_trail.size() > 1) {
        for (usize i = 1; i < m_trail.size(); ++i) {
            const f32 t = std::clamp(m_trail[i].second / 30.0f, 0.0f, 1.0f); // white at 30 m/s
            line(out, m_trail[i - 1].first, m_trail[i].first, glm::mix(glm::vec3(0.4f, 0.3f, 0.9f), glm::vec3(1.0f), t));
        }
    }
}

void SwordStrikeTest::drawViewportLabels(ImDrawList* draw, const glm::mat4& viewProjection, const ImVec2& p0,
                                         const ImVec2& size) const
{
    if (!m_active || !m_debug.labels) {
        return;
    }
    const auto project = [&](const glm::vec3& p, ImVec2& out) {
        const glm::vec4 clip = viewProjection * glm::vec4(p, 1.0f);
        if (clip.w <= 1e-4f) {
            return false;
        }
        const glm::vec3 ndc = glm::vec3(clip) / clip.w;
        out = ImVec2(p0.x + (ndc.x * 0.5f + 0.5f) * size.x, p0.y + (0.5f - ndc.y * 0.5f) * size.y);
        return true;
    };
    const auto label = [&](const glm::vec3& at, const char* text, ImU32 color) {
        ImVec2 s;
        if (project(at, s)) {
            const ImVec2 pos(s.x + 10.0f, s.y - 8.0f);
            const ImVec2 textSize = ImGui::CalcTextSize(text);
            draw->AddRectFilled(ImVec2(pos.x - 3, pos.y - 1), ImVec2(pos.x + textSize.x + 3, pos.y + textSize.y + 1),
                                IM_COL32(15, 16, 20, 190), 3.0f);
            draw->AddText(pos, color, text);
        }
    };

    char text[96];
    for (usize i = 0; i < 3; ++i) {
        const physics::JointState& s = m_world.joint(m_joints[i].id);
        const f32 load = std::abs(s.muscleTorque) / std::max(m_muscles[i].maxTorque, 1e-3f);
        std::snprintf(text, sizeof(text), "%s %+.1f Nm  %.0f%%", m_joints[i].name, static_cast<double>(s.muscleTorque),
                      static_cast<double>(load * 100.0f));
        label(s.anchor, text, toImColor(loadColor(load)));
    }
    const physics::JointState& grip = m_world.joint(m_joints[3].id);
    std::snprintf(text, sizeof(text), "Grip %.0f N  %.1f Nm", static_cast<double>(glm::length(grip.force)),
                  static_cast<double>(glm::length(grip.torque)));
    label(grip.anchor + glm::vec3(0.0f, -0.05f, 0.0f), text, IM_COL32(255, 160, 60, 255));

    const physics::BodyState& sword = m_world.body(m_sword);
    if (sword.inContact) {
        std::snprintf(text, sizeof(text), "Contact %.0f N", static_cast<double>(glm::length(sword.contactForce)));
        label(sword.contactPoint, text, IM_COL32(255, 80, 80, 255));
    }
}

// --- UI -----------------------------------------------------------------------------------------------------

void SwordStrikeTest::drawWindow(bool* open)
{
    ImGui::SetNextWindowSize(ImVec2(560.0f, 720.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Physics", open)) {
        ImGui::End();
        return;
    }
    if (!m_active) {
        ImGui::TextDisabled("Start the test from Physics > Sword Strike Test.");
        ImGui::End();
        return;
    }

    // --- Simulation controls ---
    if (ImGui::Button(m_paused ? "Resume" : "Pause", ImVec2(70, 0))) {
        m_paused = !m_paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Step")) {
        m_paused = true;
        m_stepOnce = true;
    }
    tooltip("One physics step");
    ImGui::SameLine();
    if (ImGui::Button("Restart")) {
        buildWorld();
        m_player.play(0);
        m_simTime = 0.0f;
        m_accumulator = 0.0f;
        m_trail.clear();
        resetAnalysis();
        m_paused = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &m_loop);
    ImGui::SameLine();
    ImGui::Checkbox("Freeze on hit", &m_freezeOnHit);
    tooltip("Pause at the first contact of each strike, to inspect the vectors");

    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Time scale", &m_timeScale, 0.01f, 1.0f, "%.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    char rate[16];
    std::snprintf(rate, sizeof(rate), "%.0f Hz", static_cast<double>(m_stepHz));
    if (ImGui::BeginCombo("Step rate", rate)) {
        for (const f32 hz : {240.0f, 480.0f, 960.0f, 1920.0f}) {
            std::snprintf(rate, sizeof(rate), "%.0f Hz", static_cast<double>(hz));
            if (ImGui::Selectable(rate, hz == m_stepHz)) {
                m_stepHz = hz;
            }
        }
        ImGui::EndCombo();
    }
    tooltip("Physics steps per second. A rigid impact resolves in about one step: higher rates mean less\n"
            "penetration and a finer time resolution (impulses stay the same, per-step peaks grow).");
    const char* phaseName = m_phase == Phase::Swing ? "swing" : m_phase == Phase::Reflex ? "REFLEX" : "RECOVER";
    ImGui::TextDisabled("t = %.3f s  |  clip %.2f s  |  phase: %s", static_cast<double>(m_simTime),
                        static_cast<double>(m_player.time()), phaseName);

    // --- Live joint table ---
    ImGui::SeparatorText("Live");
    const physics::BodyState& sword = m_world.body(m_sword);
    if (ImGui::BeginTable("##live", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("Angle / target");
        ImGui::TableSetupColumn("Muscle");
        ImGui::TableSetupColumn("Load", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("Joint force");
        ImGui::TableSetupColumn("J^T F");
        ImGui::TableHeadersRow();
        for (usize i = 0; i < 3; ++i) {
            const physics::JointState& s = m_world.joint(m_joints[i].id);
            const f32 load = std::abs(s.muscleTorque) / std::max(m_muscles[i].maxTorque, 1e-3f);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(m_joints[i].name);
            ImGui::TableNextColumn();
            ImGui::Text("%.0f / %.0f deg", static_cast<double>(glm::degrees(s.angle)), static_cast<double>(glm::degrees(s.targetAngle)));
            ImGui::TableNextColumn();
            ImGui::Text("%+.1f Nm", static_cast<double>(s.muscleTorque));
            ImGui::TableNextColumn();
            const glm::vec3 c = loadColor(load);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(c.r, c.g, c.b, 1.0f));
            char overlay[16];
            std::snprintf(overlay, sizeof(overlay), "%.0f%%", static_cast<double>(load * 100.0f));
            ImGui::ProgressBar(std::min(load, 1.0f), ImVec2(-FLT_MIN, 0.0f), overlay);
            ImGui::PopStyleColor();
            ImGui::TableNextColumn();
            ImGui::Text("%.0f N", static_cast<double>(glm::length(s.force)));
            ImGui::TableNextColumn();
            if (sword.inContact) {
                ImGui::Text("%+.1f Nm", static_cast<double>(jToFEstimate(i)));
            } else {
                ImGui::TextDisabled("-");
            }
        }
        const physics::JointState& grip = m_world.joint(m_joints[3].id);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Grip");
        ImGui::TableNextColumn();
        ImGui::TextDisabled("welded");
        ImGui::TableNextColumn();
        ImGui::Text("%.1f Nm", static_cast<double>(glm::length(grip.torque)));
        ImGui::TableNextColumn();
        ImGui::TextDisabled("-");
        ImGui::TableNextColumn();
        ImGui::Text("%.0f N", static_cast<double>(glm::length(grip.force)));
        ImGui::TableNextColumn();
        ImGui::TextDisabled("-");
        ImGui::EndTable();
    }
    if (sword.inContact) {
        ImGui::Text("Blade contact: %.0f N", static_cast<double>(glm::length(sword.contactForce)));
    } else {
        ImGui::TextDisabled("Blade contact: none");
    }
    ImGui::TextDisabled("J^T F: torque each joint needs to hold the contact force (quasi-static). "
                        "Muscle vs J^T F shows how much inertia / gravity / limits take instead.");

    // --- Hits ---
    const HitRecord* strongest = strongestHit();
    ImGui::SeparatorText(m_hits.empty() ? "Hits (none yet)" : "Hits");
    if (!m_hits.empty()) {
        if (m_selectedHit >= static_cast<i32>(m_hits.size())) {
            m_selectedHit = -1;
        }
        if (ImGui::BeginTable("##hits", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0.0f, std::min(6.5f, static_cast<f32>(m_hits.size()) + 1.5f) * ImGui::GetTextLineHeightWithSpacing()))) {
            ImGui::TableSetupScrollFreeze(0, 1);
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("Time");
            ImGui::TableSetupColumn("Duration");
            ImGui::TableSetupColumn("Peak contact");
            ImGui::TableSetupColumn("Impulse");
            ImGui::TableHeadersRow();
            for (usize n = 0; n < m_hits.size(); ++n) {
                const usize k = m_hits.size() - 1 - n; // newest first
                const HitRecord& hit = m_hits[k];
                const bool shown = m_selectedHit < 0 ? k == m_hits.size() - 1 : static_cast<usize>(m_selectedHit) == k;
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                char id[32];
                std::snprintf(id, sizeof(id), "%zu%s", k + 1, &hit == strongest ? "  strongest" : "");
                if (ImGui::Selectable(id, shown, ImGuiSelectableFlags_SpanAllColumns)) {
                    m_selectedHit = static_cast<i32>(k);
                }
                ImGui::TableNextColumn();
                ImGui::Text("%.3f s", static_cast<double>(hit.start));
                ImGui::TableNextColumn();
                ImGui::Text("%.1f ms", static_cast<double>(hit.duration * 1000.0f));
                ImGui::TableNextColumn();
                ImGui::Text("%.0f N", static_cast<double>(hit.peakContact));
                ImGui::TableNextColumn();
                ImGui::Text("%.3f N s", static_cast<double>(glm::length(hit.impulse)));
            }
            ImGui::EndTable();
        }
        if (ImGui::SmallButton("Follow latest")) {
            m_selectedHit = -1;
        }
        ImGui::SameLine();
        if (strongest && ImGui::SmallButton("Show strongest")) {
            m_selectedHit = static_cast<i32>(strongest - m_hits.data());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) {
            m_hits.clear();
            m_inHit = false;
            m_selectedHit = -1;
        }

        const HitRecord& hit = m_selectedHit < 0 ? m_hits.back() : m_hits[static_cast<usize>(m_selectedHit)];
        f32 totalEnergy = 0.0f;
        for (usize i = 0; i < 3; ++i) {
            totalEnergy += hit.joints[i].absorbedEnergy;
        }
        const f32 contactImpulse = glm::length(hit.impulse);
        // Half-sine force pulse of duration T carrying impulse J peaks at pi J / (2 T).
        const f32 estimatedPeak = 3.14159265f * contactImpulse / (2.0f * m_contactTimeMs * 0.001f);
        ImGui::Text("Hit %zu: blade impulse %.3f N s", static_cast<usize>(&hit - m_hits.data()) + 1,
                    static_cast<double>(contactImpulse));
        ImGui::SameLine();
        ImGui::TextDisabled("|  peak force ~%.0f N for a", static_cast<double>(estimatedPeak));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        ImGui::SliderFloat("##contactTime", &m_contactTimeMs, 0.2f, 10.0f, "%.1f ms contact", ImGuiSliderFlags_Logarithmic);
        tooltip("Rigid bodies resolve an impact in one step, so the simulated force is impulse / dt and\n"
                "depends on the step rate. Real contacts last ~0.5-2 ms (steel on stone) to ~5-10 ms (wood):\n"
                "the peak is estimated from the impulse for that duration (half-sine pulse).");
        if (ImGui::BeginTable("##hit", 6, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Joint");
            ImGui::TableSetupColumn("Impulse");
            ImGui::TableSetupColumn("Muscle impulse");
            ImGui::TableSetupColumn("Peak load");
            ImGui::TableSetupColumn("Absorbed");
            ImGui::TableSetupColumn("Share");
            ImGui::TableHeadersRow();
            for (usize i = 0; i < 4; ++i) {
                const HitStats& h = hit.joints[i];
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(m_joints[i].name);
                ImGui::TableNextColumn();
                ImGui::Text("%.3f N s", static_cast<double>(glm::length(h.impulse)));
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Linear impulse through the joint (step independent).\nPer-step peak: %.0f N, %.1f N m",
                                      static_cast<double>(h.peakForce), static_cast<double>(h.peakTorque));
                }
                ImGui::TableNextColumn();
                if (m_joints[i].hinge) {
                    ImGui::Text("%+.3f Nms", static_cast<double>(h.angularImpulse));
                } else {
                    ImGui::TextDisabled("-");
                }
                ImGui::TableNextColumn();
                if (m_joints[i].hinge) {
                    const glm::vec3 c = loadColor(h.peakLoad);
                    ImGui::TextColored(ImVec4(c.r, c.g, c.b, 1.0f), "%.0f%%", static_cast<double>(h.peakLoad * 100.0f));
                } else {
                    ImGui::TextDisabled("-");
                }
                ImGui::TableNextColumn();
                if (m_joints[i].hinge) {
                    ImGui::Text("%.2f J", static_cast<double>(h.absorbedEnergy));
                    ImGui::TableNextColumn();
                    ImGui::Text("%.0f%%", static_cast<double>(totalEnergy > 1e-6f ? h.absorbedEnergy / totalEnergy * 100.0f : 0.0f));
                } else {
                    ImGui::TextDisabled("-");
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("-");
                }
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Impulse: momentum passed through the joint. Muscle impulse: what the muscle delivered.\n"
                            "Peak load 100%% = the muscle gave way. Absorbed: work done on the stretched muscle\n"
                            "(eccentric) during the hit + 150 ms; Share = of all three muscles.");
    }

    // --- Graphs ---
    if (ImGui::CollapsingHeader("Graphs (last 2 s)", ImGuiTreeNodeFlags_DefaultOpen)) {
        const auto offset = static_cast<int>((m_historyHead + 1) % kHistory);
        for (usize i = 0; i < 3; ++i) {
            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%s torque %+.1f Nm", m_joints[i].name,
                          static_cast<double>(m_torqueHistory[i][m_historyHead]));
            const f32 range = m_muscles[i].maxTorque * 1.1f;
            ImGui::PlotLines(("##torque" + std::to_string(i)).c_str(), m_torqueHistory[i].data(), static_cast<int>(kHistory),
                             offset, overlay, -range, range, ImVec2(-FLT_MIN, 46.0f));
        }
        char overlay[64];
        std::snprintf(overlay, sizeof(overlay), "Blade contact %.0f N", static_cast<double>(m_contactHistory[m_historyHead]));
        const f32 maxContact = std::max(100.0f, *std::max_element(m_contactHistory.begin(), m_contactHistory.end()));
        ImGui::PlotLines("##contact", m_contactHistory.data(), static_cast<int>(kHistory), offset, overlay, 0.0f,
                         maxContact * 1.1f, ImVec2(-FLT_MIN, 46.0f));
    }

    // --- Reaction ---
    if (ImGui::CollapsingHeader("Reaction to the hit", ImGuiTreeNodeFlags_DefaultOpen)) {
        Reaction& r = m_reaction;
        ImGui::Checkbox("Enabled", &r.enabled);
        tooltip("Off: the muscles keep following the animation through the stone (no reaction)");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150.0f);
        if (ImGui::SliderFloat("Restitution", &r.restitution, 0.0f, 0.9f, "%.2f")) {
            applyMaterials();
        }
        tooltip("How much the blade bounces off the stone (steel on granite ~0.3-0.5, on wood ~0.1)");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::SliderFloat("Reflex", &r.reflexMs, 0.0f, 200.0f, "%.0f ms");
        tooltip("Reaction time: until then the muscles keep pushing the swing (humans ~40-80 ms)");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150.0f);
        ImGui::SliderFloat("Recover", &r.recoverTime, 0.05f, 1.5f, "%.2f s");
        tooltip("Time until the arm follows the animation again");
        ImGui::SetNextItemWidth(150.0f);
        ImGui::SliderFloat("Hold", &r.hold, 0.0f, 1.0f, "%.2f");
        tooltip("Right after the reflex: 0 = arm goes limp with the rebound, 1 = muscles pull straight back to the animation");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150.0f);
        ImGui::SliderFloat("Continue at", &r.recoveryClipTime, 0.95f, 1.5f, "clip %.2f s");
        tooltip("Where the animation continues after the hit (skips the follow-through)");
    }

    // --- Muscles ---
    if (ImGui::CollapsingHeader("Muscles")) {
        ImGui::TextDisabled("Tone presets scale stiffness and damping (strength stays).");
        const auto scaleTone = [&](f32 factor) {
            const physics::Muscle base[3] = {{.stiffness = 900.0f, .damping = 60.0f, .maxTorque = m_muscles[0].maxTorque},
                                             {.stiffness = 450.0f, .damping = 30.0f, .maxTorque = m_muscles[1].maxTorque},
                                             {.stiffness = 120.0f, .damping = 6.0f, .maxTorque = m_muscles[2].maxTorque}};
            for (usize i = 0; i < 3; ++i) {
                m_muscles[i].stiffness = base[i].stiffness * factor;
                m_muscles[i].damping = base[i].damping * std::sqrt(factor);
                m_world.setMuscle(m_joints[i].id, m_muscles[i]);
            }
        };
        if (ImGui::SmallButton("Relaxed")) scaleTone(0.4f);
        ImGui::SameLine();
        if (ImGui::SmallButton("Normal")) scaleTone(1.0f);
        ImGui::SameLine();
        if (ImGui::SmallButton("Braced")) scaleTone(2.5f);
        for (usize i = 0; i < 3; ++i) {
            ImGui::PushID(static_cast<int>(i));
            ImGui::Text("%s", m_joints[i].name);
            bool changed = false;
            ImGui::SetNextItemWidth(160.0f);
            changed |= ImGui::SliderFloat("Strength", &m_muscles[i].maxTorque, 1.0f, 200.0f, "%.0f Nm");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(130.0f);
            changed |= ImGui::SliderFloat("Stiffness", &m_muscles[i].stiffness, 0.0f, 3000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
            ImGui::SetNextItemWidth(160.0f);
            changed |= ImGui::SliderFloat("Damping", &m_muscles[i].damping, 0.0f, 200.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
            ImGui::SameLine();
            changed |= ImGui::Checkbox("Active", &m_muscles[i].enabled);
            if (changed) {
                m_world.setMuscle(m_joints[i].id, m_muscles[i]);
            }
            ImGui::PopID();
        }
    }

    // --- Debug draw ---
    if (ImGui::CollapsingHeader("Debug draw", ImGuiTreeNodeFlags_DefaultOpen)) {
        DebugOptions& o = m_debug;
        if (ImGui::BeginTable("##debug", 3)) {
            const auto option = [](const char* label, bool* value, const char* help) {
                ImGui::TableNextColumn();
                ImGui::Checkbox(label, value);
                tooltip(help);
            };
            option("Shapes", &o.shapes, "Collision shapes as wireframes");
            option("Centres of mass", &o.centerOfMass, "White crosses");
            option("Joint axes", &o.jointAxes, "Blue: anchors and hinge axes");
            option("Joint forces", &o.jointForces, "Orange: force each joint applies to the next segment");
            option("Muscle torques", &o.muscleTorques, "Arcs around each joint: sweep = load (270 deg = full strength), green -> red");
            option("Contact forces", &o.contactForces, "Red: resultant contact force on the blade; yellow: contact points");
            option("Grip", &o.gripForce, "Orange: grip force; magenta: grip torque vector (1 cm per N m)");
            option("Velocities", &o.velocities, "Cyan: velocity of each centre of mass");
            option("Gravity", &o.gravity, "Grey: weight of each segment");
            option("Target pose", &o.targetPose, "Blue ghost: the pose the animation asks for");
            option("Blade trail", &o.bladeTrail, "Tip path over 0.6 s, white = 30 m/s");
            option("Labels", &o.labels, "Numbers next to joints and the contact in the Viewport");
            ImGui::EndTable();
        }
        f32 mmPerNewton = o.forceScale * 1000.0f;
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::SliderFloat("Force scale", &mmPerNewton, 0.05f, 20.0f, "%.2f mm/N", ImGuiSliderFlags_Logarithmic)) {
            o.forceScale = mmPerNewton / 1000.0f;
        }
    }
    ImGui::End();
}

} // namespace kinetic::editor
