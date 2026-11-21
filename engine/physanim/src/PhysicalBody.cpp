#include "kinetic/physanim/PhysicalBody.h"

#include <kinetic/animation/PoseOps.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace kinetic::physanim {

namespace {

constexpr f32 kPi = 3.14159265f;

struct Frame {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 scale{1.0f};
};

// Splits a world matrix into position, rotation (orthonormalised) and per-axis scale.
Frame decompose(const glm::mat4& m)
{
    Frame f;
    f.position = glm::vec3(m[3]);
    glm::mat3 r(m);
    for (int i = 0; i < 3; ++i) {
        f.scale[i] = glm::length(r[i]);
        if (f.scale[i] > 1e-8f) r[i] /= f.scale[i];
    }
    if (glm::determinant(r) < 0.0f) { // mirrored: keep a proper rotation
        r[0] = -r[0];
        f.scale.x = -f.scale.x;
    }
    f.rotation = glm::normalize(glm::quat_cast(r));
    return f;
}

glm::mat4 compose(const glm::vec3& p, const glm::quat& r, const glm::vec3& s)
{
    return glm::translate(glm::mat4(1.0f), p) * glm::mat4_cast(r) * glm::scale(glm::mat4(1.0f), s);
}

glm::quat rotationBetween(const glm::vec3& from, const glm::vec3& to)
{
    const glm::vec3 a = glm::normalize(from);
    const glm::vec3 b = glm::normalize(to);
    if (glm::dot(a, b) < -0.9999f) {
        glm::vec3 axis = glm::cross(a, glm::vec3(1.0f, 0.0f, 0.0f));
        if (glm::length(axis) < 1e-3f) axis = glm::cross(a, glm::vec3(0.0f, 0.0f, 1.0f));
        return glm::angleAxis(kPi, glm::normalize(axis));
    }
    return glm::rotation(a, b);
}

glm::vec3 perpendicular(const glm::vec3& v)
{
    const glm::vec3 other = std::abs(v.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    return glm::normalize(glm::cross(v, other));
}

f32 capsuleVolume(f32 radius, f32 halfHeight)
{
    return kPi * radius * radius * 2.0f * halfHeight + 4.0f / 3.0f * kPi * radius * radius * radius;
}

f32 angleOf(const glm::quat& q)
{
    return 2.0f * std::acos(std::min(1.0f, std::abs(glm::normalize(q).w)));
}

std::string lower(std::string s)
{
    std::ranges::transform(s, s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

} // namespace

// ---------------------------------------------------------------------------------------------- //
// Layout

BodyLayout layoutBody(const anim::SkinnedModel& model, f32 minSegment)
{
    BodyLayout layout;
    const anim::Skeleton& skeleton = model.skeleton;
    const usize count = skeleton.jointCount();
    layout.segmentOfJoint.assign(count, -1);
    if (count == 0) {
        return layout;
    }
    std::vector<glm::mat4> rest(count);
    anim::localToModel(skeleton, skeleton.restPose(), rest);
    const auto pos = [&](usize j) { return glm::vec3(rest[j][3]); };

    f32 low = 1e30f;
    f32 high = -1e30f;
    for (usize j = 0; j < count; ++j) {
        low = std::min(low, pos(j).y);
        high = std::max(high, pos(j).y);
    }
    layout.height = model.restBoundsMax.y > model.restBoundsMin.y ? model.restBoundsMax.y - model.restBoundsMin.y : std::max(high - low, 1e-3f);
    const f32 minLength = minSegment * layout.height;

    // Bone hierarchy (helper nodes skipped).
    const auto boneParent = [&](usize j) -> i32 {
        i32 p = skeleton.parents()[j];
        while (p >= 0 && !model.jointIsBone(static_cast<usize>(p))) p = skeleton.parents()[static_cast<usize>(p)];
        return p;
    };
    std::vector<std::vector<i32>> children(count);
    std::vector<i32> parentOf(count, -1);
    for (usize j = 0; j < count; ++j) {
        if (!model.jointIsBone(j)) continue;
        parentOf[j] = boneParent(j);
        if (parentOf[j] >= 0) children[static_cast<usize>(parentOf[j])].push_back(static_cast<i32>(j));
    }
    std::vector<i32> subtree(count, 1);
    for (usize j = count; j-- > 0;) { // parents come first, so children are done before them
        if (parentOf[j] >= 0) subtree[static_cast<usize>(parentOf[j])] += subtree[j];
    }
    // The root bone: the biggest bone tree.
    i32 root = -1;
    for (usize j = 0; j < count; ++j) {
        if (model.jointIsBone(j) && parentOf[j] < 0 && (root < 0 || subtree[j] > subtree[static_cast<usize>(root)])) root = static_cast<i32>(j);
    }

    // Where each bone's body goes: towards the child that continues the chain (the one most in
    // line with the bone above; for the root, the biggest subtree - the spine, not a leg).
    const auto primaryChild = [&](usize j) -> i32 {
        const auto& c = children[j];
        if (c.empty()) return -1;
        if (c.size() == 1) return c.front();
        const i32 p = parentOf[j];
        if (p >= 0 && glm::length(pos(j) - pos(static_cast<usize>(p))) > 1e-5f) {
            const glm::vec3 along = glm::normalize(pos(j) - pos(static_cast<usize>(p)));
            i32 best = -1;
            f32 bestDot = -2.0f;
            for (const i32 k : c) {
                const glm::vec3 d = pos(static_cast<usize>(k)) - pos(j);
                if (glm::length(d) < 1e-6f) continue;
                const f32 dot = glm::dot(glm::normalize(d), along);
                if (dot > bestDot) {
                    bestDot = dot;
                    best = k;
                }
            }
            if (best >= 0) return best;
        }
        return *std::max_element(c.begin(), c.end(), [&](i32 a, i32 b) { return subtree[static_cast<usize>(a)] < subtree[static_cast<usize>(b)]; });
    };

    std::vector<i32> segmentIndex(count, -1);
    for (usize j = 0; j < count; ++j) {
        if (!model.jointIsBone(j)) continue;
        const i32 child = primaryChild(j);
        const f32 length = child >= 0 ? glm::length(pos(static_cast<usize>(child)) - pos(j)) : 0.0f;
        const bool isRoot = static_cast<i32>(j) == root;
        if (child < 0 || (length < minLength && !isRoot)) continue;
        // Parent segment: the nearest bone above with a body.
        i32 parentSegment = -1;
        for (i32 p = parentOf[j]; p >= 0; p = parentOf[static_cast<usize>(p)]) {
            if (segmentIndex[static_cast<usize>(p)] >= 0) {
                parentSegment = segmentIndex[static_cast<usize>(p)];
                break;
            }
        }
        if (parentSegment < 0 && !isRoot) continue; // a separate tree (props, helpers)
        BodySegment s;
        s.joint = static_cast<i32>(j);
        s.parent = parentSegment;
        s.start = pos(j);
        s.end = pos(static_cast<usize>(child));
        segmentIndex[j] = static_cast<i32>(layout.segments.size());
        layout.segments.push_back(std::move(s));
    }
    // Riders: every other bone rides on the nearest body above it.
    for (usize j = 0; j < count; ++j) {
        if (segmentIndex[j] >= 0) {
            layout.segmentOfJoint[j] = segmentIndex[j];
            continue;
        }
        for (i32 p = skeleton.parents()[j]; p >= 0; p = skeleton.parents()[static_cast<usize>(p)]) {
            if (segmentIndex[static_cast<usize>(p)] >= 0) {
                layout.segmentOfJoint[j] = segmentIndex[static_cast<usize>(p)];
                if (model.jointIsBone(j)) layout.segments[static_cast<usize>(segmentIndex[static_cast<usize>(p)])].riders.push_back(static_cast<i32>(j));
                break;
            }
        }
    }
    // Stretch each capsule over its riders (a hand over its fingers, a head to its top).
    for (BodySegment& s : layout.segments) {
        const f32 length = glm::length(s.end - s.start);
        if (length < 1e-6f) continue;
        const glm::vec3 axis = (s.end - s.start) / length;
        f32 reach = length;
        for (const i32 r : s.riders) reach = std::max(reach, glm::dot(pos(static_cast<usize>(r)) - s.start, axis));
        s.end = s.start + axis * reach;
    }
    // Thickness from the skin: median distance of the vertices bound mostly to each body.
    std::vector<std::vector<f32>> distances(layout.segments.size());
    if (model.skin.isValid(count) && !model.skin.mesh.positions.empty()) {
        const MeshData posed = anim::skinnedRestMesh(model);
        for (usize v = 0; v < posed.positions.size() && v < model.skin.joints.size(); ++v) {
            const glm::vec4& w = model.skin.weights[v];
            int best = 0;
            for (int k = 1; k < 4; ++k) {
                if (w[k] > w[best]) best = k;
            }
            const u32 joint = model.skin.joints[v][best];
            if (joint >= count) continue;
            const i32 seg = layout.segmentOfJoint[joint];
            if (seg < 0) continue;
            const BodySegment& s = layout.segments[static_cast<usize>(seg)];
            const glm::vec3 d = s.end - s.start;
            const f32 t = glm::dot(d, d) > 1e-12f ? std::clamp(glm::dot(posed.positions[v] - s.start, d) / glm::dot(d, d), 0.0f, 1.0f) : 0.0f;
            distances[static_cast<usize>(seg)].push_back(glm::length(posed.positions[v] - (s.start + d * t)));
        }
    }
    for (usize i = 0; i < layout.segments.size(); ++i) {
        BodySegment& s = layout.segments[i];
        auto& d = distances[i];
        const f32 length = glm::length(s.end - s.start);
        f32 radius = length * 0.15f;
        if (d.size() >= 8) {
            std::nth_element(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(d.size() / 2), d.end());
            radius = d[d.size() / 2];
        }
        s.radius = std::clamp(radius, 0.012f * layout.height, 0.1f * layout.height);
    }
    return layout;
}

std::vector<i32> weaponArmJoints(const BodyLayout& layout, i32 hand)
{
    std::vector<i32> joints;
    for (i32 s = layout.segmentOf(hand); s >= 0 && joints.size() < 3; s = layout.segments[static_cast<usize>(s)].parent) {
        joints.push_back(layout.segments[static_cast<usize>(s)].joint);
    }
    return joints;
}

std::vector<i32> fullBodyJoints(const BodyLayout& layout)
{
    std::vector<i32> joints;
    for (const BodySegment& s : layout.segments) joints.push_back(s.joint);
    return joints;
}

// ---------------------------------------------------------------------------------------------- //
// Build

bool PhysicalBody::build(physics::PhysicsWorld& world, const anim::SkinnedModel& model, std::span<const glm::mat4> pose,
                         const glm::mat4& entityWorld, std::span<const WeaponMount> weapons, const std::string& name,
                         u32 collisionGroup, const BodySettings& settings)
{
    m_settings = settings;
    m_segments.clear();
    m_weapons.clear();
    const usize count = model.skeleton.jointCount();
    if (pose.size() != count) {
        return false;
    }
    m_layout = layoutBody(model);
    if (m_layout.segments.empty()) {
        return false;
    }
    std::vector<glm::mat4> rest(count);
    anim::localToModel(model.skeleton, model.skeleton.restPose(), rest);

    // Density so the body weighs settings.bodyMass (capsules around armour or loose clothes would
    // otherwise make it far too heavy).
    f32 volume = 0.0f;
    for (const BodySegment& ls : m_layout.segments) {
        const usize j = static_cast<usize>(ls.joint);
        const glm::mat4 toWorld = entityWorld * pose[j] * glm::inverse(rest[j]);
        const f32 length = std::max(glm::length(glm::vec3(toWorld * glm::vec4(ls.end, 1.0f)) - glm::vec3(toWorld * glm::vec4(ls.start, 1.0f))), 0.02f);
        const f32 restLength = glm::length(ls.end - ls.start);
        const f32 scale = restLength > 1e-6f ? length / restLength : 1.0f;
        const f32 radius = std::min(ls.radius * scale, length * 0.5f + ls.radius * scale * 0.5f);
        volume += capsuleVolume(radius, std::max(length * 0.5f - radius, 0.005f));
    }
    const f32 density = settings.bodyMass > 0.0f && volume > 0.0f ? settings.bodyMass / volume : settings.density;

    // Bodies.
    m_totalMass = 0.0f;
    f32 heightScale = 0.0f;
    for (const BodySegment& ls : m_layout.segments) {
        const usize j = static_cast<usize>(ls.joint);
        const glm::mat4 jointWorld = entityWorld * pose[j];
        const Frame f = decompose(jointWorld);
        // The capsule in the joint's space (rest), carried into the current pose.
        const glm::mat4 toJoint = glm::inverse(rest[j]);
        const glm::vec3 startW = glm::vec3(jointWorld * toJoint * glm::vec4(ls.start, 1.0f));
        const glm::vec3 endW = glm::vec3(jointWorld * toJoint * glm::vec4(ls.end, 1.0f));
        const f32 restLength = glm::length(ls.end - ls.start);
        const f32 length = std::max(glm::length(endW - startW), 0.02f);
        const f32 toWorld = restLength > 1e-6f ? length / restLength : 1.0f;
        heightScale = std::max(heightScale, toWorld);
        const glm::vec3 local = glm::inverse(f.rotation) * glm::normalize(endW - startW);
        const f32 radius = std::min(ls.radius * toWorld, length * 0.5f + ls.radius * toWorld * 0.5f);
        const f32 halfHeight = std::max(length * 0.5f - radius, 0.005f);

        Segment s;
        s.joint = ls.joint;
        s.parent = ls.parent;
        s.scale = f.scale;
        s.localEnd = local * length;
        s.target = s.previous = s.driven = Target{f.position, f.rotation};

        physics::BodyDesc d;
        d.name = name + " " + model.skeleton.names()[j];
        d.shape.type = physics::ShapeType::Capsule;
        d.shape.radius = radius;
        d.shape.halfHeight = halfHeight;
        d.shape.offset = local * (length * 0.5f);
        d.shape.rotation = rotationBetween(glm::vec3(0.0f, 1.0f, 0.0f), local);
        d.position = f.position;
        d.rotation = f.rotation;
        d.motion = physics::MotionType::Kinematic;
        d.mass = density * capsuleVolume(radius, halfHeight);
        d.collisionGroup = collisionGroup;
        d.friction = 0.7f;
        d.restitution = 0.05f;
        d.hardness = scene::surfaceProperties(scene::SurfaceMaterial::Flesh).hardness;
        s.mass = d.mass;
        s.body = world.addBody(d);
        m_totalMass += s.mass;
        m_segments.push_back(s);
    }
    m_worldHeight = m_layout.height * (heightScale > 0.0f ? heightScale : 1.0f);

    // Weapons, welded to the body of their joint.
    for (const WeaponMount& mount : weapons) {
        const i32 owner = m_layout.segmentOf(mount.joint);
        if (owner < 0 || mount.joint < 0 || static_cast<usize>(mount.joint) >= count) continue;
        const Frame socket = decompose(entityWorld * pose[static_cast<usize>(mount.joint)] * mount.offset);
        physics::BodyDesc w;
        w.name = name + " weapon";
        w.shape.type = physics::ShapeType::Box;
        w.shape.halfExtents = glm::max(mount.weapon.halfExtents, glm::vec3(0.003f));
        w.shape.offset = mount.weapon.center;
        w.centerOfMass = mount.weapon.balance - mount.weapon.center;
        w.position = socket.position;
        w.rotation = socket.rotation;
        w.motion = physics::MotionType::Kinematic;
        w.mass = std::max(mount.weapon.mass, 0.05f);
        w.friction = 0.2f;     // steel
        w.restitution = 0.05f; // low: Jolt takes the max, so the surface decides the bounce
        w.sharp = mount.weapon.sharp;
        w.hardness = 1.0f;
        w.continuousCollision = true;
        w.collisionGroup = collisionGroup;
        Weapon weapon;
        weapon.body = world.addBody(w);
        weapon.owner = owner;
        const Segment& o = m_segments[static_cast<usize>(owner)];
        weapon.inOwner.rotation = glm::inverse(o.target.rotation) * socket.rotation;
        weapon.inOwner.position = glm::inverse(o.target.rotation) * (socket.position - o.target.position);
        // The arm that reacts to its hits: the owner and the bodies above it, up to the torso (a
        // body carrying three or more others).
        for (i32 s = owner; s >= 0 && weapon.chain.size() < 4; s = m_segments[static_cast<usize>(s)].parent) {
            usize children = 0;
            for (const Segment& c : m_segments) children += c.parent == s ? 1 : 0;
            if (children >= 3) break;
            weapon.chain.push_back(s);
        }
        physics::FixedDesc grip;
        grip.name = name + " grip";
        grip.parent = o.body;
        grip.child = weapon.body;
        grip.anchor = socket.position;
        world.addFixed(grip);
        m_totalMass += w.mass;
        m_weapons.push_back(std::move(weapon));
    }

    // Ball joints to the parent body, with muscles sized to what they move: every body below a
    // joint (and the weapons in its hands) as a rod - its mass at its centre plus m L^2 / 12 -
    // gives the inertia about the joint. A sword balances near the guard but is long: the rod
    // term is what makes a wrist holding it need real strength.
    struct Mass {
        f32 mass;
        glm::vec3 center;
        f32 length;
        i32 segment; // owner
    };
    std::vector<Mass> masses;
    for (usize i = 0; i < m_segments.size(); ++i) {
        const Segment& s = m_segments[i];
        masses.push_back({s.mass, s.target.position + s.target.rotation * (s.localEnd * 0.5f), glm::length(s.localEnd), static_cast<i32>(i)});
    }
    for (const Weapon& w : m_weapons) {
        // From the description: the body state is only read back after the first step.
        const physics::BodyDesc& d = world.bodyDesc(w.body);
        const glm::vec3 he = d.shape.halfExtents;
        masses.push_back({d.mass, d.position + d.rotation * (d.shape.offset + d.centerOfMass), 2.0f * std::max({he.x, he.y, he.z}), w.owner});
    }
    const auto below = [&](i32 segment, i32 joint) {
        for (i32 s = segment; s >= 0; s = m_segments[static_cast<usize>(s)].parent) {
            if (s == joint) return true;
        }
        return false;
    };
    for (usize i = 0; i < m_segments.size(); ++i) {
        Segment& s = m_segments[i];
        if (s.parent < 0) continue;
        f32 mass = 0.0f;
        f32 inertia = 0.0f;
        for (const Mass& m : masses) {
            if (!below(m.segment, static_cast<i32>(i))) continue;
            const glm::vec3 d = m.center - s.target.position;
            mass += m.mass;
            inertia += m.mass * (glm::dot(d, d) + m.length * m.length / 12.0f);
        }
        inertia = std::max(inertia, 1e-3f);
        // Light limbs (a hand) are quicker than heavy ones (a leg, the torso).
        const f32 quick = std::clamp(std::sqrt(0.1f * m_totalMass / std::max(mass, 0.05f)), 1.0f, 2.5f);
        const f32 omega = 2.0f * kPi * settings.frequency * quick;
        s.grip = std::ranges::any_of(m_weapons, [&](const Weapon& w) { return w.owner == static_cast<i32>(i); });
        s.muscle = physics::Muscle{inertia * omega * omega, 2.0f * settings.dampingRatio * inertia * omega,
                                   settings.strength * std::sqrt(inertia) * (s.grip ? settings.gripStrength : 1.0f), true};

        physics::SwingTwistDesc j;
        j.name = name + " " + model.skeleton.names()[static_cast<usize>(s.joint)];
        j.parent = m_segments[static_cast<usize>(s.parent)].body;
        j.child = s.body;
        j.anchor = s.target.position;
        const glm::vec3 along = s.target.rotation * s.localEnd;
        j.twistAxis = glm::length(along) > 1e-5f ? glm::normalize(along) : glm::vec3(0.0f, 1.0f, 0.0f);
        j.planeAxis = perpendicular(j.twistAxis);
        j.normalHalfCone = kPi;
        j.planeHalfCone = kPi;
        j.twistMin = -kPi;
        j.twistMax = kPi;
        j.muscle = s.muscle;
        j.muscle.enabled = false;
        s.constraint = world.addSwingTwist(j);
        if (!s.constraint.valid()) {
            return false;
        }
    }
    m_hasTargets = true;
    return true;
}

// ---------------------------------------------------------------------------------------------- //
// Runtime

bool PhysicalBody::simulated() const
{
    return std::ranges::any_of(m_segments, [](const Segment& s) { return s.dynamic; });
}

void PhysicalBody::setTargets(const glm::mat4& entityWorld, std::span<const glm::mat4> animatedModel, f32 frameTime,
                              std::span<const i32> physicsJoints, f32 strength)
{
    m_frameTime = std::max(frameTime, 1e-4f);
    m_strength = std::max(strength, 0.0f);
    for (Segment& s : m_segments) {
        if (static_cast<usize>(s.joint) >= animatedModel.size()) return;
        const Frame f = decompose(entityWorld * animatedModel[static_cast<usize>(s.joint)]);
        s.previous = s.target;
        s.target = Target{f.position, f.rotation};
        s.wanted = false;
    }
    for (const i32 joint : physicsJoints) {
        if (const i32 seg = m_layout.segmentOf(joint); seg >= 0) m_segments[static_cast<usize>(seg)].wanted = true;
    }
}

PhysicalBody::Target PhysicalBody::interpolated(const Segment& s, f32 alpha) const
{
    return Target{glm::mix(s.previous.position, s.target.position, alpha), glm::slerp(s.previous.rotation, s.target.rotation, alpha)};
}

void PhysicalBody::setSegmentDynamic(physics::PhysicsWorld& world, usize i, bool dynamic)
{
    Segment& s = m_segments[i];
    if (s.dynamic == dynamic) return;
    s.dynamic = dynamic;
    world.setMotionType(s.body, dynamic ? physics::MotionType::Dynamic : physics::MotionType::Kinematic);
    s.releaseStart = -1.0f;
}

void PhysicalBody::syncWeapons(physics::PhysicsWorld& world)
{
    for (Weapon& w : m_weapons) {
        const bool dynamic = m_segments[static_cast<usize>(w.owner)].dynamic;
        const bool was = world.bodyDesc(w.body).motion == physics::MotionType::Dynamic;
        if (dynamic != was) {
            world.setMotionType(w.body, dynamic ? physics::MotionType::Dynamic : physics::MotionType::Kinematic);
            w.tracker.reset();
        }
    }
}

void PhysicalBody::assist(physics::PhysicsWorld& world, f32 dt, f32 alpha)
{
    // Balance assist: the pelvis is pulled towards its animated place with limited force (it
    // carries the weight of every simulated body), and turned towards its animated orientation.
    const Segment& root = m_segments.front();
    const physics::BodyState& b = world.body(root.body);
    f32 mass = 0.0f;
    for (const Segment& s : m_segments) mass += s.dynamic ? s.mass : 0.0f;
    for (const Weapon& w : m_weapons) mass += m_segments[static_cast<usize>(w.owner)].dynamic ? world.body(w.body).mass : 0.0f;
    const Target t = interpolated(root, alpha);
    const glm::vec3 targetVelocity = (root.target.position - root.previous.position) / m_frameTime;
    const f32 w = 2.0f * kPi * m_settings.assistFrequency;
    glm::vec3 accel = (t.position - b.position) * (w * w) + (targetVelocity - b.linearVelocity) * (2.0f * w);
    const f32 limit = m_settings.assistMaxAcceleration;
    if (glm::length(accel) > limit) accel = glm::normalize(accel) * limit;
    const glm::vec3 force = mass * (accel - world.gravity());

    glm::quat error = glm::normalize(t.rotation * glm::inverse(b.rotation));
    if (error.w < 0.0f) error = -error;
    const f32 angle = angleOf(error);
    const glm::vec3 axis = angle > 1e-5f ? glm::normalize(glm::vec3(error.x, error.y, error.z)) : glm::vec3(0.0f);
    const f32 inertia = mass * (0.2f * m_worldHeight) * (0.2f * m_worldHeight);
    glm::vec3 alphaVec = axis * (angle * w * w) - b.angularVelocity * (2.0f * w);
    const f32 alphaLimit = limit / std::max(0.2f * m_worldHeight, 0.05f);
    if (glm::length(alphaVec) > alphaLimit) alphaVec = glm::normalize(alphaVec) * alphaLimit;
    world.addForce(root.body, force, alphaVec * inertia);
    (void)dt;
}

void PhysicalBody::step(physics::PhysicsWorld& world, f32 dt, f32 alpha, f32 time)
{
    if (!built() || !m_hasTargets) {
        return;
    }
    // Which bodies are simulated.
    for (usize i = 0; i < m_segments.size(); ++i) {
        Segment& s = m_segments[i];
        if (s.wanted) {
            if (!s.dynamic) {
                setSegmentDynamic(world, i, true);
            }
            s.releaseStart = -1.0f;
        } else if (s.dynamic) {
            if (s.releaseStart < 0.0f) {
                s.releaseStart = time;
            } else if (time - s.releaseStart >= m_settings.releaseTime) {
                setSegmentDynamic(world, i, false);
            }
        }
    }
    syncWeapons(world);

    // Kinematic bodies: the animation, riding on simulated parents (parents come first).
    const f32 lead = std::clamp(m_settings.lead / m_frameTime, 0.0f, 3.0f); // in frames
    for (usize i = 0; i < m_segments.size(); ++i) {
        Segment& s = m_segments[i];
        const Target anim = interpolated(s, alpha);
        if (s.dynamic) {
            const physics::BodyState& b = world.body(s.body);
            s.driven = Target{b.position, b.rotation};
        } else {
            s.driven = anim;
            if (s.parent >= 0) {
                const Segment& p = m_segments[static_cast<usize>(s.parent)];
                const bool parentMoved = p.dynamic || glm::length(p.driven.position - interpolated(p, alpha).position) > 1e-5f;
                if (parentMoved) {
                    const Target pa = interpolated(p, alpha);
                    const glm::quat relR = glm::inverse(pa.rotation) * anim.rotation;
                    const glm::vec3 relP = glm::inverse(pa.rotation) * (anim.position - pa.position);
                    s.driven = Target{p.driven.position + p.driven.rotation * relP, glm::normalize(p.driven.rotation * relR)};
                }
            }
            world.moveKinematic(s.body, s.driven.position, s.driven.rotation, dt);
        }
        if (!s.constraint.valid()) continue;

        // Muscle of the joint above a simulated body: the animated rotation relative to the parent.
        physics::Muscle m = s.muscle;
        m.enabled = s.dynamic;
        m.maxTorque *= m_strength;
        if (!s.dynamic) {
            world.setMuscle(s.constraint, m);
            continue;
        }
        const Segment& p = m_segments[static_cast<usize>(s.parent)];
        glm::quat animated = glm::normalize(glm::inverse(p.target.rotation) * s.target.rotation);
        if (lead > 0.0f) {
            const glm::quat before = glm::normalize(glm::inverse(p.previous.rotation) * s.previous.rotation);
            glm::quat delta = glm::normalize(animated * glm::inverse(before));
            if (delta.w < 0.0f) delta = -delta;
            animated = glm::normalize(glm::slerp(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), delta, lead) * animated);
        }
        glm::quat target = animated;
        const glm::quat current = glm::normalize(glm::inverse(world.body(p.body).rotation) * world.body(s.body).rotation);
        const glm::quat pose = glm::normalize(glm::inverse(p.target.rotation) * s.target.rotation);
        const f32 limit = s.grip ? m_settings.wristLimit : m_settings.jointLimit;
        const f32 bent = angleOf(current * glm::inverse(pose));
        const f32 sinceReflex = time - s.reflexStart;
        if (bent > limit) {
            // Past the ligaments: back onto the animation, and much harder than a muscle could.
            m.maxTorque *= 1.0f + m_settings.ligamentStiffness * (bent - limit) / limit;
        } else if (sinceReflex < m_settings.reflexTime) {
            m.maxTorque *= m_settings.reflexStrength; // yielding to the hit
            target = glm::normalize(glm::inverse(world.body(p.body).rotation) * world.body(s.body).rotation);
        } else if (sinceReflex < m_settings.reflexTime + m_settings.recoverTime) {
            // Back onto the animation, from where the hit left it.
            const f32 x = std::clamp((sinceReflex - m_settings.reflexTime) / m_settings.recoverTime, 0.0f, 1.0f);
            const f32 w = m_settings.hold + (1.0f - m_settings.hold) * x * x * (3.0f - 2.0f * x);
            target = glm::slerp(current, glm::dot(current, animated) < 0.0f ? -animated : animated, w);
        }
        world.setMuscle(s.constraint, m);
        world.setTargetRotation(s.constraint, target);
    }
    // Kinematic weapons follow their hand.
    for (Weapon& w : m_weapons) {
        const Segment& o = m_segments[static_cast<usize>(w.owner)];
        if (o.dynamic) continue;
        world.moveKinematic(w.body, o.driven.position + o.driven.rotation * w.inOwner.position, o.driven.rotation * w.inOwner.rotation, dt);
    }

    // Full body: the balance assist (it never gives up: no falling over).
    if (m_segments.front().dynamic) {
        assist(world, dt, alpha);
    }
}

void PhysicalBody::afterStep(physics::PhysicsWorld& world, const SurfaceMap& surfaces, f32 time, std::vector<HitEvent>* events)
{
    if (!built()) {
        return;
    }
    for (Weapon& w : m_weapons) {
        if (!m_segments[static_cast<usize>(w.owner)].dynamic) continue;
        const HitTracker::Events e = w.tracker.update(world, w.body, surfaces, time);
        // Shoulder, elbow, wrist (the chain runs hand first).
        std::array<f32, 3> loads{};
        const usize n = std::min<usize>(3, w.chain.size());
        for (usize k = 0; k < n; ++k) loads[k] = load(world, static_cast<usize>(w.chain[n - 1 - k]));
        w.tracker.recordLoads(loads, time);
        if (e.impact && events && !w.tracker.hits().empty()) {
            const Hit& h = w.tracker.hits().back();
            events->push_back(HitEvent{h.time, h.point, h.normal, h.velocityBefore, h.material, h.soft, h.targetBody, w.body.index});
        }
        // A hard hit (rebound / stop) or a stuck blade: the arm yields, then recovers.
        if (e.judged && (e.result == HitResult::Rebound || e.result == HitResult::Stop || e.result == HitResult::Stuck)) {
            ++m_hardHits;
            for (const i32 s : w.chain) {
                Segment& seg = m_segments[static_cast<usize>(s)];
                if (seg.grip) continue; // the wrist keeps its grip; the elbow and shoulder give
                if (seg.dynamic && time - seg.reflexStart > m_settings.reflexTime) seg.reflexStart = time;
            }
        }
    }
}

void PhysicalBody::writeOverrides(const physics::PhysicsWorld& world, const glm::mat4& entityWorld, scene::AnimatorComponent& animator) const
{
    if (!built() || !simulated()) {
        clearOverrides(animator);
        return;
    }
    const glm::mat4 toModel = glm::inverse(entityWorld);
    animator.overrideAnchor = -1;
    animator.boneOverrides.clear();
    animator.overrideWeight = 1.0f;
    for (const Segment& s : m_segments) {
        // Simulated bodies and kinematic ones riding on them: where the physics has them.
        if (!s.dynamic && s.parent >= 0 && !m_segments[static_cast<usize>(s.parent)].dynamic) {
            bool ridesOnSimulated = false;
            for (i32 p = s.parent; p >= 0; p = m_segments[static_cast<usize>(p)].parent) {
                if (m_segments[static_cast<usize>(p)].dynamic) {
                    ridesOnSimulated = true;
                    break;
                }
            }
            if (!ridesOnSimulated) continue;
        } else if (!s.dynamic && s.parent < 0) {
            continue;
        }
        const physics::BodyState& b = world.body(s.body);
        animator.boneOverrides.push_back({s.joint, toModel * compose(b.position, b.rotation, s.scale)});
    }
}

void PhysicalBody::clearOverrides(scene::AnimatorComponent& animator)
{
    animator.boneOverrides.clear();
    animator.overrideAnchor = -1;
    animator.overrideWeight = 1.0f;
}

glm::vec3 PhysicalBody::segmentEnd(const physics::PhysicsWorld& world, usize i) const
{
    const physics::BodyState& b = world.body(m_segments[i].body);
    return b.position + b.rotation * m_segments[i].localEnd;
}

f32 PhysicalBody::load(const physics::PhysicsWorld& world, usize i) const
{
    const Segment& s = m_segments[i];
    if (!s.constraint.valid() || !s.dynamic) return 0.0f;
    return std::abs(world.joint(s.constraint).muscleTorque) / std::max(s.muscle.maxTorque * m_strength, 1e-3f);
}

f32 PhysicalBody::trackingError(const physics::PhysicsWorld& world, i32 joint) const
{
    const i32 seg = m_layout.segmentOf(joint);
    if (seg < 0) return 0.0f;
    const Segment& s = m_segments[static_cast<usize>(seg)];
    return angleOf(world.body(s.body).rotation * glm::inverse(s.target.rotation));
}

glm::vec3 PhysicalBody::rootPosition(const physics::PhysicsWorld& world) const
{
    return built() ? world.body(m_segments.front().body).position : glm::vec3(0.0f);
}

// ---------------------------------------------------------------------------------------------- //
// Strike analysis

StrikePoint findStrikePoint(const anim::Skeleton& skeleton, const anim::AnimationClip& clip, i32 hand,
                            const glm::mat4& weaponOffset, const scene::WeaponComponent& weapon)
{
    StrikePoint best;
    if (hand < 0 || static_cast<usize>(hand) >= skeleton.jointCount() || clip.duration() <= 0.0f) {
        return best;
    }
    constexpr usize kSamples = 120;
    const f32 dt = clip.duration() / static_cast<f32>(kSamples);
    anim::Pose pose;
    std::vector<glm::mat4> model(skeleton.jointCount());
    const auto bladeAt = [&](f32 t) {
        pose = skeleton.restPose();
        clip.sample(t, false, pose);
        anim::localToModel(skeleton, pose, model);
        const glm::vec3 local = glm::vec3(model[static_cast<usize>(hand)] * weaponOffset * glm::vec4(weapon.center, 1.0f));
        return local + clip.sampleRootMotion(t) - clip.sampleRootMotion(0.0f);
    };
    // Height of the character: the highest joint of the rest pose.
    f32 height = 0.0f;
    pose = skeleton.restPose();
    anim::localToModel(skeleton, pose, model);
    for (const glm::mat4& m : model) height = std::max(height, m[3].y);
    const f32 low = 0.25f * height;
    const f32 high = 0.85f * height;

    glm::vec3 previous = bladeAt(0.0f);
    f32 bestSpeed = -1.0f;
    f32 anySpeed = -1.0f;
    StrikePoint any;
    for (usize i = 1; i <= kSamples; ++i) {
        const f32 t = static_cast<f32>(i) * dt;
        const glm::vec3 p = bladeAt(t);
        const glm::vec3 v = (p - previous) / dt;
        const glm::vec3 mid = (p + previous) * 0.5f;
        const f32 time = (static_cast<f32>(i) - 0.5f) / static_cast<f32>(kSamples);
        if (glm::length(v) > anySpeed) { // fallback: no sample at body height
            anySpeed = glm::length(v);
            any = {time, mid, v};
        }
        if (glm::length(v) > bestSpeed && mid.y >= low && mid.y <= high) {
            bestSpeed = glm::length(v);
            best = {time, mid, v};
        }
        previous = p;
    }
    return bestSpeed >= 0.0f ? best : any;
}

TargetBox strikeTarget(const StrikePoint& strike, const glm::mat4& entityWorld, f32 radius)
{
    const glm::vec3 p = glm::vec3(entityWorld * glm::vec4(strike.bladeCenter, 1.0f));
    const f32 top = std::clamp(p.y + 0.2f, 0.5f, 2.2f);
    // Its far face at the strike point: the blade meets it just before, at nearly full speed (a
    // box straddling the strike point can catch a slow wind-up instead).
    glm::vec3 along = glm::vec3(entityWorld * glm::vec4(strike.velocity, 0.0f));
    along.y = 0.0f;
    along = glm::length(along) > 1e-4f ? glm::normalize(along) : glm::vec3(0.0f);
    TargetBox box;
    box.halfExtents = {radius, top * 0.5f, radius};
    box.center = glm::vec3(p.x, top * 0.5f, p.z) - along * radius;
    return box;
}

} // namespace kinetic::physanim
