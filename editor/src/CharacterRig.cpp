#include "CharacterRig.h"

#include <kinetic/core/Primitives.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>

namespace kinetic::editor::character {

namespace {

physics::ShapeDesc capsuleDown(f32 length, f32 radius)
{
    physics::ShapeDesc s;
    s.type = physics::ShapeType::Capsule;
    s.radius = radius;
    s.halfHeight = std::max(length * 0.5f - radius, 0.01f);
    s.offset = {0.0f, -length * 0.5f, 0.0f};
    return s;
}

physics::ShapeDesc box(const glm::vec3& halfExtents, const glm::vec3& offset)
{
    physics::ShapeDesc s;
    s.type = physics::ShapeType::Box;
    s.halfExtents = halfExtents;
    s.offset = offset;
    return s;
}

std::vector<Part> makeParts()
{
    std::vector<Part> p(kJointCount);
    // Strengths: shoulder ~95, elbow ~70, wrist ~15 N m (flexion, adult). Masses: 75 kg body.
    const physics::Muscle shoulder{.stiffness = 900.0f, .damping = 60.0f, .maxTorque = 95.0f};
    const physics::Muscle elbow{.stiffness = 450.0f, .damping = 30.0f, .maxTorque = 70.0f};
    const physics::Muscle wrist{.stiffness = 120.0f, .damping = 6.0f, .maxTorque = 15.0f};

    p[kRoot] = {.name = "Root", .parent = -1, .restPosition = {0.0f, 0.0f, 0.0f}};

    p[kPelvis] = {.name = "Pelvis", .parent = kRoot, .restPosition = {0.0f, 0.95f, 0.0f},
                  .shape = box({0.15f, 0.09f, 0.10f}, {0.0f, 0.0f, 0.0f}), .mass = 11.0f};
    {
        physics::ShapeDesc chest;
        chest.type = physics::ShapeType::Capsule;
        chest.radius = 0.14f;
        chest.halfHeight = 0.08f;
        chest.offset = {0.0f, 0.22f, 0.0f};
        p[kChest] = {.name = "Chest", .parent = kPelvis, .restPosition = {0.0f, 1.03f, 0.0f}, .shape = chest, .mass = 22.0f,
                     .joint = JointType::SwingTwist, .cone = 25.0f, .twist = 20.0f};
    }
    {
        physics::ShapeDesc head;
        head.type = physics::ShapeType::Sphere;
        head.radius = 0.11f;
        head.offset = {0.0f, 0.13f, 0.0f};
        p[kHead] = {.name = "Head", .parent = kChest, .restPosition = {0.0f, 1.50f, 0.0f}, .shape = head, .mass = 5.5f,
                    .joint = JointType::SwingTwist, .cone = 35.0f, .twist = 40.0f};
    }

    p[kUpperArmR] = {.name = "UpperArm.R", .parent = kChest, .restPosition = {-0.21f, 1.42f, 0.0f},
                     .shape = capsuleDown(0.30f, 0.045f), .mass = 2.1f, .joint = JointType::Hinge, .limitMin = -60.0f,
                     .limitMax = 180.0f, .muscle = shoulder, .swordArm = true};
    p[kForearmR] = {.name = "Forearm.R", .parent = kUpperArmR, .restPosition = {-0.21f, 1.12f, 0.0f},
                    .shape = capsuleDown(0.27f, 0.038f), .mass = 1.25f, .joint = JointType::Hinge, .limitMin = -5.0f,
                    .limitMax = 150.0f, .muscle = elbow, .swordArm = true};
    p[kHandR] = {.name = "Hand.R", .parent = kForearmR, .restPosition = {-0.21f, 0.85f, 0.0f},
                 .shape = box({0.02f, 0.045f, 0.04f}, {0.0f, -0.045f, 0.0f}), .mass = 0.45f, .joint = JointType::Hinge,
                 .limitMin = -70.0f, .limitMax = 80.0f, .muscle = wrist, .swordArm = true};
    // Longsword, 1.3 kg: 0.9 m blade (edge facing forward / back), balance 12 cm in front of the guard.
    p[kSword] = {.name = "Sword", .parent = kHandR, .restPosition = {-0.21f, 0.79f, 0.0f},
                 .shape = box({0.004f, 0.45f, 0.022f}, {0.0f, -0.55f, 0.0f}), .mass = 1.3f,
                 .centerOfMass = {0.0f, 0.45f - 0.12f, 0.0f}, .joint = JointType::Fixed, .swordArm = true};

    p[kUpperArmL] = {.name = "UpperArm.L", .parent = kChest, .restPosition = {0.21f, 1.42f, 0.0f},
                     .shape = capsuleDown(0.30f, 0.045f), .mass = 2.1f, .joint = JointType::SwingTwist, .cone = 70.0f,
                     .twist = 40.0f};
    p[kForearmL] = {.name = "Forearm.L", .parent = kUpperArmL, .restPosition = {0.21f, 1.12f, 0.0f},
                    .shape = capsuleDown(0.27f, 0.038f), .mass = 1.25f, .joint = JointType::Hinge, .limitMin = -5.0f,
                    .limitMax = 150.0f};
    p[kHandL] = {.name = "Hand.L", .parent = kForearmL, .restPosition = {0.21f, 0.85f, 0.0f},
                 .shape = box({0.02f, 0.045f, 0.04f}, {0.0f, -0.045f, 0.0f}), .mass = 0.45f, .joint = JointType::Hinge,
                 .limitMin = -70.0f, .limitMax = 80.0f};

    for (const bool right : {true, false}) {
        const f32 x = right ? -0.10f : 0.10f;
        const u32 thigh = right ? kThighR : kThighL;
        const u32 shin = right ? kShinR : kShinL;
        const u32 foot = right ? kFootR : kFootL;
        p[thigh] = {.name = right ? "Thigh.R" : "Thigh.L", .parent = kPelvis, .restPosition = {x, 0.90f, 0.0f},
                    .shape = capsuleDown(0.42f, 0.07f), .mass = 7.5f, .joint = JointType::SwingTwist, .cone = 60.0f,
                    .twist = 20.0f};
        // Knees bend backwards: negative sagittal angles.
        p[shin] = {.name = right ? "Shin.R" : "Shin.L", .parent = static_cast<i32>(thigh), .restPosition = {x, 0.48f, 0.0f},
                   .shape = capsuleDown(0.40f, 0.05f), .mass = 3.5f, .joint = JointType::Hinge, .limitMin = -150.0f,
                   .limitMax = 3.0f};
        p[foot] = {.name = right ? "Foot.R" : "Foot.L", .parent = static_cast<i32>(shin), .restPosition = {x, 0.08f, 0.0f},
                   .shape = box({0.045f, 0.04f, 0.11f}, {0.0f, -0.04f, 0.06f}), .mass = 1.0f, .joint = JointType::Hinge,
                   .limitMin = -40.0f, .limitMax = 40.0f};
    }
    return p;
}

MeshData partMesh(u32 joint, const Part& part)
{
    const physics::ShapeDesc& s = part.shape;
    if (joint == kSword) {
        MeshData blade = makeBoxMesh(s.halfExtents, s.offset);
        MeshData guard = makeBoxMesh({0.015f, 0.012f, 0.09f}, {0.0f, -0.10f, 0.0f});
        MeshData grip = makeCapsuleMesh(0.016f, 0.045f, {0.0f, -0.04f, 0.0f}, 10, 3);
        for (MeshData* m : {&guard, &grip}) {
            const auto base = static_cast<u32>(blade.positions.size());
            blade.positions.insert(blade.positions.end(), m->positions.begin(), m->positions.end());
            blade.normals.insert(blade.normals.end(), m->normals.begin(), m->normals.end());
            for (const u32 i : m->indices) {
                blade.indices.push_back(base + i);
            }
        }
        return blade;
    }
    switch (s.type) {
    case physics::ShapeType::Box: return makeBoxMesh(s.halfExtents, s.offset);
    case physics::ShapeType::Sphere: return makeSphereMesh(s.radius, s.offset);
    case physics::ShapeType::Capsule: return makeCapsuleMesh(s.radius, s.halfHeight, s.offset, 14, 4);
    }
    return {};
}

// Keyframed clip from per-joint sagittal angles (degrees). Joints without keys are not animated.
struct AngleKeys {
    u32 joint;
    std::vector<std::pair<f32, f32>> keys; // (time, degrees)
};

anim::AnimationClip makeClip(const char* name, f32 duration, const std::vector<AngleKeys>& angles, anim::ClipKind kind,
                             const std::vector<std::pair<f32, f32>>& pelvisHeight = {})
{
    std::vector<anim::JointTracks> tracks(kJointCount);
    for (const AngleKeys& a : angles) {
        for (const auto& [time, degrees] : a.keys) {
            tracks[a.joint].rotation.times.push_back(time);
            tracks[a.joint].rotation.values.push_back(sagittal(degrees));
        }
    }
    for (const auto& [time, height] : pelvisHeight) {
        tracks[kPelvis].translation.times.push_back(time);
        tracks[kPelvis].translation.values.push_back({0.0f, height, 0.0f});
    }
    anim::AnimationClip clip(name, duration, std::move(tracks));
    clip.setKind(kind);
    return clip;
}

// Right arm in guard (sword up and forward): the start / end pose of the strike.
std::vector<AngleKeys> guardArm(f32 duration)
{
    return {{kUpperArmR, {{0.0f, 45.0f}, {duration, 45.0f}}},
            {kForearmR, {{0.0f, 75.0f}, {duration, 75.0f}}},
            {kHandR, {{0.0f, 25.0f}, {duration, 25.0f}}}};
}

} // namespace

glm::quat sagittal(f32 degrees) { return glm::angleAxis(glm::radians(degrees), kSagittalAxis); }

f32 sagittalAngle(const glm::quat& q)
{
    // Twist about -X: 2 atan2(dot(q.xyz, axis), w).
    return 2.0f * std::atan2(-q.x, q.w);
}

const std::vector<Part>& parts()
{
    static const std::vector<Part> p = makeParts();
    return p;
}

std::shared_ptr<anim::SkinnedModel> buildModel()
{
    const auto& p = parts();
    auto model = std::make_shared<anim::SkinnedModel>();
    model->name = "Character";

    std::vector<std::string> names;
    std::vector<i32> parents;
    anim::Pose rest(kJointCount);
    std::vector<glm::mat4> inverseBind;
    for (u32 j = 0; j < kJointCount; ++j) {
        names.emplace_back(p[j].name);
        parents.push_back(p[j].parent);
        const glm::vec3 parentPosition = p[j].parent >= 0 ? p[static_cast<usize>(p[j].parent)].restPosition : glm::vec3(0.0f);
        rest.translations[j] = p[j].restPosition - parentPosition;
        inverseBind.push_back(glm::translate(glm::mat4(1.0f), -p[j].restPosition));
        model->isBone.push_back(j == kRoot ? 0 : 1);

        if (p[j].mass <= 0.0f) {
            continue;
        }
        // Rigid skinning: every vertex of a part follows its bone.
        const MeshData mesh = partMesh(j, p[j]);
        anim::SkinnedMesh& skin = model->skin;
        const auto base = static_cast<u32>(skin.mesh.positions.size());
        for (usize v = 0; v < mesh.positions.size(); ++v) {
            skin.mesh.positions.push_back(mesh.positions[v] + p[j].restPosition);
            skin.mesh.normals.push_back(mesh.normals[v]);
            skin.joints.emplace_back(j, 0u, 0u, 0u);
            skin.weights.emplace_back(1.0f, 0.0f, 0.0f, 0.0f);
        }
        for (const u32 i : mesh.indices) {
            skin.mesh.indices.push_back(base + i);
        }
    }
    model->skeleton = anim::Skeleton(std::move(names), std::move(parents), std::move(rest), std::move(inverseBind));

    // --- Idle (kinematic): breathing, left arm relaxed, sword in guard ---
    std::vector<AngleKeys> idle = guardArm(2.0f);
    idle.push_back({kChest, {{0.0f, 2.0f}, {1.0f, 3.5f}, {2.0f, 2.0f}}});
    idle.push_back({kHead, {{0.0f, -2.0f}, {1.0f, -3.0f}, {2.0f, -2.0f}}});
    idle.push_back({kUpperArmL, {{0.0f, 5.0f}, {1.0f, 7.0f}, {2.0f, 5.0f}}});
    idle.push_back({kForearmL, {{0.0f, 15.0f}, {2.0f, 15.0f}}});
    idle.push_back({kShinR, {{0.0f, -4.0f}, {2.0f, -4.0f}}});
    idle.push_back({kShinL, {{0.0f, -4.0f}, {2.0f, -4.0f}}});
    idle.push_back({kThighR, {{0.0f, 2.0f}, {2.0f, 2.0f}}});
    idle.push_back({kThighL, {{0.0f, 2.0f}, {2.0f, 2.0f}}});

    // --- Walk (kinematic): one cycle = two steps of 0.8 m in 1 s (1.6 m/s), in place ---
    std::vector<AngleKeys> walk = guardArm(1.0f);
    walk.push_back({kThighR, {{0.0f, 25.0f}, {0.25f, 5.0f}, {0.5f, -20.0f}, {0.75f, 5.0f}, {1.0f, 25.0f}}});
    walk.push_back({kShinR, {{0.0f, -5.0f}, {0.25f, -15.0f}, {0.5f, -10.0f}, {0.75f, -55.0f}, {1.0f, -5.0f}}});
    walk.push_back({kFootR, {{0.0f, 10.0f}, {0.25f, 0.0f}, {0.5f, -15.0f}, {0.75f, 5.0f}, {1.0f, 10.0f}}});
    walk.push_back({kThighL, {{0.0f, -20.0f}, {0.25f, 5.0f}, {0.5f, 25.0f}, {0.75f, 5.0f}, {1.0f, -20.0f}}});
    walk.push_back({kShinL, {{0.0f, -10.0f}, {0.25f, -55.0f}, {0.5f, -5.0f}, {0.75f, -15.0f}, {1.0f, -10.0f}}});
    walk.push_back({kFootL, {{0.0f, -15.0f}, {0.25f, 5.0f}, {0.5f, 10.0f}, {0.75f, 0.0f}, {1.0f, -15.0f}}});
    walk.push_back({kUpperArmL, {{0.0f, 18.0f}, {0.5f, -18.0f}, {1.0f, 18.0f}}}); // opposite to the left leg
    walk.push_back({kForearmL, {{0.0f, 30.0f}, {0.5f, 15.0f}, {1.0f, 30.0f}}});
    walk.push_back({kChest, {{0.0f, 5.0f}, {0.25f, 4.0f}, {0.5f, 5.0f}, {0.75f, 4.0f}, {1.0f, 5.0f}}});
    walk.push_back({kHead, {{0.0f, -4.0f}, {1.0f, -4.0f}}});
    const std::vector<std::pair<f32, f32>> bob = {{0.0f, 0.935f}, {0.25f, 0.965f}, {0.5f, 0.935f}, {0.75f, 0.965f}, {1.0f, 0.935f}};

    // --- Strike (physics): sword arm only; the muscles follow it, the simulation decides ---
    const std::vector<AngleKeys> strike = {
        {kUpperArmR, {{0.00f, 45.0f}, {0.45f, 160.0f}, {0.62f, 150.0f}, {0.78f, 95.0f}, {0.95f, 40.0f}, {1.50f, 45.0f}}},
        {kForearmR, {{0.00f, 75.0f}, {0.45f, 105.0f}, {0.62f, 60.0f}, {0.78f, 15.0f}, {0.95f, 10.0f}, {1.50f, 75.0f}}},
        {kHandR, {{0.00f, 25.0f}, {0.45f, 45.0f}, {0.62f, 20.0f}, {0.78f, -5.0f}, {0.95f, -15.0f}, {1.50f, 25.0f}}},
    };

    model->clips.push_back(makeClip("Idle", 2.0f, idle, anim::ClipKind::Kinematic));
    model->clips.push_back(makeClip("Walk", 1.0f, walk, anim::ClipKind::Kinematic, bob));
    model->clips.push_back(makeClip("Strike", 1.5f, strike, anim::ClipKind::Physics));
    anim::computeRestBounds(*model);
    // In the rest pose the sword hangs into the floor; previews and placement should stand on the feet.
    model->restBoundsMin.y = 0.0f;
    return model;
}

} // namespace kinetic::editor::character
