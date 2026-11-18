#include <doctest/doctest.h>

#include <kinetic/core/Primitives.h>
#include <kinetic/physics/PhysicsWorld.h>

#include <glm/gtc/quaternion.hpp>

using namespace kinetic;
using namespace kinetic::physics;

namespace {

constexpr f32 kDt = 1.0f / 240.0f;
constexpr f32 kG = 9.81f;

void checkOutwardWinding(const MeshData& mesh, const glm::vec3& center)
{
    for (usize t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::vec3& a = mesh.positions[mesh.indices[t]];
        const glm::vec3& b = mesh.positions[mesh.indices[t + 1]];
        const glm::vec3& c = mesh.positions[mesh.indices[t + 2]];
        const glm::vec3 n = glm::cross(b - a, c - a);
        if (glm::dot(n, n) < 1e-12f) {
            continue; // degenerate pole triangle
        }
        CHECK(glm::dot(n, (a + b + c) / 3.0f - center) > 0.0f);
    }
}

// Static anchor + one dynamic rod hanging from a hinge at the origin, pointing along +X
// (horizontal), hinge axis Z. Rod: mass m, length L, uniform -> centre of mass at L/2.
struct HorizontalArm {
    PhysicsWorld world;
    BodyId anchor;
    BodyId rod;
    JointId shoulder;
    f32 mass = 2.0f;
    f32 length = 0.6f;

    explicit HorizontalArm(const Muscle& muscle)
    {
        REQUIRE(world.init());
        world.setGravity({0.0f, -kG, 0.0f});
        BodyDesc a;
        a.shape.halfExtents = glm::vec3(0.05f);
        a.motion = MotionType::Static;
        a.collisionGroup = 1;
        anchor = world.addBody(a);

        BodyDesc r;
        r.shape.type = ShapeType::Box;
        r.shape.halfExtents = {length * 0.5f, 0.02f, 0.02f};
        r.position = {length * 0.5f, 0.0f, 0.0f};
        r.mass = mass;
        r.collisionGroup = 1;
        rod = world.addBody(r);

        HingeDesc h;
        h.parent = anchor;
        h.child = rod;
        h.anchor = {0.0f, 0.0f, 0.0f};
        h.axis = {0.0f, 0.0f, 1.0f};
        h.referenceParent = {1.0f, 0.0f, 0.0f};
        h.referenceChild = {1.0f, 0.0f, 0.0f};
        h.muscle = muscle;
        shoulder = world.addHinge(h);
    }

    void run(f32 seconds)
    {
        for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
            world.step(kDt);
        }
    }
};

} // namespace

TEST_CASE("Primitive meshes wind counter-clockwise seen from outside")
{
    const MeshData box = makeBoxMesh({0.5f, 1.0f, 0.25f}, {1, 2, 3});
    CHECK(box.triangleCount() == 12);
    checkOutwardWinding(box, {1, 2, 3});
    CHECK(box.boundsMax().y == doctest::Approx(3.0f));

    const MeshData capsule = makeCapsuleMesh(0.1f, 0.3f);
    checkOutwardWinding(capsule, {0, 0, 0});
    CHECK(capsule.boundsMax().y == doctest::Approx(0.4f));
    CHECK(capsule.boundsMin().y == doctest::Approx(-0.4f));
}

TEST_CASE("Hinge angle is the child's rotation about the axis relative to the parent")
{
    HorizontalArm arm(Muscle{.stiffness = 0.0f, .damping = 0.0f, .maxTorque = 0.0f, .enabled = false});
    CHECK(arm.world.joint(arm.shoulder).angle == doctest::Approx(0.0f).epsilon(1e-4));
    arm.run(0.1f); // falls under gravity: rotates clockwise about +Z, i.e. negative angle
    CHECK(arm.world.joint(arm.shoulder).angle < -0.01f);
    CHECK(arm.world.joint(arm.shoulder).angularVelocity < 0.0f);
}

TEST_CASE("A muscle holding a horizontal arm carries m g L/2 (J^T F), the joint carries m g")
{
    HorizontalArm arm(Muscle{.stiffness = 4000.0f, .damping = 100.0f, .maxTorque = 1000.0f});
    arm.world.setTargetAngle(arm.shoulder, 0.0f);
    arm.run(2.0f);

    const JointState& s = arm.world.joint(arm.shoulder);
    const f32 expectedTorque = arm.mass * kG * arm.length * 0.5f; // gravity moment about the hinge
    CHECK(s.angle == doctest::Approx(0.0f).epsilon(0.01));
    // The muscle (plus the small spring error) must lift the rod: positive torque about +Z.
    CHECK(s.muscleTorque + s.limitTorque == doctest::Approx(expectedTorque).epsilon(0.02));
    // The anchor pushes the rod up with its full weight.
    CHECK(s.force.y == doctest::Approx(arm.mass * kG).epsilon(0.02));
    CHECK(std::abs(s.force.x) < 0.05f);
}

TEST_CASE("A weak muscle saturates at its strength and the arm sags")
{
    const f32 strength = 2.0f; // far less than m g L/2 = 5.9 N m
    HorizontalArm arm(Muscle{.stiffness = 4000.0f, .damping = 100.0f, .maxTorque = strength});
    arm.world.setTargetAngle(arm.shoulder, 0.0f);
    arm.run(0.25f);
    const JointState& s = arm.world.joint(arm.shoulder);
    CHECK(s.angle < -0.05f);
    CHECK(std::abs(s.muscleTorque) == doctest::Approx(strength).epsilon(0.01));
}

TEST_CASE("Contact force on a resting box is reconstructed as its weight")
{
    PhysicsWorld world;
    REQUIRE(world.init());
    world.setGravity({0.0f, -kG, 0.0f});
    BodyDesc floor;
    floor.shape.halfExtents = {5.0f, 0.5f, 5.0f};
    floor.position = {0.0f, -0.5f, 0.0f};
    floor.motion = MotionType::Static;
    world.addBody(floor);

    BodyDesc box;
    box.shape.halfExtents = glm::vec3(0.25f);
    box.position = {0.0f, 0.25f, 0.0f};
    box.mass = 10.0f;
    const BodyId id = world.addBody(box);
    for (int i = 0; i < 240; ++i) {
        world.step(kDt);
    }
    const BodyState& s = world.body(id);
    REQUIRE(s.inContact);
    CHECK(s.contactForce.y == doctest::Approx(10.0f * kG).epsilon(0.01));
    CHECK(std::abs(s.contactPoint.y) < 0.05f);
    CHECK_FALSE(world.contacts().empty());
}

TEST_CASE("Kinematic bodies follow targets, switch to dynamic, swing-twist joints hold")
{
    PhysicsWorld world;
    REQUIRE(world.init());
    world.setGravity({0.0f, -kG, 0.0f});

    BodyDesc parentDesc;
    parentDesc.shape.halfExtents = glm::vec3(0.1f);
    parentDesc.position = {0.0f, 2.0f, 0.0f};
    parentDesc.motion = MotionType::Kinematic;
    parentDesc.mass = 5.0f;
    parentDesc.collisionGroup = 7;
    const BodyId parent = world.addBody(parentDesc);

    BodyDesc childDesc;
    childDesc.shape.type = ShapeType::Capsule;
    childDesc.shape.radius = 0.05f;
    childDesc.shape.halfHeight = 0.15f;
    childDesc.shape.offset = {0.0f, -0.2f, 0.0f};
    childDesc.position = {0.0f, 1.9f, 0.0f};
    childDesc.motion = MotionType::Kinematic;
    childDesc.mass = 2.0f;
    childDesc.collisionGroup = 7;
    const BodyId child = world.addBody(childDesc);
    CHECK(world.body(child).mass == doctest::Approx(2.0f));

    SwingTwistDesc joint;
    joint.parent = parent;
    joint.child = child;
    joint.anchor = {0.0f, 1.9f, 0.0f};
    joint.twistAxis = {0.0f, -1.0f, 0.0f};
    joint.planeAxis = {1.0f, 0.0f, 0.0f};
    const JointId j = world.addSwingTwist(joint);
    REQUIRE(j.valid());

    // Kinematic: moves to its target exactly, ignores gravity.
    for (int i = 0; i < 24; ++i) {
        world.moveKinematic(parent, {0.0f, 2.0f, 0.1f * static_cast<f32>(i + 1) / 24.0f}, glm::quat(1, 0, 0, 0), kDt);
        world.moveKinematic(child, {0.0f, 1.9f, 0.1f * static_cast<f32>(i + 1) / 24.0f}, glm::quat(1, 0, 0, 0), kDt);
        world.step(kDt);
    }
    CHECK(world.body(parent).position.z == doctest::Approx(0.1f).epsilon(0.01));
    CHECK(world.body(child).position.y == doctest::Approx(1.9f).epsilon(0.001));

    // Dynamic child: hangs from the kinematic parent; the joint carries its weight.
    world.setMotionType(child, MotionType::Dynamic);
    for (int i = 0; i < 480; ++i) {
        world.step(kDt);
    }
    CHECK(world.body(child).position.y == doctest::Approx(1.9f).epsilon(0.01)); // still attached
    CHECK(world.joint(j).force.y == doctest::Approx(2.0f * kG).epsilon(0.05));

    // Teleport puts a body anywhere at rest.
    world.setMotionType(parent, MotionType::Dynamic);
    world.teleport(parent, {3.0f, 5.0f, 0.0f}, glm::quat(1, 0, 0, 0));
    CHECK(world.body(parent).position.x == doctest::Approx(3.0f));
}
