#include "kinetic/physics/PhysicsWorld.h"

#include "kinetic/core/Assert.h"
#include "kinetic/core/Log.h"

// Jolt.h must come first.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <unordered_map>

namespace kinetic::physics {

namespace {

// --- Jolt global setup (once per process, ref-counted) -------------------------------------------

int g_joltUsers = 0;

void joltTrace(const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    KN_INFO("Jolt: {}", buffer);
}

#ifdef JPH_ENABLE_ASSERTS
bool joltAssert(const char* expression, const char* message, const char* file, JPH::uint line)
{
    KN_ERROR("Jolt assert: {} {} ({}:{})", expression, message ? message : "", file, line);
    return true; // break into the debugger
}
#endif

void acquireJolt()
{
    if (g_joltUsers++ == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = joltTrace;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = joltAssert;)
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

void releaseJolt()
{
    if (--g_joltUsers == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

// --- Conversions -----------------------------------------------------------------------------------

JPH::Vec3 toJolt(const glm::vec3& v) { return {v.x, v.y, v.z}; }
JPH::Quat toJolt(const glm::quat& q) { return {q.x, q.y, q.z, q.w}; }
glm::vec3 toGlm(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
glm::quat toGlm(JPH::QuatArg q) { return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; }

// --- Layers ------------------------------------------------------------------------------------------

constexpr JPH::ObjectLayer kLayerStatic = 0;
constexpr JPH::ObjectLayer kLayerMoving = 1;
constexpr JPH::uint kNumLayers = 2;

// Bodies with the same (non-invalid) group id never collide: one group per rig.
class RigGroupFilter final : public JPH::GroupFilter {
public:
    bool CanCollide(const JPH::CollisionGroup& a, const JPH::CollisionGroup& b) const override
    {
        return a.GetGroupID() != b.GetGroupID();
    }
};

// Collects contact points of the current step (single-threaded job system, but Jolt documents
// the callbacks as possibly concurrent, so keep the mutex).
class ContactCollector final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold,
                        JPH::ContactSettings&) override
    {
        record(a, b, manifold);
    }
    void OnContactPersisted(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold,
                            JPH::ContactSettings&) override
    {
        record(a, b, manifold);
    }

    std::vector<ContactPoint> points;
    std::mutex mutex;

private:
    void record(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold)
    {
        const std::scoped_lock lock(mutex);
        for (JPH::uint i = 0; i < manifold.mRelativeContactPointsOn1.size(); ++i) {
            ContactPoint p;
            p.bodyA = BodyId{static_cast<u32>(a.GetUserData())};
            p.bodyB = BodyId{static_cast<u32>(b.GetUserData())};
            // Midpoint between the two surfaces.
            p.position = (toGlm(JPH::Vec3(manifold.GetWorldSpaceContactPointOn1(i))) +
                          toGlm(JPH::Vec3(manifold.GetWorldSpaceContactPointOn2(i)))) * 0.5f;
            p.normal = toGlm(manifold.mWorldSpaceNormal);
            p.depth = manifold.mPenetrationDepth;
            points.push_back(p);
        }
    }
};

JPH::ShapeRefC createShape(const ShapeDesc& desc, const glm::vec3& centerOfMass)
{
    JPH::ShapeSettings::ShapeResult result;
    switch (desc.type) {
    case ShapeType::Box: {
        const f32 minHalf = std::min({desc.halfExtents.x, desc.halfExtents.y, desc.halfExtents.z});
        result = JPH::BoxShapeSettings(toJolt(desc.halfExtents), std::min(0.02f, minHalf * 0.5f)).Create();
        break;
    }
    case ShapeType::Sphere:
        result = JPH::SphereShapeSettings(desc.radius).Create();
        break;
    case ShapeType::Capsule:
        result = JPH::CapsuleShapeSettings(desc.halfHeight, desc.radius).Create();
        break;
    }
    if (result.HasError()) {
        KN_ERROR("Physics shape: {}", result.GetError().c_str());
        return nullptr;
    }
    JPH::ShapeRefC shape = result.Get();
    if (desc.offset != glm::vec3(0.0f)) {
        shape = JPH::RotatedTranslatedShapeSettings(toJolt(desc.offset), JPH::Quat::sIdentity(), shape).Create().Get();
    }
    if (centerOfMass != glm::vec3(0.0f)) {
        shape = JPH::OffsetCenterOfMassShapeSettings(toJolt(centerOfMass), shape).Create().Get();
    }
    return shape;
}

} // namespace

// ---------------------------------------------------------------------------------------------------

struct PhysicsWorld::Impl {
    struct Body {
        BodyDesc desc;
        JPH::BodyID id;
        BodyState state;
        glm::vec3 velocityBefore{0.0f};
    };
    enum class JointType : u8 { Hinge, Fixed, SwingTwist };
    struct Joint {
        std::string name;
        BodyId parent;
        BodyId child;
        JPH::Ref<JPH::TwoBodyConstraint> constraint;
        bool hinge = true;
        JointType type = JointType::Hinge;
        Muscle muscle;
        f32 target = 0.0f;
        JointState state;
    };

    JPH::BroadPhaseLayerInterfaceTable broadPhaseLayers{kNumLayers, 2};
    JPH::ObjectLayerPairFilterTable layerPairs{kNumLayers};
    std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilterTable> objectVsBroadPhase;
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;
    std::unique_ptr<JPH::JobSystemSingleThreaded> jobs;
    std::unique_ptr<JPH::PhysicsSystem> system;
    JPH::Ref<RigGroupFilter> groupFilter = new RigGroupFilter();
    ContactCollector contactCollector;

    std::vector<Body> bodies;
    std::vector<Joint> joints;
    std::vector<ContactPoint> contacts;
    f32 lastDt = 0.0f;
    bool initialized = false;

    void applyMuscle(Joint& joint)
    {
        auto* hinge = static_cast<JPH::HingeConstraint*>(joint.constraint.GetPtr());
        JPH::MotorSettings& motor = hinge->GetMotorSettings();
        motor.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::StiffnessAndDamping,
                                                    std::max(joint.muscle.stiffness, 0.0f),
                                                    std::max(joint.muscle.damping, 0.0f));
        motor.SetTorqueLimit(std::max(joint.muscle.maxTorque, 0.0f));
        const bool on = joint.muscle.enabled && joint.muscle.stiffness > 0.0f;
        hinge->SetMotorState(on ? JPH::EMotorState::Position : JPH::EMotorState::Off);
        hinge->SetTargetAngle(joint.target);
    }

    void readStates(f32 dt)
    {
        JPH::BodyInterface& bi = system->GetBodyInterfaceNoLock();
        const glm::vec3 g = toGlm(system->GetGravity());

        for (Body& body : bodies) {
            BodyState& s = body.state;
            s.position = toGlm(JPH::Vec3(bi.GetPosition(body.id)));
            s.rotation = toGlm(bi.GetRotation(body.id));
            s.centerOfMass = toGlm(JPH::Vec3(bi.GetCenterOfMassPosition(body.id)));
            s.linearVelocity = toGlm(bi.GetLinearVelocity(body.id));
            s.angularVelocity = toGlm(bi.GetAngularVelocity(body.id));
            s.contactForce = glm::vec3(0.0f);
            s.inContact = false;
        }

        for (Joint& joint : joints) {
            JointState& s = joint.state;
            const JPH::Body& b1 = *joint.constraint->GetBody1();
            s.anchor = toGlm(JPH::Vec3(b1.GetCenterOfMassTransform() * joint.constraint->GetConstraintToBody1Matrix().GetTranslation()));
            if (joint.hinge) {
                auto* hinge = static_cast<JPH::HingeConstraint*>(joint.constraint.GetPtr());
                s.axis = toGlm(b1.GetRotation() * hinge->GetLocalSpaceHingeAxis1());
                s.angle = hinge->GetCurrentAngle();
                s.targetAngle = joint.target;
                s.angularVelocity =
                    glm::dot(s.axis, bodies[joint.child.index].state.angularVelocity - bodies[joint.parent.index].state.angularVelocity);
                s.force = toGlm(hinge->GetTotalLambdaPosition()) / dt;
                s.muscleTorque = hinge->GetTotalLambdaMotor() / dt;
                s.limitTorque = hinge->GetTotalLambdaRotationLimits() / dt;
                s.torque = s.axis * (s.muscleTorque + s.limitTorque);
            } else if (joint.type == JointType::Fixed) {
                auto* fixed = static_cast<JPH::FixedConstraint*>(joint.constraint.GetPtr());
                s.force = toGlm(fixed->GetTotalLambdaPosition()) / dt;
                s.torque = toGlm(fixed->GetTotalLambdaRotation()) / dt;
            } else {
                auto* swing = static_cast<JPH::SwingTwistConstraint*>(joint.constraint.GetPtr());
                s.force = toGlm(swing->GetTotalLambdaPosition()) / dt;
            }
        }

        // Contact force on each dynamic body = momentum change - gravity - joint forces. The
        // solver applies exactly these impulses (damping is 0 by default), so this is exact.
        std::vector<glm::vec3> jointImpulse(bodies.size(), glm::vec3(0.0f));
        for (const Joint& joint : joints) {
            jointImpulse[joint.child.index] += joint.state.force * dt;
            jointImpulse[joint.parent.index] -= joint.state.force * dt;
        }
        std::vector<glm::vec3> pointSum(bodies.size(), glm::vec3(0.0f));
        std::vector<int> pointCount(bodies.size(), 0);
        for (const ContactPoint& c : contacts) {
            for (const BodyId id : {c.bodyA, c.bodyB}) {
                if (id.index < bodies.size()) {
                    pointSum[id.index] += c.position;
                    ++pointCount[id.index];
                }
            }
        }
        for (usize i = 0; i < bodies.size(); ++i) {
            Body& body = bodies[i];
            if (pointCount[i] == 0) {
                continue;
            }
            body.state.inContact = true;
            body.state.contactPoint = pointSum[i] / static_cast<f32>(pointCount[i]);
            if (body.desc.motion == MotionType::Dynamic) {
                const glm::vec3 momentumChange = body.state.mass * (body.state.linearVelocity - body.velocityBefore);
                const glm::vec3 impulse = momentumChange - body.state.mass * g * dt - jointImpulse[i];
                body.state.contactForce = impulse / dt;
            }
        }
    }
};

// Jolt must be set up (allocator, factory) before Impl is built: its members allocate through Jolt.
PhysicsWorld::PhysicsWorld()
{
    acquireJolt();
    m_impl = std::make_unique<Impl>();
}

PhysicsWorld::~PhysicsWorld()
{
    shutdown();
    m_impl.reset();
    releaseJolt();
}

bool PhysicsWorld::init(u32 maxBodies)
{
    if (m_impl->initialized) {
        return true;
    }
    Impl& w = *m_impl;
    w.broadPhaseLayers.MapObjectToBroadPhaseLayer(kLayerStatic, JPH::BroadPhaseLayer(0));
    w.broadPhaseLayers.MapObjectToBroadPhaseLayer(kLayerMoving, JPH::BroadPhaseLayer(1));
    w.layerPairs.EnableCollision(kLayerStatic, kLayerMoving);
    w.layerPairs.EnableCollision(kLayerMoving, kLayerMoving);
    w.objectVsBroadPhase =
        std::make_unique<JPH::ObjectVsBroadPhaseLayerFilterTable>(w.broadPhaseLayers, 2, w.layerPairs, kNumLayers);

    w.tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(16 * 1024 * 1024);
    w.jobs = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    w.system = std::make_unique<JPH::PhysicsSystem>();
    w.system->Init(maxBodies, 0, maxBodies * 4, maxBodies * 4, w.broadPhaseLayers, *w.objectVsBroadPhase, w.layerPairs);
    w.system->SetContactListener(&w.contactCollector);

    // More solver iterations than the game default: forces through joint chains converge better.
    JPH::PhysicsSettings settings;
    settings.mNumVelocitySteps = 24;
    settings.mNumPositionSteps = 6;
    // Default slop (2 cm) lets resting / pressed contacts sink that deep before being pushed out;
    // for measuring we want contacts at the surface.
    settings.mPenetrationSlop = 0.002f;
    w.system->SetPhysicsSettings(settings);
    w.initialized = true;
    return true;
}

void PhysicsWorld::shutdown()
{
    if (!m_impl || !m_impl->initialized) {
        return;
    }
    clear();
    m_impl->system.reset();
    m_impl->jobs.reset();
    m_impl->tempAllocator.reset();
    m_impl->objectVsBroadPhase.reset();
    m_impl->initialized = false;
}

void PhysicsWorld::clear()
{
    Impl& w = *m_impl;
    if (!w.initialized) {
        return;
    }
    for (Impl::Joint& joint : w.joints) {
        w.system->RemoveConstraint(joint.constraint);
    }
    w.joints.clear();
    JPH::BodyInterface& bi = w.system->GetBodyInterface();
    for (const Impl::Body& body : w.bodies) {
        bi.RemoveBody(body.id);
        bi.DestroyBody(body.id);
    }
    w.bodies.clear();
    w.contacts.clear();
}

BodyId PhysicsWorld::addBody(const BodyDesc& desc)
{
    Impl& w = *m_impl;
    KN_ASSERT(w.initialized);
    const JPH::ShapeRefC shape = createShape(desc.shape, desc.centerOfMass);
    if (!shape) {
        return {};
    }
    const JPH::EMotionType motion = desc.motion == MotionType::Static      ? JPH::EMotionType::Static
                                    : desc.motion == MotionType::Kinematic ? JPH::EMotionType::Kinematic
                                                                           : JPH::EMotionType::Dynamic;
    JPH::BodyCreationSettings settings(shape, JPH::RVec3(toJolt(desc.position)), toJolt(desc.rotation), motion,
                                       desc.motion == MotionType::Static ? kLayerStatic : kLayerMoving);
    settings.mFriction = desc.friction;
    settings.mRestitution = desc.restitution;
    settings.mLinearDamping = desc.linearDamping;
    settings.mAngularDamping = desc.angularDamping;
    settings.mMaxAngularVelocity = 500.0f; // don't clamp fast swings (it would also break force reconstruction)
    settings.mAllowSleeping = false;
    settings.mMotionQuality = desc.continuousCollision ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
    if (desc.mass > 0.0f) {
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = desc.mass;
    }
    settings.mAllowDynamicOrKinematic = desc.motion != MotionType::Static;
    if (desc.collisionGroup != 0) {
        settings.mCollisionGroup = JPH::CollisionGroup(w.groupFilter.GetPtr(), desc.collisionGroup, 0);
    }
    const auto index = static_cast<u32>(w.bodies.size());
    settings.mUserData = index;

    JPH::BodyInterface& bi = w.system->GetBodyInterface();
    const JPH::BodyID id = bi.CreateAndAddBody(settings, JPH::EActivation::Activate);
    if (id.IsInvalid()) {
        KN_ERROR("Physics: too many bodies");
        return {};
    }
    Impl::Body body;
    body.desc = desc;
    body.id = id;
    if (desc.motion != MotionType::Static) {
        const JPH::BodyLockRead lock(w.system->GetBodyLockInterface(), id);
        const f32 inverseMass = lock.GetBody().GetMotionPropertiesUnchecked()->GetInverseMassUnchecked();
        body.state.mass = inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f;
    }
    w.bodies.push_back(body);
    w.readStates(1.0f);
    return BodyId{index};
}

JointId PhysicsWorld::addHinge(const HingeDesc& desc)
{
    Impl& w = *m_impl;
    JPH::HingeConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::WorldSpace;
    settings.mPoint1 = settings.mPoint2 = JPH::RVec3(toJolt(desc.anchor));
    settings.mHingeAxis1 = settings.mHingeAxis2 = toJolt(glm::normalize(desc.axis));
    settings.mNormalAxis1 = toJolt(glm::normalize(desc.referenceParent));
    settings.mNormalAxis2 = toJolt(glm::normalize(desc.referenceChild));
    settings.mLimitsMin = desc.limitMin;
    settings.mLimitsMax = desc.limitMax;
    settings.mMaxFrictionTorque = desc.friction;

    JPH::TwoBodyConstraint* constraint = w.system->GetBodyInterface().CreateConstraint(
        &settings, w.bodies[desc.parent.index].id, w.bodies[desc.child.index].id);
    if (!constraint) {
        return {};
    }
    w.system->AddConstraint(constraint);

    Impl::Joint joint;
    joint.name = desc.name;
    joint.parent = desc.parent;
    joint.child = desc.child;
    joint.constraint = constraint;
    joint.hinge = true;
    joint.muscle = desc.muscle;
    joint.target = static_cast<JPH::HingeConstraint*>(constraint)->GetCurrentAngle();
    w.joints.push_back(joint);
    w.applyMuscle(w.joints.back());
    return JointId{static_cast<u32>(w.joints.size() - 1)};
}

JointId PhysicsWorld::addFixed(const FixedDesc& desc)
{
    Impl& w = *m_impl;
    JPH::FixedConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::WorldSpace;
    settings.mAutoDetectPoint = false;
    settings.mPoint1 = settings.mPoint2 = JPH::RVec3(toJolt(desc.anchor));

    JPH::TwoBodyConstraint* constraint = w.system->GetBodyInterface().CreateConstraint(
        &settings, w.bodies[desc.parent.index].id, w.bodies[desc.child.index].id);
    if (!constraint) {
        return {};
    }
    w.system->AddConstraint(constraint);

    Impl::Joint joint;
    joint.name = desc.name;
    joint.parent = desc.parent;
    joint.child = desc.child;
    joint.constraint = constraint;
    joint.hinge = false;
    joint.type = Impl::JointType::Fixed;
    joint.muscle.enabled = false;
    joint.state.hinge = false;
    w.joints.push_back(joint);
    return JointId{static_cast<u32>(w.joints.size() - 1)};
}

JointId PhysicsWorld::addSwingTwist(const SwingTwistDesc& desc)
{
    Impl& w = *m_impl;
    JPH::SwingTwistConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::WorldSpace;
    settings.mPosition1 = settings.mPosition2 = JPH::RVec3(toJolt(desc.anchor));
    settings.mTwistAxis1 = settings.mTwistAxis2 = toJolt(glm::normalize(desc.twistAxis));
    settings.mPlaneAxis1 = settings.mPlaneAxis2 = toJolt(glm::normalize(desc.planeAxis));
    settings.mNormalHalfConeAngle = desc.normalHalfCone;
    settings.mPlaneHalfConeAngle = desc.planeHalfCone;
    settings.mTwistMinAngle = desc.twistMin;
    settings.mTwistMaxAngle = desc.twistMax;
    settings.mMaxFrictionTorque = desc.friction;

    JPH::TwoBodyConstraint* constraint = w.system->GetBodyInterface().CreateConstraint(
        &settings, w.bodies[desc.parent.index].id, w.bodies[desc.child.index].id);
    if (!constraint) {
        return {};
    }
    w.system->AddConstraint(constraint);

    Impl::Joint joint;
    joint.name = desc.name;
    joint.parent = desc.parent;
    joint.child = desc.child;
    joint.constraint = constraint;
    joint.hinge = false;
    joint.type = Impl::JointType::SwingTwist;
    joint.muscle.enabled = false;
    joint.state.hinge = false;
    w.joints.push_back(joint);
    return JointId{static_cast<u32>(w.joints.size() - 1)};
}

void PhysicsWorld::setMotionType(BodyId body, MotionType motion)
{
    Impl::Body& b = m_impl->bodies[body.index];
    if (b.desc.motion == MotionType::Static || motion == MotionType::Static || b.desc.motion == motion) {
        return;
    }
    b.desc.motion = motion;
    m_impl->system->GetBodyInterface().SetMotionType(
        b.id, motion == MotionType::Kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic,
        JPH::EActivation::Activate);
}

void PhysicsWorld::moveKinematic(BodyId body, const glm::vec3& position, const glm::quat& rotation, f32 dt)
{
    m_impl->system->GetBodyInterface().MoveKinematic(m_impl->bodies[body.index].id, JPH::RVec3(toJolt(position)),
                                                     toJolt(glm::normalize(rotation)), dt);
}

void PhysicsWorld::teleport(BodyId body, const glm::vec3& position, const glm::quat& rotation)
{
    JPH::BodyInterface& bi = m_impl->system->GetBodyInterface();
    const JPH::BodyID id = m_impl->bodies[body.index].id;
    bi.SetPositionAndRotation(id, JPH::RVec3(toJolt(position)), toJolt(glm::normalize(rotation)), JPH::EActivation::Activate);
    bi.SetLinearAndAngularVelocity(id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    BodyState& state = m_impl->bodies[body.index].state;
    state.position = position;
    state.rotation = rotation;
}

void PhysicsWorld::setMuscle(JointId id, const Muscle& muscle)
{
    Impl::Joint& joint = m_impl->joints[id.index];
    if (joint.hinge) {
        joint.muscle = muscle;
        m_impl->applyMuscle(joint);
    }
}

void PhysicsWorld::setRestitution(BodyId body, f32 restitution)
{
    Impl::Body& b = m_impl->bodies[body.index];
    b.desc.restitution = restitution;
    m_impl->system->GetBodyInterface().SetRestitution(b.id, restitution);
}

void PhysicsWorld::setFriction(BodyId body, f32 friction)
{
    Impl::Body& b = m_impl->bodies[body.index];
    b.desc.friction = friction;
    m_impl->system->GetBodyInterface().SetFriction(b.id, friction);
}

void PhysicsWorld::setTargetAngle(JointId id, f32 angle)
{
    Impl::Joint& joint = m_impl->joints[id.index];
    if (joint.hinge) {
        auto* hinge = static_cast<JPH::HingeConstraint*>(joint.constraint.GetPtr());
        hinge->SetTargetAngle(angle);
        joint.target = hinge->GetTargetAngle(); // clamped to the limits
    }
}

void PhysicsWorld::setGravity(const glm::vec3& gravity)
{
    m_impl->system->SetGravity(toJolt(gravity));
}

glm::vec3 PhysicsWorld::gravity() const
{
    return toGlm(m_impl->system->GetGravity());
}

void PhysicsWorld::step(f32 dt)
{
    Impl& w = *m_impl;
    if (!w.initialized || dt <= 0.0f) {
        return;
    }
    JPH::BodyInterface& bi = w.system->GetBodyInterfaceNoLock();
    for (Impl::Body& body : w.bodies) {
        body.velocityBefore = toGlm(bi.GetLinearVelocity(body.id));
    }
    w.contactCollector.points.clear();
    w.system->Update(dt, 1, w.tempAllocator.get(), w.jobs.get());
    w.contacts = w.contactCollector.points;
    w.lastDt = dt;
    w.readStates(dt);
}

usize PhysicsWorld::bodyCount() const { return m_impl->bodies.size(); }
usize PhysicsWorld::jointCount() const { return m_impl->joints.size(); }
const BodyDesc& PhysicsWorld::bodyDesc(BodyId body) const { return m_impl->bodies[body.index].desc; }
const BodyState& PhysicsWorld::body(BodyId body) const { return m_impl->bodies[body.index].state; }
const std::string& PhysicsWorld::jointName(JointId joint) const { return m_impl->joints[joint.index].name; }
const Muscle& PhysicsWorld::muscle(JointId joint) const { return m_impl->joints[joint.index].muscle; }
const JointState& PhysicsWorld::joint(JointId joint) const { return m_impl->joints[joint.index].state; }
std::span<const ContactPoint> PhysicsWorld::contacts() const { return m_impl->contacts; }
f32 PhysicsWorld::lastStep() const { return m_impl->lastDt; }

} // namespace kinetic::physics
