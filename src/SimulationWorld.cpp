#include "base/sim/SimulationWorld.h"

// Jolt requires this header before all other Jolt headers.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string_view>
#include <utility>
namespace base::sim
{
namespace
{
JPH::Vec3 V(Vec3 v)
{
    return {float(v.x), float(v.y), float(v.z)};
}
Vec3 V(JPH::Vec3 v)
{
    return {v.GetX(), v.GetY(), v.GetZ()};
}
JPH::Quat Q(Quaternion q)
{
    return {float(q.x), float(q.y), float(q.z), float(q.w)};
}
bool Finite(Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && std::abs(v.x) < 1e15 &&
           std::abs(v.y) < 1e15 && std::abs(v.z) < 1e15;
}
void CheckPose(Pose p)
{
    auto q = p.rotation;
    double n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!Finite(p.position) || !std::isfinite(n) || std::abs(n - 1) > 1e-5)
        throw std::invalid_argument("Invalid simulation pose");
}
struct Global
{
    Global()
    {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory;
        JPH::RegisterTypes();
    }
    ~Global()
    {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
};
struct Layers : JPH::BroadPhaseLayerInterface
{
    JPH::uint GetNumBroadPhaseLayers() const override
    {
        return 2;
    }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer l) const override
    {
        return JPH::BroadPhaseLayer(l);
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override
    {
        return "simulation";
    }
#endif
};
struct Pair : JPH::ObjectLayerPairFilter
{
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
    {
        return a || b;
    }
};
struct BroadPair : JPH::ObjectVsBroadPhaseLayerFilter
{
    bool ShouldCollide(JPH::ObjectLayer a, JPH::BroadPhaseLayer b) const override
    {
        return a || b.GetValue();
    }
};
} // namespace
Backend BackendFromEnvironment()
{
    auto *p = std::getenv("FRC_SIM_PHYSICS");
    if (!p || std::string_view(p).empty())
        return Backend::Full3D;
    if (std::string_view(p) == "phoenix")
        return Backend::Phoenix;
    if (std::string_view(p) == "jolt-planar")
        return Backend::Planar;
    if (std::string_view(p) == "jolt-3d")
        return Backend::Full3D;
    throw std::invalid_argument("FRC_SIM_PHYSICS must be phoenix, jolt-planar, or jolt-3d");
}
struct SimulationWorld::Impl
{
    struct Contacts : JPH::ContactListener
    {
        uint32_t manifolds{};
        struct Record
        {
            JPH::BodyID id;
            RigidContacts counts;
        };
        std::array<Record, 32> records{};
        size_t count{};
        void RecordContact(const JPH::Body &body, JPH::SubShapeID sub)
        {
            const auto tag = body.GetShape()->GetSubShapeUserData(sub);
            if (tag != 1 && tag != 2)
                return;
            size_t i = 0;
            while (i < count && records[i].id != body.GetID())
                ++i;
            if (i == count)
            {
                if (count == records.size())
                    throw std::runtime_error("Rigid diagnostics capacity exceeded");
                records[count++] = {body.GetID(), {}};
            }
            if (tag == 1)
                ++records[i].counts.pan;
            else
                ++records[i].counts.backing;
        }
        void OnContactAdded(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m,
                            JPH::ContactSettings &) override
        {
            ++manifolds;
            RecordContact(a, m.mSubShapeID1);
            RecordContact(b, m.mSubShapeID2);
        }
        void OnContactPersisted(const JPH::Body &a, const JPH::Body &b, const JPH::ContactManifold &m,
                                JPH::ContactSettings &) override
        {
            ++manifolds;
            RecordContact(a, m.mSubShapeID1);
            RecordContact(b, m.mSubShapeID2);
        }
    } contacts;
    Backend mode;
    Layers layers;
    Pair pair;
    BroadPair broad;
    JPH::PhysicsSystem physics;
    JPH::TempAllocatorImpl allocator{16 * 1024 * 1024};
    JPH::JobSystemSingleThreaded jobs{2048};
    struct Entry
    {
        JPH::BodyID id;
        uint32_t generation{1};
        bool alive{};
        BodySpec spec;
        bool enabled{true};
    };
    std::vector<Entry> bodies;
    explicit Impl(Backend m) : mode(m)
    {
        physics.Init(4096, 0, 16384, 4096, layers, broad, pair);
        // Jolt's metre-scale defaults allow 20mm speculative contacts and
        // penetration, larger than our 12.7mm ramp lip and 3.175mm pan. Use
        // millimetre-scale contact settings; swept robot bodies cover motion
        // between these narrow detection margins at the fixed 4ms timestep.
        auto settings = physics.GetPhysicsSettings();
        settings.mSpeculativeContactDistance = .001f;
        settings.mPenetrationSlop = .0001f;
        physics.SetPhysicsSettings(settings);
        physics.SetGravity({0, 0, -9.80665f});
        physics.SetContactListener(&contacts);
    }
    Entry &Get(BodyHandle h)
    {
        if (h.index >= bodies.size() || !bodies[h.index].alive || bodies[h.index].generation != h.generation)
            throw std::invalid_argument("Stale simulation body handle");
        return bodies[h.index];
    }
    const Entry &Get(BodyHandle h) const
    {
        return const_cast<Impl *>(this)->Get(h);
    }
    ~Impl()
    {
        auto &bi = physics.GetBodyInterface();
        for (auto &e : bodies)
            if (e.alive)
            {
                if (e.enabled)
                    bi.RemoveBody(e.id);
                bi.DestroyBody(e.id);
            }
    }
};
SimulationWorld::SimulationWorld(Backend m)
{
    if (m == Backend::Phoenix)
        throw std::invalid_argument("Phoenix has no Jolt world");
    static Global global;
    impl_ = std::make_unique<Impl>(m);
}
SimulationWorld::~SimulationWorld() = default;
Backend SimulationWorld::Mode() const
{
    return impl_->mode;
}
uint32_t SimulationWorld::BodyCount() const
{
    return impl_->physics.GetNumBodies();
}
uint32_t SimulationWorld::ContactManifoldCount() const
{
    return impl_->contacts.manifolds;
}
BodyHandle SimulationWorld::CreateBody(const BodySpec &s)
{
    CheckPose(s.pose);
    if (!std::isfinite(s.mass) || s.mass < 0 || !Finite(s.halfExtents) || s.halfExtents.x <= 0 ||
        s.halfExtents.y <= 0 || s.halfExtents.z <= 0 || !std::isfinite(s.radius) || s.radius <= 0 ||
        !Finite(s.inertia) || !Finite(s.centerOfMass) || s.inertia.x < 0 || s.inertia.y < 0 || s.inertia.z < 0 ||
        !std::isfinite(s.friction) || s.friction < 0 || !std::isfinite(s.restitution) || s.restitution < 0 ||
        s.restitution > 1)
        throw std::invalid_argument("Invalid simulation body configuration");
    bool anyInertia = s.inertia.x > 0 || s.inertia.y > 0 || s.inertia.z > 0;
    if (anyInertia &&
        (s.inertia.x <= 0 || s.inertia.y <= 0 || s.inertia.z <= 0 || s.inertia.x > s.inertia.y + s.inertia.z ||
         s.inertia.y > s.inertia.x + s.inertia.z || s.inertia.z > s.inertia.x + s.inertia.y))
        throw std::invalid_argument("Invalid inertia tensor");
    JPH::RefConst<JPH::Shape> shape;
    if (s.shape == BodySpec::Shape::Box)
        shape = new JPH::BoxShape(V(s.halfExtents), float(std::min({0.05, s.halfExtents.x * .5, s.halfExtents.y * .5,
                                                                    s.halfExtents.z * .5})));
    else if (s.shape == BodySpec::Shape::Sphere)
        shape = new JPH::SphereShape(float(s.radius));
    else if (s.shape == BodySpec::Shape::Compound)
    {
        if (s.parts.empty())
            throw std::invalid_argument("Empty robot compound");
        JPH::StaticCompoundShapeSettings compound;
        for (const auto &p : s.parts)
        {
            if (!Finite(p.center) || !Finite(p.halfExtents) || p.halfExtents.x <= 0 || p.halfExtents.y <= 0 ||
                p.halfExtents.z <= 0)
                throw std::invalid_argument("Invalid robot compound part");
            auto *box = new JPH::BoxShape(
                V(p.halfExtents),
                float(std::min({.001, p.halfExtents.x * .1, p.halfExtents.y * .1, p.halfExtents.z * .1})));
            box->SetUserData(p.tag);
            compound.AddShape(V(p.center), JPH::Quat::sIdentity(), box);
        }
        auto result = compound.Create();
        if (result.HasError())
            throw std::invalid_argument(result.GetError().c_str());
        shape = result.Get();
        // Compound COM is geometry-derived; retain the declared body origin.
        shape = new JPH::OffsetCenterOfMassShape(shape, -shape->GetCenterOfMass());
    }
    else if (s.shape == BodySpec::Shape::BumperRing)
    {
        const double t = s.radius;
        if (t >= std::min(s.halfExtents.x, s.halfExtents.y))
            throw std::invalid_argument("Bumper wall thickness closes the centre opening");
        JPH::StaticCompoundShapeSettings ring;
        auto box = [](Vec3 half) {
            return new JPH::BoxShape(V(half), float(std::min({kBumperCornerRadius, half.x*.95, half.y*.95, half.z*.95})));
        };
        // Front/rear span the full width. Side segments meet them without
        // overlap, so the shell has no invisible underbody or doubled corners.
        for (double sign : {-1., 1.})
        {
            ring.AddShape(JPH::Vec3(float(sign*(s.halfExtents.x-t/2)), 0, 0), JPH::Quat::sIdentity(),
                          box({t/2, s.halfExtents.y, s.halfExtents.z}));
            ring.AddShape(JPH::Vec3(0, float(sign*(s.halfExtents.y-t/2)), 0), JPH::Quat::sIdentity(),
                          box({s.halfExtents.x-t, t/2, s.halfExtents.z}));
        }
        auto result = ring.Create();
        if (result.HasError())
            throw std::invalid_argument("Invalid bumper compound: " + std::string(result.GetError().c_str()));
        shape = result.Get();
    }
    else
        throw std::invalid_argument("Unknown simulation body shape");
    if (s.centerOfMass.x || s.centerOfMass.y || s.centerOfMass.z)
        shape = new JPH::OffsetCenterOfMassShape(shape, V(s.centerOfMass));
    JPH::BodyCreationSettings c(shape, JPH::RVec3(V(s.pose.position)), Q(s.pose.rotation),
                                s.mass > 0 ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static, s.mass > 0 ? 1 : 0);
    c.mFriction = float(s.friction);
    c.mRestitution = float(s.restitution);
    // A 4ms drive step can move farther than the plywood thickness. Sweep the
    // robot's hard shell so it meets the rounded lip before deep face overlap
    // changes the collision normal and creates an artificial tripping impulse.
    if (s.mass > 0 && (s.shape == BodySpec::Shape::Compound || s.shape == BodySpec::Shape::BumperRing))
        c.mMotionQuality = JPH::EMotionQuality::LinearCast;
    c.mLinearDamping = 0;
    c.mAngularDamping = 0;
    if (s.mass > 0)
    {
        c.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        c.mMassPropertiesOverride.mMass = float(s.mass);
        if (s.inertia.x > 0 && s.inertia.y > 0 && s.inertia.z > 0)
        {
            c.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
            c.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(V(s.inertia));
        }
        if (s.planar)
            c.mAllowedDOFs = JPH::EAllowedDOFs::Plane2D;
    }
    auto &bi = impl_->physics.GetBodyInterface();
    auto id = bi.CreateAndAddBody(c, JPH::EActivation::Activate);
    if (id.IsInvalid())
        throw std::runtime_error("Jolt body capacity exceeded");
    uint32_t i = 0;
    while (i < impl_->bodies.size() && impl_->bodies[i].alive)
        ++i;
    if (i == impl_->bodies.size())
        impl_->bodies.emplace_back();
    auto &e = impl_->bodies[i];
    e.id = id;
    e.alive = true;
    e.spec = s;
    e.enabled = true;
    return {i, e.generation};
}
void SimulationWorld::DestroyBody(BodyHandle h)
{
    auto &e = impl_->Get(h);
    auto &bi = impl_->physics.GetBodyInterface();
    if (e.enabled)
        bi.RemoveBody(e.id);
    bi.DestroyBody(e.id);
    e.alive = false;
    ++e.generation;
}
BodyState SimulationWorld::ReadBody(BodyHandle h) const
{
    auto &e = impl_->Get(h);
    auto &bi = impl_->physics.GetBodyInterface();
    auto p = bi.GetPosition(e.id);
    auto q = bi.GetRotation(e.id);
    return {{{p.GetX(), p.GetY(), p.GetZ()}, {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}},
            V(bi.GetLinearVelocity(e.id)),
            V(bi.GetAngularVelocity(e.id)),
            e.enabled};
}
void SimulationWorld::SetBodyState(BodyHandle h, const BodyState &s)
{
    CheckPose(s.pose);
    if (!Finite(s.linearVelocity) || !Finite(s.angularVelocity))
        throw std::invalid_argument("Invalid body velocity");
    auto &e = impl_->Get(h);
    auto &bi = impl_->physics.GetBodyInterface();
    bi.SetPositionAndRotation(e.id, JPH::RVec3(V(s.pose.position)), Q(s.pose.rotation), JPH::EActivation::Activate);
    bi.SetLinearAndAngularVelocity(e.id, V(s.linearVelocity), V(s.angularVelocity));
    SetEnabled(h, s.enabled);
}
void SimulationWorld::ClearForces(BodyHandle h)
{
    auto &e = impl_->Get(h);
    for (size_t i = 0; i < impl_->contacts.count; ++i)
        if (impl_->contacts.records[i].id == e.id)
            impl_->contacts.records[i].counts = {};
    if (e.spec.mass > 0)
    {
        JPH::BodyLockWrite lock(impl_->physics.GetBodyLockInterface(), e.id);
        lock.GetBody().ResetForce();
        lock.GetBody().ResetTorque();
    }
}
void SimulationWorld::SetLinearVelocity(BodyHandle h, Vec3 velocity)
{
    if (!Finite(velocity))
        throw std::invalid_argument("Invalid body velocity");
    auto &e = impl_->Get(h);
    if (e.enabled && e.spec.mass > 0)
        impl_->physics.GetBodyInterface().SetLinearVelocity(e.id, V(velocity));
}
void SimulationWorld::SetEnabled(BodyHandle h, bool enable)
{
    auto &e = impl_->Get(h);
    if (enable == e.enabled)
        return;
    auto &bi = impl_->physics.GetBodyInterface();
    if (enable)
        bi.AddBody(e.id, JPH::EActivation::Activate);
    else
        bi.RemoveBody(e.id);
    e.enabled = enable;
}
void SimulationWorld::AddForce(BodyHandle h, Vec3 f, Vec3 p)
{
    if (!Finite(f) || !Finite(p))
        throw std::invalid_argument("Invalid force");
    auto &e = impl_->Get(h);
    if (e.enabled && e.spec.mass > 0)
        impl_->physics.GetBodyInterface().AddForce(e.id, V(f), JPH::RVec3(V(p)));
}
void SimulationWorld::AddTorque(BodyHandle h, Vec3 t)
{
    if (!Finite(t))
        throw std::invalid_argument("Invalid torque");
    auto &e = impl_->Get(h);
    if (e.enabled && e.spec.mass > 0)
        impl_->physics.GetBodyInterface().AddTorque(e.id, V(t));
}
std::optional<RayHit> SimulationWorld::Raycast(Vec3 o, Vec3 d, std::optional<BodyHandle> x) const
{
    if (!Finite(o) || !Finite(d))
        throw std::invalid_argument("Invalid ray");
    JPH::BodyFilter filter;
    JPH::IgnoreSingleBodyFilter exclude(x ? impl_->Get(*x).id : JPH::BodyID());
    JPH::RayCastResult hit;
    JPH::RRayCast ray(JPH::RVec3(V(o)), V(d));
    if (!impl_->physics.GetNarrowPhaseQuery().CastRay(ray, hit, {}, {},
                                                      x ? static_cast<const JPH::BodyFilter &>(exclude) : filter))
        return {};
    for (uint32_t i = 0; i < impl_->bodies.size(); ++i)
    {
        auto &e = impl_->bodies[i];
        if (e.alive && e.id == hit.mBodyID)
        {
            auto point = ray.GetPointOnRay(hit.mFraction);
            return RayHit{{i, e.generation},
                          {point.GetX(), point.GetY(), point.GetZ()},
                          V(impl_->physics.GetBodyInterface().GetTransformedShape(e.id).GetWorldSpaceSurfaceNormal(
                              hit.mSubShapeID2, point)),
                          hit.mFraction};
        }
    }
    return {};
}
RigidContacts SimulationWorld::ContactCounts(BodyHandle h) const
{
    auto id = impl_->Get(h).id;
    for (size_t i = 0; i < impl_->contacts.count; ++i)
        if (impl_->contacts.records[i].id == id)
            return impl_->contacts.records[i].counts;
    return {};
}
FoamOverlaps SimulationWorld::QueryFoamCapsule(Pose pose, double halfLength, double radius) const
{
    CheckPose(pose);
    if (!std::isfinite(halfLength) || halfLength <= 0 || !std::isfinite(radius) || radius <= 0)
        throw std::invalid_argument("Invalid foam capsule");
    JPH::CapsuleShape capsule{float(halfLength), float(radius)};
    struct StaticOnly : JPH::ObjectLayerFilter
    {
        bool ShouldCollide(JPH::ObjectLayer l) const override { return l == 0; }
    } filter;
    struct Collector : JPH::CollideShapeCollector
    {
        FoamOverlaps result;
        const JPH::BodyInterface *bodies{};
        void AddHit(const JPH::CollideShapeResult &hit) override
        {
            if (hit.mPenetrationDepth <= 0)
                return;
            const auto n = -hit.mPenetrationAxis.Normalized();
            FoamOverlap p{V(hit.mContactPointOn2), V(n), hit.mPenetrationDepth,
                          bodies->GetFriction(hit.mBodyID2), hit.mBodyID2.GetIndexAndSequenceNumber()};
            for (size_t i = 0; i < result.count; ++i)
            {
                const auto &old = result.patches[i];
                double dx = p.point.x - old.point.x, dy = p.point.y - old.point.y,
                       dz = p.point.z - old.point.z;
                double dot =
                    p.normal.x * old.normal.x + p.normal.y * old.normal.y + p.normal.z * old.normal.z;
                if (dx * dx + dy * dy + dz * dz < .0001 && dot > .98)
                {
                    if (p.depth > old.depth)
                        result.patches[i] = p;
                    return;
                }
            }
            if (result.count < 4)
                result.patches[result.count++] = p;
            else
            {
                auto weakest = std::min_element(result.patches.begin(), result.patches.end(),
                                                [](auto &a, auto &b) { return a.depth < b.depth; });
                if (p.depth > weakest->depth)
                    *weakest = p;
            }
        }
    } collector;
    collector.bodies = &impl_->physics.GetBodyInterface();
    impl_->physics.GetNarrowPhaseQuery().CollideShape(
        &capsule, JPH::Vec3::sReplicate(1),
        JPH::RMat44::sRotationTranslation(Q(pose.rotation), JPH::RVec3(V(pose.position))),
        JPH::CollideShapeSettings{}, JPH::RVec3::sZero(), collector, {}, filter);
    return collector.result;
}
std::optional<WheelContact> SimulationWorld::QueryWheelContact(Pose pose, double radius, double width) const
{
    CheckPose(pose);
    if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(width) || width <= 0)
        throw std::invalid_argument("Invalid wheel contact dimensions");
    JPH::CylinderShape wheel{float(width / 2), float(radius),
                             float(std::min({.001, width / 20, radius / 20}))};
    struct StaticOnly : JPH::ObjectLayerFilter
    {
        bool ShouldCollide(JPH::ObjectLayer l) const override { return l == 0; }
    } filter;
    struct Collector : JPH::CollideShapeCollector
    {
        JPH::Vec3 up;
        std::optional<WheelContact> result;
        void AddHit(const JPH::CollideShapeResult &hit) override
        {
            const auto normal = -hit.mPenetrationAxis.Normalized();
            if (normal.Dot(up) <= .25f)
                return;
            const double gap = -hit.mPenetrationDepth;
            if (!result || gap < result->gap)
                result = WheelContact{V(hit.mContactPointOn2), V(normal), gap};
        }
    } collector;
    collector.up = Q(pose.rotation) * JPH::Vec3::sAxisZ();
    JPH::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = .01f;
    impl_->physics.GetNarrowPhaseQuery().CollideShape(
        &wheel, JPH::Vec3::sReplicate(1),
        JPH::RMat44::sRotationTranslation(Q(pose.rotation), JPH::RVec3(V(pose.position))), settings,
        JPH::RVec3::sZero(), collector, {}, filter);
    return collector.result;
}
void SimulationWorld::Step(double dt)
{
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.02)
        throw std::invalid_argument("Invalid physics step");
    impl_->contacts.manifolds = 0;
    impl_->contacts.count = 0;
    if (impl_->physics.Update(float(dt), 1, &impl_->allocator, &impl_->jobs) != JPH::EPhysicsUpdateError::None)
        throw std::runtime_error("Jolt physics capacity exceeded");
}
void SimulationWorld::Reset()
{
    for (uint32_t i = 0; i < impl_->bodies.size(); ++i)
    {
        auto &e = impl_->bodies[i];
        if (e.alive)
        {
            SetBodyState({i, e.generation}, {e.spec.pose, {}, {}, true});
            if (e.spec.mass > 0)
            {
                JPH::BodyLockWrite lock(impl_->physics.GetBodyLockInterface(), e.id);
                lock.GetBody().ResetForce();
                lock.GetBody().ResetTorque();
            }
        }
    }
}
struct SimulationClock::Registry
{
    uint64_t next{};
    std::map<uint64_t, PlantCallbacks> callbacks;
};
Registration::Registration(std::function<void()> r) : release_(std::move(r))
{
}
Registration::~Registration()
{
    if (release_)
        release_();
}
Registration::Registration(Registration &&r) noexcept : release_(std::exchange(r.release_, {}))
{
}
Registration &Registration::operator=(Registration &&r) noexcept
{
    if (this != &r)
    {
        if (release_)
            release_();
        release_ = std::exchange(r.release_, {});
    }
    return *this;
}
SimulationClock::SimulationClock(Backend b) : registry_(std::make_shared<Registry>())
{
    if (b != Backend::Phoenix)
        world_ = std::make_unique<SimulationWorld>(b);
}
SimulationClock::~SimulationClock() = default;
Registration SimulationClock::Register(PlantCallbacks c)
{
    auto id = registry_->next++;
    registry_->callbacks.emplace(id, std::move(c));
    return Registration([weak = std::weak_ptr<Registry>(registry_), id] {
        if (auto r = weak.lock())
            r->callbacks.erase(id);
    });
}
void SimulationClock::SetBusVoltage(double voltage)
{
    if (!std::isfinite(voltage) || voltage < 0 || voltage > 16)
        throw std::invalid_argument("Simulation bus voltage must be in [0,16]");
    busVoltage_ = voltage;
}
void SimulationClock::StepRobotPeriod()
{
    using Clock = std::chrono::steady_clock;
    for (int i = 0; i < 5; ++i)
    {
        auto start = Clock::now();
        std::vector<uint64_t> ids;
        for (auto &[id, c] : registry_->callbacks)
            ids.push_back(id);
        for (auto id : ids)
            if (auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
            {
                auto f = it->second.beforePhysics;
                if (f)
                    f(.004);
            }
        if (world_)
            world_->Step(.004);
        time_ += .004;
        for (auto id : ids)
            if (auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
            {
                auto f = it->second.afterPhysics;
                if (f)
                    f(.004);
            }
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        timing_.seconds = t;
        timing_.maximumSeconds = std::max(timing_.maximumSeconds, t);
        ++timing_.substeps;
        if (t > .004)
            ++timing_.deadlineMisses;
    }
    supplyCurrent_ = 0;
    std::vector<uint64_t> powerIds;
    for (auto &[id, c] : registry_->callbacks)
        powerIds.push_back(id);
    for (auto id : powerIds)
    {
        if (auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
        {
            auto callback = it->second.supplyCurrent;
            if (callback)
            {
                const double current = callback();
                if (!std::isfinite(current) || current < 0)
                    throw std::runtime_error("Invalid simulated supply current");
                if (!std::isfinite(supplyCurrent_ + current))
                    throw std::runtime_error("Simulated aggregate supply current overflow");
                supplyCurrent_ += current;
            }
        }
    }
}
void SimulationClock::Reset()
{
    if (world_)
        world_->Reset();
    time_ = 0;
    supplyCurrent_ = 0;
    busVoltage_ = 12;
    timing_ = {};
    std::vector<uint64_t> ids;
    for (auto &[id, c] : registry_->callbacks)
        ids.push_back(id);
    for (auto id : ids)
    {
        if (auto it = registry_->callbacks.find(id); it != registry_->callbacks.end())
        {
            auto callback = it->second.reset;
            if (callback)
                callback();
        }
    }
}
SimulationClock &GetSimulationRuntime()
{
    static SimulationClock clock(BackendFromEnvironment());
    return clock;
}
} // namespace base::sim
