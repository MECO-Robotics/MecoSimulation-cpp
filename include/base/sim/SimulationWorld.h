#pragma once
#include <cstdint>
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace base::sim
{
// Estimated padded bumper edge radius, bounded by each segment's dimensions.
// This rounds the rigid contact envelope; it is not a foam compliance model.
inline constexpr double kBumperCornerRadius = .0381; // 1.5 inches

struct Vec3
{
    double x{}, y{}, z{};
};
struct Quaternion
{
    double x{}, y{}, z{}, w{1};
};
struct Pose
{
    Vec3 position{};
    Quaternion rotation{};
};
enum class Backend
{
    Phoenix,
    Planar,
    Full3D
};
Backend BackendFromEnvironment();
struct BodyHandle
{
    uint32_t index{};
    uint32_t generation{};
    bool operator==(const BodyHandle &) const = default;
};
struct BodySpec
{
    enum class Shape
    {
        Box,
        Sphere,
        BumperRing, // Four rectangular perimeter segments, hollow at the centre.
        Compound
    };
    struct BoxPart
    {
        Vec3 center, halfExtents;
        uint64_t tag{};
    };
    Shape shape{Shape::Box};
    Vec3 halfExtents{0.5, 0.5, 0.5};
    double radius{0.1}; // Sphere radius, or BumperRing wall thickness; metres.
    double mass{};  // zero creates a static collider
    Vec3 inertia{}; // zero selects shape-derived inertia
    Vec3 centerOfMass{};
    double friction{0.7}, restitution{};
    bool planar{};
    Pose pose{};
    std::vector<BoxPart> parts; // Compound only; tags 1=pan, 2=backing.
};
struct BodyState
{
    Pose pose{};
    // Pose is the body origin; linear velocity is at the center of mass.
    // Both velocity vectors are expressed in world coordinates.
    Vec3 linearVelocity{}, angularVelocity{};
    bool enabled{true};
};
struct RayHit
{
    BodyHandle body;
    Vec3 position, normal;
    double fraction;
};
struct FoamOverlap
{
    Vec3 point, normal;
    double depth{}, friction{};
    uint32_t surface{}; // Opaque terrain identity for compliant-contact history.
};
struct FoamOverlaps
{
    std::array<FoamOverlap, 4> patches{};
    std::size_t count{};
};
struct WheelContact
{
    Vec3 point, normal;
    double gap{};
};
struct RigidContacts
{
    uint32_t pan{}, backing{};
};
struct StepTiming
{
    double seconds{}, maximumSeconds{};
    uint64_t substeps{}, deadlineMisses{};
};
struct PlantCallbacks
{
    std::function<void(double)> beforePhysics;
    std::function<void(double)> afterPhysics;
    std::function<void()> reset;
    // Nonnegative battery-side amperes, sampled once after a robot period.
    std::function<double()> supplyCurrent{};
};
// A registration can outlive its clock. Destroy it before callback captures
// die.
class Registration
{
  public:
    Registration() = default;
    ~Registration();
    Registration(Registration &&) noexcept;
    Registration &operator=(Registration &&) noexcept;
    Registration(const Registration &) = delete;
    Registration &operator=(const Registration &) = delete;

  private:
    friend class SimulationClock;
    explicit Registration(std::function<void()> release);
    std::function<void()> release_;
};
// Single-threaded owner: create, query, and step on the robot thread.
// No Jolt types escape this boundary; all quantities use metres, kilograms, seconds.
class SimulationWorld
{
  public:
    explicit SimulationWorld(Backend mode);
    ~SimulationWorld();
    SimulationWorld(const SimulationWorld &) = delete;
    SimulationWorld &operator=(const SimulationWorld &) = delete;
    BodyHandle CreateBody(const BodySpec &);
    void DestroyBody(BodyHandle);
    BodyState ReadBody(BodyHandle) const;
    void SetBodyState(BodyHandle, const BodyState &);
    // Changes only linear velocity. Static or disabled bodies are left unchanged.
    void SetLinearVelocity(BodyHandle, Vec3);
    void ClearForces(BodyHandle);
    void SetEnabled(BodyHandle, bool);
    void AddForce(BodyHandle, Vec3 force, Vec3 worldPosition);
    void AddTorque(BodyHandle, Vec3 torque);
    std::optional<RayHit> Raycast(Vec3 origin, Vec3 displacement, std::optional<BodyHandle> exclude = {}) const;
    // Capsule local axis is Y. Static geometry only; no query-body allocation.
    FoamOverlaps QueryFoamCapsule(Pose, double halfLength, double radius) const;
    // Rigid finite-radius tire support, local axle Y; static terrain only.
    std::optional<WheelContact> QueryWheelContact(Pose, double radius, double width) const;
    RigidContacts ContactCounts(BodyHandle) const;
    void Step(double seconds);
    void Reset(); // Restore all extant bodies to their creation state.
    Backend Mode() const;
    uint32_t BodyCount() const;
    uint32_t ContactManifoldCount() const; // Added/persisted manifolds during latest step.

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// Registry and motor stepping remain available with Phoenix (no Jolt world).
class SimulationClock;
class SimulationClock
{
  public:
    explicit SimulationClock(Backend backend = Backend::Phoenix);
    ~SimulationClock();
    Registration Register(PlantCallbacks);
    void StepRobotPeriod(); // exactly five 4 ms steps, before -> world -> after
    void Reset();
    double Time() const
    {
        return time_;
    }
    double SupplyCurrent() const
    {
        return supplyCurrent_;
    }
    double BusVoltage() const
    {
        return busVoltage_;
    }
    void SetBusVoltage(double);
    const StepTiming &Timing() const
    {
        return timing_;
    }
    SimulationWorld *World()
    {
        return world_.get();
    }

  private:
    struct Registry;
    std::shared_ptr<Registry> registry_;
    std::unique_ptr<SimulationWorld> world_;
    double time_{};
    double supplyCurrent_{}, busVoltage_{12};
    StepTiming timing_{};
};
SimulationClock &GetSimulationRuntime();
} // namespace base::sim
