#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include <stdexcept>
#include <frc/geometry/Pose3d.h>

namespace meco::sim {
// SI positions and velocities. The body pose and center of mass are in the
// field and body frames respectively; body velocities are field-relative, with
// linear velocity measured at the center of mass.
struct ParticlePoint { double x{}, y{}, z{}; };
struct Particle { ParticlePoint position, velocity; };
struct ContactBody { frc::Pose3d pose; ParticlePoint velocity, angularVelocity, centerOfMass; };
struct ContactBox { ParticlePoint center, halfExtents; };
namespace detail {
inline frc::Translation3d Translation(ParticlePoint p) { return {units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z}}; }
inline ParticlePoint Point3d(const frc::Translation3d& p) { return {p.X().value(), p.Y().value(), p.Z().value()}; }
inline ParticlePoint World(ParticlePoint p, const frc::Pose3d& body) { return Point3d(body.Translation()+Translation(p).RotateBy(body.Rotation())); }
} // namespace detail
// Equal-mass spherical particles contact prescribed oriented boxes. Corrections
// do not exert reaction forces on the body; boxes are in its local frame.
// Radii and box extents must be positive and restitution must be in [0, 1].
// This is a discrete single-pass approximation: fast particles can tunnel, and
// resolving one contact can introduce another overlap until a subsequent call.
// Floor and airborne groups have identical contact behavior and must be distinct.
namespace detail {
inline void BoxContacts(Particle& ball, const ContactBody& robot, double radius, double restitution, const std::vector<ContactBox>& boxes) {
 auto local=detail::Point3d((detail::Translation(ball.position)-robot.pose.Translation()).RotateBy(-robot.pose.Rotation()));
 for(const auto& box : boxes){
  const auto& center=box.center;
  const auto& extent=box.halfExtents;
  ParticlePoint delta{local.x-center.x,local.y-center.y,local.z-center.z};
  ParticlePoint nearest{std::clamp(delta.x,-extent.x,extent.x),std::clamp(delta.y,-extent.y,extent.y),std::clamp(delta.z,-extent.z,extent.z)};
  ParticlePoint normal{delta.x-nearest.x,delta.y-nearest.y,delta.z-nearest.z};
  double distance=std::sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
  if(distance>=radius)continue;
  if(distance<1e-12){
   const double dx=extent.x-std::abs(delta.x),dy=extent.y-std::abs(delta.y),dz=extent.z-std::abs(delta.z);
   normal={};
   if(dx<=dy&&dx<=dz){normal.x=delta.x>=0?1:-1;distance=-dx;}
   else if(dy<=dz){normal.y=delta.y>=0?1:-1;distance=-dy;}
   else{normal.z=delta.z>=0?1:-1;distance=-dz;}
  }else{normal.x/=distance;normal.y/=distance;normal.z/=distance;}
  local.x+=normal.x*(radius-distance);local.y+=normal.y*(radius-distance);local.z+=normal.z*(radius-distance);
  ball.position=detail::World(local,robot.pose);
  const auto n=detail::Point3d(detail::Translation(normal).RotateBy(robot.pose.Rotation()));
  const auto com=detail::World(robot.centerOfMass,robot.pose);
  const ParticlePoint r{ball.position.x-radius*n.x-com.x,ball.position.y-radius*n.y-com.y,ball.position.z-radius*n.z-com.z};
  const auto w=robot.angularVelocity;
  const ParticlePoint surface{robot.velocity.x+w.y*r.z-w.z*r.y,robot.velocity.y+w.z*r.x-w.x*r.z,robot.velocity.z+w.x*r.y-w.y*r.x};
  const double speed=(ball.velocity.x-surface.x)*n.x+(ball.velocity.y-surface.y)*n.y+(ball.velocity.z-surface.z)*n.z;
  if(speed<0){ball.velocity.x-=(1+restitution)*speed*n.x;ball.velocity.y-=(1+restitution)*speed*n.y;ball.velocity.z-=(1+restitution)*speed*n.z;}
 }
}
} // namespace detail
inline void ResolveParticleContacts(std::vector<Particle>& floor, std::vector<Particle>& airborne, const ContactBody& robot, double radius, double boxRestitution, double particleRestitution, const std::vector<ContactBox>& boxes){
 if (&floor == &airborne)
  throw std::invalid_argument("Particle groups must be distinct");
 if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(boxRestitution) ||
     boxRestitution < 0 || boxRestitution > 1 || !std::isfinite(particleRestitution) ||
     particleRestitution < 0 || particleRestitution > 1)
  throw std::invalid_argument("Invalid particle radius or restitution");
 for (const auto& box : boxes) {
  const auto& c=box.center; const auto& e=box.halfExtents;
  if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) ||
      !std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z) ||
      e.x <= 0 || e.y <= 0 || e.z <= 0)
   throw std::invalid_argument("Invalid contact box");
 }
 const auto finite = [](ParticlePoint p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
 const auto& pose = robot.pose;
 const auto q = pose.Rotation().GetQuaternion();
 if (!std::isfinite(pose.X().value()) || !std::isfinite(pose.Y().value()) || !std::isfinite(pose.Z().value()) ||
     !std::isfinite(q.W()) || !std::isfinite(q.X()) || !std::isfinite(q.Y()) || !std::isfinite(q.Z()) ||
     !finite(robot.velocity) || !finite(robot.angularVelocity) || !finite(robot.centerOfMass))
  throw std::invalid_argument("Invalid contact body state");
 for (const auto* group : {&floor, &airborne})
  for (const auto& p : *group)
   if (!finite(p.position) || !finite(p.velocity))
    throw std::invalid_argument("Invalid particle state");
 std::vector<Particle*> balls;balls.reserve(floor.size()+airborne.size());
 for(auto& p:floor)balls.push_back(&p);
 for(auto& p:airborne)balls.push_back(&p);
 for(auto* p:balls)detail::BoxContacts(*p,robot,radius,boxRestitution,boxes);
 const double diameter=2*radius;
 // Sweep broad phase avoids all-pairs work for the large particle sets.
 std::sort(balls.begin(),balls.end(),[](auto* a,auto* b){return a->position.x<b->position.x;});
 std::vector<double> sweepX; sweepX.reserve(balls.size());
 for(auto* p:balls)sweepX.push_back(p->position.x);
 for(std::size_t i=0;i<balls.size();++i)for(std::size_t j=i+1;j<balls.size();++j){
  auto& a=*balls[i];auto& b=*balls[j];
  if(sweepX[j]-sweepX[i]>diameter)break;
  ParticlePoint n{b.position.x-a.position.x,b.position.y-a.position.y,b.position.z-a.position.z};
  double d=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);if(d>=diameter)continue;
  if(d<1e-12){n={1,0,0};d=0;}else{n.x/=d;n.y/=d;n.z/=d;}
  const double correction=(diameter-d)/2;
  a.position.x-=n.x*correction;a.position.y-=n.y*correction;a.position.z-=n.z*correction;
  b.position.x+=n.x*correction;b.position.y+=n.y*correction;b.position.z+=n.z*correction;
  const double closing=(b.velocity.x-a.velocity.x)*n.x+(b.velocity.y-a.velocity.y)*n.y+(b.velocity.z-a.velocity.z)*n.z;
  if(closing<0){const double impulse=-(1+particleRestitution)*.5*closing;
   a.velocity.x-=impulse*n.x;a.velocity.y-=impulse*n.y;a.velocity.z-=impulse*n.z;
   b.velocity.x+=impulse*n.x;b.velocity.y+=impulse*n.y;b.velocity.z+=impulse*n.z;
  }
 }
}
} // namespace meco::sim
