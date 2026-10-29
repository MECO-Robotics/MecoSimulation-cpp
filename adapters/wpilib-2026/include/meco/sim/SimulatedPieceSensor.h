#pragma once
#include <base/PieceDetection.h>
#include <base/ObjectPerception.h>
#include <base/sim/SimulationWorld.h>
#include <frc/geometry/Transform3d.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <span>
#include <stdexcept>

namespace meco::sim {
struct PieceSensorConfig {
    frc::Transform3d robotToCamera;
    double horizontalFov = 120 * std::numbers::pi / 180;
    double verticalFov = 90 * std::numbers::pi / 180;
    double maxRange = 8., dropout = .02, angularNoise = .002;
    unsigned seed = 3737;
};
// Synthetic camera only. Produces the same typed ray consumed by the real
// PhotonPieceDetection IO path; it never supplies a hidden field-space goal.
class SimulatedPieceSensor {
 public:
    explicit SimulatedPieceSensor(PieceSensorConfig config) : config_(config), random_(config.seed) {
        if (!std::isfinite(config.maxRange) || config.maxRange <= 0 ||
            !(config.horizontalFov > 0 && config.horizontalFov <= 2*std::numbers::pi) ||
            !(config.verticalFov > 0 && config.verticalFov <= std::numbers::pi) ||
            !(config.dropout >= 0 && config.dropout <= 1) ||
            !std::isfinite(config.angularNoise) || config.angularNoise < 0)
            throw std::invalid_argument("Invalid simulated piece sensor configuration");
    }
    void SetMount(const frc::Transform3d& mount) { config_.robotToCamera=mount; }
    base::ObjectDetectionFrame ObserveFrame(const frc::Pose3d& robot,
            std::span<const frc::Translation3d> pieces,
            std::span<const base::sim::BodySpec> obstacles, units::second_t timestamp, uint64_t sequence) {
        const auto camera=robot.TransformBy(config_.robotToCamera);
        std::vector<frc::Translation3d> sorted(pieces.begin(),pieces.end());
        std::stable_sort(sorted.begin(),sorted.end(),[&](const auto& a,const auto& b) {
            return a.Distance(camera.Translation())<b.Distance(camera.Translation());
        });
        base::ObjectDetectionFrame frame{sequence,timestamp,{}};
        for(const auto& point:sorted) {
            const std::array<frc::Translation3d,1> one{point};
            if(const auto ray=Observe(robot,one,obstacles,timestamp)) {
                const double range=point.Distance(camera.Translation()).value();
                const double yaw=-ray->yawDegrees*std::numbers::pi/180, pitch=ray->pitchDegrees*std::numbers::pi/180;
                frame.detections.push_back({{units::meter_t{range*std::cos(pitch)*std::cos(yaw)},
                    units::meter_t{range*std::cos(pitch)*std::sin(yaw)},units::meter_t{range*std::sin(pitch)}},1.});
                if(frame.detections.size()==128) break;
            }
        }
        return frame;
    }
    std::optional<base::PieceRay> Observe(const frc::Pose3d& robot,
            std::span<const frc::Translation3d> pieces,
            std::span<const base::sim::BodySpec> obstacles, units::second_t timestamp) {
        const auto camera = robot.TransformBy(config_.robotToCamera);
        std::optional<frc::Translation3d> nearest;
        double nearestRange = config_.maxRange;
        for (const auto& point : pieces) {
            const frc::Pose3d target{point, frc::Rotation3d{}};
            const auto relative = frc::Transform3d{camera, target}.Translation();
            const double x = relative.X().value(), y = relative.Y().value(), z = relative.Z().value();
            const double range = relative.Norm().value();
            if (x <= 0 || range > nearestRange ||
                std::abs(std::atan2(y,x)) > config_.horizontalFov/2 ||
                std::abs(std::atan2(z,std::hypot(x,y))) > config_.verticalFov/2) continue;
            bool hidden = false;
            for (const auto& box : obstacles) {
                const auto q = box.pose.rotation;
                const frc::Pose3d pose{units::meter_t{box.pose.position.x}, units::meter_t{box.pose.position.y},
                    units::meter_t{box.pose.position.z}, frc::Rotation3d{frc::Quaternion{q.w,q.x,q.y,q.z}}};
                const auto a = frc::Transform3d{pose,camera}.Translation();
                const auto b = frc::Transform3d{pose,target}.Translation();
                const std::array<double,3> origin{a.X().value(),a.Y().value(),a.Z().value()};
                const std::array<double,3> end{b.X().value(),b.Y().value(),b.Z().value()};
                const std::array<double,3> half{box.halfExtents.x+.03,box.halfExtents.y+.03,box.halfExtents.z+.03};
                double near=.02, far=.98;
                for (int axis=0;axis<3;++axis) {
                    const double d=end[axis]-origin[axis];
                    if (std::abs(d)<1e-8) { if (std::abs(origin[axis])>half[axis]) far=-1; }
                    else {
                        const double first=(-half[axis]-origin[axis])/d, second=(half[axis]-origin[axis])/d;
                        near=std::max(near,std::min(first,second)); far=std::min(far,std::max(first,second));
                    }
                }
                if (far>=near) { hidden=true; break; }
            }
            if (!hidden) { nearest = relative; nearestRange = range; }
        }
        if (!nearest || std::uniform_real_distribution<double>{0,1}(random_) < config_.dropout) return std::nullopt;
        auto noise = [this] { return config_.angularNoise == 0 ? 0. : std::normal_distribution<double>{0,config_.angularNoise}(random_); };
        return base::PieceRay{timestamp,
            (-std::atan2(nearest->Y().value(),nearest->X().value())+noise())*180/std::numbers::pi,
            (std::atan2(nearest->Z().value(),std::hypot(nearest->X().value(),nearest->Y().value()))+noise())*180/std::numbers::pi};
    }
 private:
    PieceSensorConfig config_;
    std::mt19937 random_;
};
}
