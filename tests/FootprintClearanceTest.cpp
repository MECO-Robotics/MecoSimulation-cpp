#include "base/sim/FootprintClearance.h"
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>
using namespace base::sim;
void Check(bool condition) { if (!condition) throw std::runtime_error("footprint clearance regression"); }
int main() {
    BodySpec box;
    box.halfExtents = {.4, .2, .1};
    box.pose.position = {2, 2, .1};
    std::vector<BodySpec> boxes{box};
    auto clear = [&](double x, double y, double angle) {
        return IsFootprintClear(x, y, angle, .4, .2, .45, 5, 5, .02, boxes);
    };
    Check(clear(1, 2, 0));
    Check(!clear(1.2, 2, 0)); // touching and margin count as blocked
    Check(!clear(.3, 2, 0)); // field edge
    Check(!clear(1.35, 2, 0));
    Check(clear(1.35, 2, std::numbers::pi / 2)); // rectangular footprint rotation
    boxes[0].pose.position.z = 1;
    Check(clear(2, 2, 0)); // obstacle above robot height
    boxes[0].pose.position.z = .5;
    boxes[0].pose.rotation = {0, std::sin(std::numbers::pi / 8), 0, std::cos(std::numbers::pi / 8)};
    Check(!clear(2, 2, 0)); // pitched box reaches robot height
    Check(!clear(std::numeric_limits<double>::quiet_NaN(), 2, 0));
    boxes[0] = box;
    Check(clear(.41, 4, 0)); // margin does not expand the field edge
    Check(!clear(.4, 4, 0)); // touching the edge is blocked
    boxes[0].shape = BodySpec::Shape::Sphere;
    Check(!clear(1, 1, 0));
    boxes[0] = box;
    boxes[0].pose.rotation = {.1, 0, 0, std::sqrt(.99)};
    Check(!clear(1, 1, 0)); // roll unsupported
    boxes[0].pose.rotation = {0, 0, .1, std::sqrt(.99)};
    Check(!clear(1, 1, 0)); // yaw unsupported
    boxes[0] = box;
    boxes[0].pose.position.z = std::numeric_limits<double>::quiet_NaN();
    Check(!clear(1, 1, 0));
    boxes[0] = box;
    boxes[0].halfExtents.x = -1;
    Check(!clear(1, 1, 0));
    boxes[0] = box;
    boxes[0].pose.rotation = {0, 0, 0, 0};
    Check(!clear(1, 1, 0));
}
