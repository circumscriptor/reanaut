#include "constants.hpp"
#include "laser.hpp"

#include <cmath>
#include <numbers>
#include <optional>

namespace reanaut
{

auto LaserScan::toMeters() const -> Real { return distance * kScanDistanceToWorld; }

auto LaserScan::toBeamAngle() const -> Real { return Real(angle) / 180.0 * std::numbers::pi; }

auto LaserScan::toWorldAngle(Real theta) const -> Real { return std::fmod(theta - toBeamAngle(), 2.0 * std::numbers::pi); }

auto LaserScan::toWorldPoint(const Pose& pose) const -> Point2
{
    const auto dis = toMeters();
    const auto rot = toWorldAngle(pose.theta);
    return {
        .x = pose.x + (dis * std::cos(rot)),
        .y = pose.y + (dis * std::sin(rot)),
    };
}

auto LaserScan::toWorldPointSafe(const Pose& pose) const -> std::optional<Point2>
{
    if (distance < kMinLaserDsitance) {
        return std::nullopt;
    }
    return toWorldPoint(pose);
}

} // namespace reanaut
