#include "constants.hpp"
#include "laser.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace reanaut
{

auto LaserScan::toMeters() const -> Real { return distance * kScanDistanceToWorld; }

auto LaserScan::toBeamAngle() const -> Real { return Real(angle) / 180.0 * std::numbers::pi; }

auto LaserScan::toWorldAngle(Real theta) const -> Real
{
    return std::fmod(theta - toBeamAngle() + std::numbers::pi, 2.0 * std::numbers::pi) - std::numbers::pi;
}

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

auto findClosestSampleFromAngle(const std::vector<LaserScan>& scans, RealType targetDeg) -> LaserScan
{
    if (scans.empty()) {
        return LaserScan{};
    }

    // NOLINTNEXTLINE(readability-magic-numbers)
    auto   minDiff = RealType(361); // Larger than any possible angle difference
    size_t index   = 0;

    for (size_t i = 0; i < scans.size(); ++i) {
        const auto& measurement = scans.at(i);
        // Convert measurement from radians to degrees for comparison

        RealType diff = std::abs(shortestAngleDiff(measurement.angle, targetDeg));

        if (diff < minDiff) {
            minDiff = diff;
            index   = i;
        }
    }
    return scans.at(index);
}

auto findShortestMeasurement(const std::vector<LaserScan>& scans) -> LaserScan
{
    if (scans.empty()) {
        return LaserScan{};
    }
    return *std::min_element(scans.data(), scans.data() + scans.size(), [](const auto& left, const auto& right) {
        // Protect against invalid 0.0 distance readings
        if (left.distance <= 0.0F) {
            return false;
        }
        if (right.distance <= 0.0F) {
            return true;
        }
        return left.distance < right.distance;
    });
}

auto findShortestMeasurementInRange(const std::vector<LaserScan>& scans, RealType startDeg, RealType endDeg) -> LaserScan
{
    RealType  minDist = std::numeric_limits<RealType>::max();
    ptrdiff_t index   = -1;

    for (size_t i = 0; i < scans.size(); ++i) {
        const auto& measurement = scans.at(i);
        if (measurement.distance <= 0.0F) {
            continue; // Ignore invalid distances
        }

        if (measurement.isBetween(startDeg, endDeg)) {
            if (measurement.distance < minDist) {
                minDist = measurement.distance;
                index   = ptrdiff_t(i);
            }
        }
    }

    if (index < 0) {
        return LaserScan{};
    }
    return scans.at(index);
}

} // namespace reanaut
