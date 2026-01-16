#include "constants.hpp"
#include "laser.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
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

void LaserLookup::fromScans(std::span<const LaserScan> scans)
{
    // 1. Reset Lookup
    std::fill(m_lookup.begin(), m_lookup.end(), kInfinity);

    if (scans.empty()) {
        return;
    }

    const size_t numScans = scans.size();
    size_t       nextIdx  = 0;

    // 2. Single Pass Interpolation (O(360 + N))
    for (int i = 0; i < 360; ++i) { // NOLINT

        auto deg = static_cast<Real>(i);

        // Advance next_idx until scans[next_idx] > deg
        // This finds the first scan point that is *after* our current degree.
        while (nextIdx < numScans && static_cast<Real>(scans[nextIdx].angle) <= deg) {
            nextIdx++;
        }

        // Identify the two bounding points: Prev and Next
        Real angPrev;
        Real distPrev;
        Real angNext;
        Real distNext;

        if (nextIdx == 0) {
            // Wrap-around case: We are before the first scan.
            // Interval is [LastScan - 360, FirstScan]
            const auto& sNext = scans[0];
            const auto& sPrev = scans[numScans - 1];

            angNext  = static_cast<Real>(sNext.angle);
            distNext = static_cast<Real>(sNext.toMeters());

            angPrev  = static_cast<Real>(sPrev.angle) - 360.0;
            distPrev = static_cast<Real>(sPrev.toMeters());

        } else if (nextIdx == numScans) {
            // Wrap-around case: We are after the last scan.
            // Interval is [LastScan, FirstScan + 360]
            const auto& sNext = scans[0];
            const auto& sPrev = scans[numScans - 1];

            angNext  = static_cast<Real>(sNext.angle) + 360.0;
            distNext = static_cast<Real>(sNext.toMeters());

            angPrev  = static_cast<Real>(sPrev.angle);
            distPrev = static_cast<Real>(sPrev.toMeters());

        } else {
            // Standard case: Interval is [scans[i-1], scans[i]]
            const auto& sNext = scans[nextIdx];
            const auto& sPrev = scans[nextIdx - 1];

            angNext  = static_cast<Real>(sNext.angle);
            distNext = static_cast<Real>(sNext.toMeters());

            angPrev  = static_cast<Real>(sPrev.angle);
            distPrev = static_cast<Real>(sPrev.toMeters());
        }

        // 3. Interpolate
        const Real gap = angNext - angPrev;

        // Only fill if the gap is small enough (avoid closing open doorways)
        if (gap < kMaxInterpGap && gap > 0.001) { // NOLINT
            const Real lt = (deg - angPrev) / gap;
            m_lookup[i]   = distPrev + lt * (distNext - distPrev);
        }
    }
}

auto LaserLookup::get(int degree) const -> Real
{
    // Handle negative or >360 degrees safely
    int idx = degree % 360; // NOLINT
    if (idx < 0) {
        idx += 360; // NOLINT
    }
    return m_lookup[idx];
}

} // namespace reanaut
