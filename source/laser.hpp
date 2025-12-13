#pragma once

#include "constants.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

namespace reanaut
{

static constexpr size_t kMaxLaserScans = 1000;

[[nodiscard]]
static constexpr auto normalizeAngle360(RealType deg) -> RealType
{
    return std::fmod(std::fmod(deg, RealType(360)) + RealType(360), RealType(360)); // NOLINT(readability-magic-numbers)
}

struct LaserScan
{
    static constexpr float kMinLaserDsitance = 20.F;

    using Real = RealType;

    uint32_t quality{};   //!< ?
    float    angle{};     //!< 0-360 [deg]
    float    distance{};  //!< 0=out-of-range [mm]
    uint32_t timestamp{}; //!< ?

    [[nodiscard]] auto toMeters() const -> Real;
    [[nodiscard]] auto toBeamAngle() const -> Real;
    [[nodiscard]] auto toWorldAngle(Real theta) const -> Real;
    [[nodiscard]] auto toWorldPoint(const Pose& pose) const -> Point2;
    [[nodiscard]] auto toWorldPointSafe(const Pose& pose) const -> std::optional<Point2>;

    [[nodiscard]] auto isBetween(RealType minDeg, RealType maxDeg) const
    {
        auto nAngle = normalizeAngle360(angle);
        auto nMin   = normalizeAngle360(minDeg);
        auto nMax   = normalizeAngle360(maxDeg);

        // Normal range (e.g., 10 to 90)
        if (nMin < nMax) {
            return (nAngle >= nMin && nAngle <= nMax);
        }
        // Zero crossing range (e.g., 350 to 10)
        if (nMin > nMax) {
            return (nAngle >= nMin || nAngle <= nMax);
        }
        // Min == Max
        return nAngle == nMin;
    }
};

[[nodiscard]] static constexpr auto shortestAngleDiff(RealType from, RealType to) -> RealType
{
    auto diff = normalizeAngle360(to) - normalizeAngle360(from);
    if (diff > 180.0F) {
        diff -= 360.0F;
    } else if (diff < -180.0F) {
        diff += 360.0F;
    }
    return diff;
}

[[nodiscard]] auto findClosestSampleFromAngle(const std::vector<LaserScan>& scans, RealType targetDeg) -> LaserScan;

[[nodiscard]] auto findShortestMeasurement(const std::vector<LaserScan>& scans) -> LaserScan;

[[nodiscard]] auto findShortestMeasurementInRange(const std::vector<LaserScan>& scans, RealType startDeg, RealType endDeg) -> LaserScan;

} // namespace reanaut
