#pragma once

#include "constants.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>

namespace reanaut
{

static constexpr size_t kMaxLaserScans = 1000;

struct LaserScan
{
    static constexpr float kMinLaserDsitance = 20.F;

    using Real = RealType;

    uint32_t quality;   //!< ?
    float    angle;     //!< 0-360 [deg]
    float    distance;  //!< 0=out-of-range [mm]
    uint32_t timestamp; //!< ?

    [[nodiscard]] auto toMeters() const -> Real;
    [[nodiscard]] auto toBeamAngle() const -> Real;
    [[nodiscard]] auto toWorldAngle(Real theta) const -> Real;
    [[nodiscard]] auto toWorldPoint(const Pose& pose) const -> Point2;
    [[nodiscard]] auto toWorldPointSafe(const Pose& pose) const -> std::optional<Point2>;
};

} // namespace reanaut
