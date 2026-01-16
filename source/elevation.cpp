#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"
#include "occupancy.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>

// #include <algorithm>

namespace reanaut
{

void ElevationGrid::update(const DepthProcessor& depth)
{
    for (const auto& [index, maxZ] : depth.observations()) {
        updateCellByIndex(Index::unpack(index), maxZ);
    }
}

void ElevationGrid::clean(const OccupancyGrid& occupancy)
{
    if (width() != occupancy.width() || height() != occupancy.height()) {
        return;
    }

    // for (size_t i = 0; i < grid().size(); ++i) {
    //     // if (not occupancy.isOccupied(i)) {
    //     //     setAtOffset(i, 0.0);
    //     // }
    // }
}

void ElevationGrid::updateCellByIndex(Index index, Real measuredZ)
{
    // static constexpr Real kAlphaRise = Real(0.5);
    // static constexpr Real kAlphaFall = Real(0.4);

    // if (auto& currentZ = at(index); measuredZ >= currentZ) {
    //     currentZ += kAlphaRise * (measuredZ - currentZ);
    // } else {
    //     currentZ += kAlphaFall * (measuredZ - currentZ);
    // }

    auto& currentZ = at(index);
    // currentZ       = std::max(currentZ, measuredZ);
    currentZ = std::clamp(measuredZ, 0.0, 1.0);
    // currentZ = (currentZ * 0.5) + (measuredZ * 0.5);
}

void ElevationGrid::updateCell(Real pointX, Real pointY, Real pointZ)
{
    if (const auto index = worldToGrid(Point2(pointX, pointY))) {
        updateCellByIndex(*index, pointZ);
    }
}

// NOLINTBEGIN
auto getElevationColor(RealType value) -> uint32_t
{
    // 0.5 meters = max
    const auto t = std::clamp(value * 2.0, RealType(0), RealType(1));
    const auto r = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(3)), RealType(0), RealType(1));
    const auto g = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(2)), RealType(0), RealType(1));
    const auto b = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(1)), RealType(0), RealType(1));
    return 0xFF000000 | (static_cast<uint32_t>(b * 255) << 16) | (static_cast<uint32_t>(g * 255) << 8) | static_cast<uint32_t>(r * 255);
}
// NOLINTEND

} // namespace reanaut
