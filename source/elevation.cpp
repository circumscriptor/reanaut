#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"

namespace reanaut
{

void ElevationGrid::update(const DepthProcessor& depth)
{
    for (const auto& [index, maxZ] : depth.observations()) {
        updateCellByIndex(Index::unpack(index), maxZ);
    }
}

// Renamed helper to update by index directly
void ElevationGrid::updateCellByIndex(Index index, Real measuredZ)
{
    auto& currentZ = at(index);

    constexpr Real kAlphaRise = Real(0.4);
    constexpr Real kAlphaFall = Real(0.1);

    if (measuredZ >= currentZ) {
        currentZ += kAlphaRise * (measuredZ - currentZ);
    } else {
        currentZ += kAlphaFall * (measuredZ - currentZ);
    }
}

// Wrapper for single point updates (e.g. from Lidar)
void ElevationGrid::updateCell(Real pointX, Real pointY, Real pointZ)
{
    if (const auto index = worldToGrid(Point2(pointX, pointY))) {
        updateCellByIndex(*index, pointZ);
    }
}

} // namespace reanaut
