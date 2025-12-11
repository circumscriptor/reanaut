#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "laser.hpp"

#include <cstdint>
#include <vector>

namespace reanaut
{

class OccupancyGrid : public Grid
{
public:

    using Real = RealType;

    OccupancyGrid();

    // Uses Bresenham's algorithm to clear free space and mark endpoints
    void updateFromScans(const Pose& robot, const std::vector<LaserScan>& scans);

    [[nodiscard]] static auto logOddsToColor(Real logOdds) -> uint32_t;

protected:

    // Bresenham's Line Algorithm implementation
    void traceLine(Index index0, Index index1);

private:

    // Precomputed log-odds increments
    const Real m_loOccInc;
    const Real m_loFreeInc;
};

} // namespace reanaut
