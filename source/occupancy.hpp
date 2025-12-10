#pragma once

#include "constants.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace reanaut
{

class OccupancyGrid
{
public:

    using Real = RealType;

    struct Index
    {
        int x{};
        int y{};
    };

    OccupancyGrid();

    [[nodiscard]] auto grid() -> std::span<Real> { return m_grid; }
    [[nodiscard]] auto grid() const -> std::span<const Real> { return m_grid; }

    // Convert World (meters) to Grid (index)
    auto worldToGrid(Point2 world, Index& index) const -> bool;

    // Convert Grid to World (for raycasting check)
    void gridToWorld(Index index, Point2& world) const;

    // Returns distance to nearest obstacle in the map from (x,y) at angle theta
    [[nodiscard]] auto getDistance(const Pose& pose) const -> Real;

    // Uses Bresenham's algorithm to clear free space and mark endpoints
    void updateFromScans(const Pose& robot, const std::vector<Real>& scans);

    [[nodiscard]] static auto logOddsToColor(Real logOdds) -> uint32_t;

protected:

    // Bresenham's Line Algorithm implementation
    void traceLine(Index index0, Index index1);

    void updateCell(Index index, Real change);

private:

    std::vector<Real> m_grid; // Stores log-odds values
    int               m_width;
    int               m_height;
    Real              m_resolution;
    Real              m_originX;
    Real              m_originY;

    // Precomputed log-odds increments
    const Real m_loOccInc;
    const Real m_loFreeInc;
};

} // namespace reanaut
