#pragma once

#include "cloud.hpp"
#include "constants.hpp"
#include "grid.hpp"
#include "laser.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace reanaut
{

struct VirtualReading
{
    using Real = RealType;

    Real angle;    // Radians [-PI, PI]
    Real distance; // Meters
    Real worldX;   // Meters
    Real worldY;   // Meters
};

class OccupancyGrid : public Grid
{
public:

    using Real = RealType;

    OccupancyGrid();

    [[nodiscard]] auto isOccupied(Point2 world) const -> bool;
    [[nodiscard]] auto getProbability(Point2 world) const -> std::optional<Real>;
    [[nodiscard]] auto getProbabilitySmooth(Point2 world) const -> std::optional<Real>;

    // Uses Bresenham's algorithm to clear free space and mark endpoints
    void updateFromScans(const Pose& robot, const std::vector<LaserScan>& scans);
    void updateFromCloud(const Pose& robot, const PointCloud& cloud);
    void getVirtualScan(std::vector<VirtualReading>& readings, const Pose& pose, Real maxDistance, int numRays = 360) const; // NOLINT

    [[nodiscard]]
    static auto logOddsToColor(Real logOdds) -> uint32_t;

protected:

    [[nodiscard]]
    auto castRay(const Pose& pose, Real maxDistance) const -> Real;

    // Bresenham's Line Algorithm implementation
    void traceLine(Index index0, Index index1);

private:

    // Precomputed log-odds increments
    const Real m_loOccInc;
    const Real m_loFreeInc;
};

} // namespace reanaut
