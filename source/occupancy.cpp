#include "constants.hpp"
#include "occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

namespace reanaut
{

OccupancyGrid::OccupancyGrid()
    : m_width(kMapWidth), m_height(kMapHeight), m_resolution(kMapResolution), //
      m_originX(kMapOriginX), m_originY(kMapOriginY),                         //
      m_loOccInc(std::log(kLogOddsOccupied / (1.0 - kLogOddsOccupied))),      //
      m_loFreeInc(std::log(kLogOddsFree / (1.0 - kLogOddsFree)))
{
    m_grid.resize(size_t(m_width) * m_height, 0.0); // Initialize to 0 (Unknown probability 0.5)
}

auto OccupancyGrid::worldToGrid(Point2 world, Index& index) const -> bool
{
    if (world.x < m_originX || world.y < m_originY) {
        return false;
    }
    index.x = static_cast<int>((world.x - m_originX) / m_resolution);
    index.y = static_cast<int>((world.y - m_originY) / m_resolution);
    return (index.x >= 0 && index.x < m_width && index.y >= 0 && index.y < m_height);
}

void OccupancyGrid::gridToWorld(Index index, Point2& world) const
{
    world.x = m_originX + (index.x + 0.5) * m_resolution;
    world.y = m_originY + (index.y + 0.5) * m_resolution;
}

auto OccupancyGrid::getDistance(const Pose& pose) const -> Real
{
    const Real stepSize = m_resolution;
    const Real maxDist  = kLidarMaxRange;

    Real currDist = 0.0;

    const Real dx = std::cos(pose.theta);
    const Real dy = std::sin(pose.theta);

    // Simple stepping raycast (faster than Bresenham for readout)
    while (currDist < maxDist) {
        const Point2 check{
            .x = pose.x + (dx * currDist),
            .y = pose.y + (dy * currDist),
        };

        Index index{};
        if (not worldToGrid(check, index)) {
            return maxDist; // Out of bounds is "open space" or max range
        }

        // If log odds > 0, it's likely occupied
        if (m_grid[size_t(index.y * m_width) + index.x] > 2.0) { // Threshold 2.0 for "confident obstacle"
            return currDist;
        }
        currDist += stepSize;
    }
    return maxDist;
}

void OccupancyGrid::updateFromScans(const Pose& robot, const std::vector<Real>& scans)
{
    Index index{};
    if (not worldToGrid(robot, index)) {
        return; // Robot off map
    }

    const Real angleStep = (2.0 * std::numbers::pi) / Real(scans.size());

    for (size_t i = 0; i < scans.size(); ++i) {
        const Real range = scans[i];
        // Filter invalid ranges
        if (range < kLidarMinRange || range >= kLidarMaxRange) {
            continue;
        }

        const Real beamAngle  = -std::numbers::pi + (Real(i) * angleStep);
        const Real worldAngle = std::fmod(robot.theta + beamAngle, 2.0 * std::numbers::pi);

        const Point2 end{
            .x = robot.x + (range * std::cos(worldAngle)),
            .y = robot.y + (range * std::sin(worldAngle)),
        };

        Index endIndex;
        if (worldToGrid(end, endIndex)) {
            traceLine(index, endIndex);
        }
    }
}

void OccupancyGrid::traceLine(Index index0, Index index1)
{
    int dx  = std::abs(index1.x - index0.x);
    int sx  = index0.x < index1.x ? 1 : -1;
    int dy  = -std::abs(index1.y - index0.y);
    int sy  = index0.y < index1.y ? 1 : -1;
    int err = dx + dy;
    int e2{};

    while (true) {
        // If we are at the end point (obstacle)
        if (index0.x == index1.x && index0.y == index1.y) {
            updateCell(index0, m_loOccInc); // Mark Occupied
            break;
        }

        // We are strictly between start and end (Free Space)
        updateCell(index0, m_loFreeInc);

        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            index0.x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            index0.y += sy;
        }
    }
}

void OccupancyGrid::updateCell(Index index, Real change)
{
    int idx = (index.y * m_width) + index.x;
    if (idx >= 0 && idx < int(m_grid.size())) {
        m_grid[idx] += change;
        // Clamp values to prevent overconfidence/underflow
        m_grid[idx] = std::clamp(m_grid[idx], kLogOddsMin, kLogOddsMax);
    }
}

auto OccupancyGrid::logOddsToColor(Real logOdds) -> uint32_t
{
    const Real clamped    = std::clamp(logOdds, kLogOddsMin, kLogOddsMax);
    const Real normalized = (clamped - kLogOddsMin) / (kLogOddsMax - kLogOddsMin);
    const auto value      = static_cast<uint8_t>(255.0 * (1.0 - normalized));
    return 0xFF000000 | (value << 16) | (value << 8) | value; // NOLINT
}

} // namespace reanaut
