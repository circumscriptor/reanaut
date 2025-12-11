#include "constants.hpp"
#include "laser.hpp"
#include "occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace reanaut
{

OccupancyGrid::OccupancyGrid()
    : m_loOccInc(std::log(kLogOddsOccupied / (1.0 - kLogOddsOccupied))), //
      m_loFreeInc(std::log(kLogOddsFree / (1.0 - kLogOddsFree)))
{
}

void OccupancyGrid::updateFromScans(const Pose& robot, const std::vector<LaserScan>& scans)
{
    Index index{};
    if (not worldToGrid(robot, index)) {
        return; // Robot off map
    }

    for (auto scan : scans) {
        const Real range = scan.toMeters();
        if (range < kLidarMinRange || range >= kLidarMaxRange) {
            continue;
        }

        const Real worldAngle = scan.toWorldAngle(robot.theta);

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

auto OccupancyGrid::logOddsToColor(Real logOdds) -> uint32_t
{
    const Real clamped    = std::clamp(logOdds, kLogOddsMin, kLogOddsMax);
    const Real normalized = (clamped - kLogOddsMin) / (kLogOddsMax - kLogOddsMin);
    const auto value      = static_cast<uint8_t>(255.0 * (1.0 - normalized));
    return 0xFF000000 | (value << 16) | (value << 8) | value; // NOLINT
}

} // namespace reanaut
