#include "cloud.hpp"
#include "constants.hpp"
#include "laser.hpp"
#include "occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace reanaut
{

OccupancyGrid::OccupancyGrid()
    : m_loOccInc(std::log(kLogOddsOccupied / (1.0 - kLogOddsOccupied))), //
      m_loFreeInc(std::log(kLogOddsFree / (1.0 - kLogOddsFree)))
{
}

auto OccupancyGrid::isOccupied(Point2 world) const -> bool
{
    auto index = worldToGrid(world);
    if (not index) {
        return false;
    }
    return at(*index) > 0.5;
}

auto OccupancyGrid::getProbability(Point2 world) const -> std::optional<Real>
{
    if (auto index = worldToGrid(world); not index) {
        return std::nullopt;
    } else {
        return 1.0 / (1.0 + std::exp(-at(*index)));
    }
}

auto OccupancyGrid::getProbabilitySmooth(Point2 world) const -> std::optional<Real>
{
    auto index = worldToGrid(world);
    if (not index) {
        return std::nullopt;
    }

    // almost certain
    if (at(*index) > (kLogOddsMax / 2.0)) {
        return 1.0;
    }

    Real maxLogOdds = std::numeric_limits<Real>::min();
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            Index check{
                .x = index->x + dx,
                .y = index->y + dy,
            };

            if (auto value = get(check); value) {
                if (value > maxLogOdds) {
                    maxLogOdds = *value;
                }
            }
        }
    }

    return 0.1 + (0.9 / (1.0 + std::exp(-maxLogOdds))); // NOLINT
}

void OccupancyGrid::updateFromScans(const Pose& robot, const std::vector<LaserScan>& scans)
{
    auto index = worldToGrid(robot);
    if (not index) {
        return; // Robot off map
    }

    for (auto scan : scans) {
        const Real range = scan.toMeters();
        if (range < kLidarMinRange || range >= kLidarMaxRange) {
            continue;
        }

        const Real   worldAngle = scan.toWorldAngle(robot.theta);
        const Point2 end{
            .x = robot.x + (range * std::cos(worldAngle)),
            .y = robot.y + (range * std::sin(worldAngle)),
        };

        if (auto endIndex = worldToGrid(end); endIndex) {
            traceLine(*index, *endIndex);
        }
    }
}

void OccupancyGrid::updateFromCloud(const Pose& robot, const PointCloud& cloud)
{
    auto index = worldToGrid(robot);
    if (not index) {
        return; // Robot off map
    }

    for (auto point : cloud.points()) {
        if (auto endIndex = worldToGrid(point); endIndex) {
            traceLine(*index, *endIndex);
        }
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void OccupancyGrid::getVirtualScan(std::vector<VirtualReading>& readings, const Pose& pose, double maxDistance, int numRays) const
{
    readings.clear();
    readings.reserve(numRays);

    const double angleStep = (2.0 * std::numbers::pi) / numRays;

    for (int i = 0; i < numRays; ++i) {
        double angle = -std::numbers::pi + (i * angleStep);

        if (angle > std::numbers::pi) {
            angle -= 2.0 * std::numbers::pi;
        }

        const double worldAngle = std::fmod(pose.theta + angle, 2.0 * std::numbers::pi);
        const double dist       = castRay(Pose({.x = pose.x, .y = pose.y}, worldAngle), maxDistance);
        readings.push_back({.angle    = angle, // Relative angle (Tangent bug needs this)
                            .distance = dist,
                            .worldX   = pose.x + (std::cos(worldAngle) * dist),
                            .worldY   = pose.y + (std::sin(worldAngle) * dist)});
    }
}

auto OccupancyGrid::castRay(const Pose& pose, Real maxDistance) const -> Real
{
    const Real step = resolution();
    const Real dx   = std::cos(pose.theta);
    const Real dy   = std::sin(pose.theta);

    Real dist = 0.0;
    while (dist < maxDistance) {
        const Point2 check{
            .x = pose.x + (dx * dist),
            .y = pose.y + (dy * dist),
        };

        if (isOccupied(check)) {
            return dist;
        }
        dist += step;
    }
    return maxDistance; // No wall found
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
