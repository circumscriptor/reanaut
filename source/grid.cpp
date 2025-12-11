#include "constants.hpp"
#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

namespace reanaut
{

auto GridBase::worldToGrid(Point2 world, Index& index) const -> bool
{
    if (world.x < m_originX || world.y < m_originY) {
        return false;
    }
    index.x = static_cast<Index::Type>((world.x - m_originX) / m_resolution);
    index.y = static_cast<Index::Type>((world.y - m_originY) / m_resolution);
    return (index.x >= 0 && index.x < m_width && index.y >= 0 && index.y < m_height);
}

auto GridBase::worldToGrid(Point2 world) const -> std::optional<Index>
{
    Index index;
    if (worldToGrid(world, index)) {
        return index;
    }
    return std::nullopt;
}

void GridBase::gridToWorld(Index index, Point2& world) const
{
    world.x = m_originX + (index.x + 0.5) * m_resolution;
    world.y = m_originY + (index.y + 0.5) * m_resolution;
}

GridBase::GridBase() : m_width(kMapWidth), m_height(kMapHeight), m_resolution(kMapResolution), m_originX(kMapOriginX), m_originY(kMapOriginY) {}

Grid::Grid()
{
    m_grid.resize(size_t(width()) * height(), 0.0); // Initialize to 0 (Unknown probability 0.5)
}

auto Grid::getDistance(const Pose& pose) const -> Real
{
    const Real stepSize = resolution();
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
        if (m_grid[size_t(index.y * width()) + index.x] > 2.0) { // Threshold 2.0 for "confident obstacle"
            return currDist;
        }
        currDist += stepSize;
    }
    return maxDist;
}

void Grid::updateCell(Index index, Real change)
{
    int idx = (index.y * width()) + index.x;
    if (idx >= 0 && idx < int(m_grid.size())) {
        m_grid[idx] += change;
        // Clamp values to prevent overconfidence/underflow
        m_grid[idx] = std::clamp(m_grid[idx], kLogOddsMin, kLogOddsMax);
    }
}

} // namespace reanaut
