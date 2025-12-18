#include "constants.hpp"
#include "grid.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <optional>

namespace reanaut
{

auto GridBase::inBounds(Index index) const -> bool { return index.x >= 0 && index.x < m_width && index.y >= 0 && index.y < m_height; }

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

template <>
void TGrid<bool>::updateCell(Index index, bool change)
{
    int idx = (index.y * width()) + index.x;
    if (idx >= 0 && idx < int(m_grid.size())) {
        m_grid[idx] = change;
    }
}

template <>
void TGrid<RealType>::updateCell(Index index, Real change)
{
    int idx = (index.y * width()) + index.x;
    if (idx >= 0 && idx < int(m_grid.size())) {
        // Clamp values to prevent overconfidence/underflow
        m_grid[idx] = std::clamp(m_grid[idx] + change, kLogOddsMin, kLogOddsMax);
    }
}

template <>
inline auto TGrid<bool>::isObstacle(Index index) const -> bool
{
    return m_grid[size_t(index.y * width()) + index.x];
}

template <>
inline auto TGrid<RealType>::isObstacle(Index index) const -> bool
{
    return m_grid[size_t(index.y * width()) + index.x] > 2.0;
}

} // namespace reanaut
