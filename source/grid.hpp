#pragma once

#include "constants.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace reanaut
{

class GridBase
{
public:

    using Real = RealType;

    GridBase();

    [[nodiscard]] auto width() const noexcept -> int { return m_width; }
    [[nodiscard]] auto height() const noexcept -> int { return m_height; }
    [[nodiscard]] auto resolution() const noexcept -> Real { return m_resolution; }
    [[nodiscard]] auto origin() const -> Point2 { return {.x = m_originX, .y = m_originY}; }

    [[nodiscard]] auto inBounds(Index index) const -> bool;

    // Convert World (meters) to Grid (index)
    auto worldToGrid(Point2 world, Index& index) const -> bool;

    [[nodiscard]]
    auto worldToGrid(Point2 world) const -> std::optional<Index>;

    // Convert Grid to World (for raycasting check)
    void gridToWorld(Index index, Point2& world) const;

    [[nodiscard]] auto gridToWorld(Index index) const -> Point2;

private:

    int  m_width;
    int  m_height;
    Real m_resolution;
    Real m_originX;
    Real m_originY;
};

template <typename Type>
class TGrid : public GridBase
{
public:

    TGrid();

    // [[nodiscard]] auto grid() -> std::span<Real> { return m_grid; }
    [[nodiscard]] auto grid() const -> std::span<const Type> { return m_grid; }
    [[nodiscard]] auto at(Index index) const -> Type;
    [[nodiscard]] auto at(Index index) -> Type&;
    [[nodiscard]] auto get(Index index) const -> std::optional<Type>;

    void set(Index index, Type value);
    auto setPoint(Point2 world, Type value) -> bool;

    // Returns distance to nearest obstacle in the map from (x,y) at angle theta
    [[nodiscard]] auto getDistance(const Pose& pose, Real maxDistance = kLidarMaxRange) const -> Real;

protected:

    [[nodiscard]] auto grid() -> std::span<Type> { return m_grid; }

    [[nodiscard]]
    auto isObstacle(Index index) const -> bool;
    void updateCell(Index index, Type change);
    void setAtOffset(size_t offset, Type value);

private:

    std::vector<Type> m_grid; // Stores log-odds values
};

template <typename Type>
inline TGrid<Type>::TGrid()
{
    m_grid.resize(size_t(width()) * height(), 0.0); // Initialize to 0 (Unknown probability 0.5)
}

template <typename Type>
inline auto TGrid<Type>::at(Index index) const -> Type
{
    assert(index.x >= 0 && index.x < width());
    assert(index.y >= 0 && index.y < height());
    return m_grid[(size_t(index.y) * width()) + index.x];
}

template <typename Type>
inline auto TGrid<Type>::at(Index index) -> Type&
{
    assert(index.x >= 0 && index.x < width());
    assert(index.y >= 0 && index.y < height());
    return m_grid[(size_t(index.y) * width()) + index.x];
}

template <typename Type>
inline auto TGrid<Type>::get(Index index) const -> std::optional<Type>
{
    if (not inBounds(index)) {
        return std::nullopt;
    }
    return at(index);
}

template <typename Type>
void TGrid<Type>::set(Index index, Type value)
{
    at(index) = value;
}

template <typename Type>
inline auto TGrid<Type>::setPoint(Point2 world, Type value) -> bool
{
    if (auto index = worldToGrid(world); index) {
        at(*index) = value;
        return true;
    }
    return false;
}

template <typename Type>
inline auto TGrid<Type>::getDistance(const Pose& pose, Real maxDistance) const -> Real
{
    const Real stepSize = resolution();

    Real currDist = 0.0;

    const Real dx = std::cos(pose.theta);
    const Real dy = std::sin(pose.theta);

    // Simple stepping raycast (faster than Bresenham for readout)
    while (currDist < maxDistance) {
        const Point2 check{
            .x = pose.x + (dx * currDist),
            .y = pose.y + (dy * currDist),
        };

        Index index{};
        if (not worldToGrid(check, index)) {
            return maxDistance; // Out of bounds is "open space" or max range
        }

        // If log odds > 0, it's likely occupied
        if (m_grid[size_t(index.y * width()) + index.x] > 2.0) { // Threshold 2.0 for "confident obstacle"
            return currDist;
        }
        currDist += stepSize;
    }
    return maxDistance;
}

template <typename Type>
inline void TGrid<Type>::setAtOffset(size_t offset, Type value)
{
    m_grid[offset] = value;
}

using Grid        = TGrid<RealType>;
using BinaryGrid  = TGrid<bool>;
using IntegerGrid = TGrid<int>;

} // namespace reanaut
