#pragma once

#include "constants.hpp"

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

    [[nodiscard]] auto width() const noexcept { return m_width; }
    [[nodiscard]] auto height() const noexcept { return m_height; }
    [[nodiscard]] auto resolution() const noexcept { return m_resolution; }

    [[nodiscard]] auto inBounds(Index index) const -> bool;

    // Convert World (meters) to Grid (index)
    auto worldToGrid(Point2 world, Index& index) const -> bool;

    [[nodiscard]]
    auto worldToGrid(Point2 world) const -> std::optional<Index>;

    // Convert Grid to World (for raycasting check)
    void gridToWorld(Index index, Point2& world) const;

private:

    int  m_width;
    int  m_height;
    Real m_resolution;
    Real m_originX;
    Real m_originY;
};

class Grid : public GridBase
{
public:

    Grid();

    // [[nodiscard]] auto grid() -> std::span<Real> { return m_grid; }
    [[nodiscard]] auto grid() const -> std::span<const Real> { return m_grid; }
    [[nodiscard]] auto at(Index index) const -> Real;
    [[nodiscard]] auto get(Index index) const -> std::optional<Real>;

    // Returns distance to nearest obstacle in the map from (x,y) at angle theta
    [[nodiscard]] auto getDistance(const Pose& pose, Real maxDistance = kLidarMaxRange) const -> Real;

protected:

    void updateCell(Index index, Real change);

private:

    std::vector<Real> m_grid; // Stores log-odds values
};

} // namespace reanaut
