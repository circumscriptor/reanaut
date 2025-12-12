#pragma once
#include "constants.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace reanaut
{

struct Polygon
{
    using Real = RealType;

    std::vector<Point2> vertices;

    Polygon() = default;
    explicit Polygon(const std::vector<Point2>& verts) : vertices(verts) {}

    // --- Accessors ---
    [[nodiscard]] auto size() const -> size_t { return vertices.size(); }
    [[nodiscard]] auto empty() const -> bool { return vertices.empty(); }

    [[nodiscard]]
    auto getVertex(size_t index) const -> Point2;

    [[nodiscard]]
    auto getEdge(size_t index) const -> std::pair<Point2, Point2>;

    // --- Tangent Bug Logic ---

    /**
     * @brief Finds the vector to follow the wall closest to the robot.
     */
    [[nodiscard]] auto getWallFollowVector(Point2 robotPos, bool ccw = true) const -> Point2;

    /**
     * @brief Find Left/Right tangents relative to viewer.
     */
    [[nodiscard]] auto getTangents(Point2 viewer) const -> std::pair<Point2, Point2>;

    // --- Math Helper ---

    [[nodiscard]] static auto distancePointToSegment(Point2 pp, Point2 pa, Point2 pb) -> Real;
};

} // namespace reanaut
