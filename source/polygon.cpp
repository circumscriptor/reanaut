#include "constants.hpp"
#include "polygon.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace reanaut
{

auto Polygon::getVertex(size_t index) const -> Point2 { return vertices[index % vertices.size()]; }

auto Polygon::getEdge(size_t index) const -> std::pair<Point2, Point2> { return {vertices[index % vertices.size()], vertices[(index + 1) % vertices.size()]}; }

auto Polygon::getWallFollowVector(Point2 robotPos, bool ccw) const -> Point2
{
    if (vertices.size() < 2) {
        return {
            .x = 0,
            .y = 0,
        };
    }

    size_t bestIdx = 0;
    Real   minDst  = std::numeric_limits<Real>::max();

    // Find closest edge
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto [p1, p2] = getEdge(i);
        const Real dist     = distancePointToSegment(robotPos, p1, p2);
        if (dist < minDst) {
            minDst  = dist;
            bestIdx = i;
        }
    }

    // Calculate direction of the closest edge
    const auto [p1, p2] = getEdge(bestIdx);
    Point2 edgeVec      = p2 - p1;

    const Real len = edgeVec.length();
    if (len < 1e-6) {
        return {
            .x = 0,
            .y = 0,
        };
    }

    // Normalize
    // Assuming Point2 is {x, y}
    const Point2 dir = {
        .x = edgeVec.x / len,
        .y = edgeVec.y / len,
    };

    // Flip if following clockwise (assuming polygon winding is CCW)
    if (!ccw) {
        return {
            .x = -dir.x,
            .y = -dir.y,
        };
    }
    return dir;
}

auto Polygon::getTangents(Point2 viewer) const -> std::pair<Point2, Point2>
{
    if (vertices.empty()) {
        return {viewer, viewer};
    }

    size_t minIdx   = 0;
    size_t maxIdx   = 0;
    Real   minAngle = std::numeric_limits<Real>::max();
    Real   maxAngle = std::numeric_limits<Real>::lowest();

    for (size_t i = 0; i < vertices.size(); ++i) {
        Point2 vec   = vertices[i] - viewer;
        Real   angle = std::atan2(vec.y, vec.x); // atan2 still needed for angle

        if (angle < minAngle) {
            minAngle = angle;
            minIdx   = i;
        }
        if (angle > maxAngle) {
            maxAngle = angle;
            maxIdx   = i;
        }
    }

    return {vertices[minIdx], vertices[maxIdx]};
}

auto Polygon::distancePointToSegment(Point2 pp, Point2 pa, Point2 pb) -> Real
{
    Real l2 = pa.distanceSq(pb);
    if (l2 == 0.0) {
        return pp.distance(pa);
    }

    // Project p onto line ab: t = dot(p-a, b-a) / |b-a|^2
    Point2 ap = pp - pa;
    Point2 ab = pb - pa;

    Real st = ap.dot(ab) / l2;
    st      = std::clamp(st, static_cast<Real>(0.0), static_cast<Real>(1.0));

    // Projection point
    // Note: Point2 doesn't have scalar mul or operator+, implementing manually
    const Point2 projection = {.x = pa.x + (st * ab.x), .y = pa.y + (st * ab.y)};
    return pp.distance(projection);
}
} // namespace reanaut
