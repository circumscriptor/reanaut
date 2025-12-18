#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "traversability.hpp"

#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace reanaut
{

class WavefrontPlanner : public IntegerGrid
{
public:

    // Main function: Returns a list of waypoints (in World Meters)
    // Returns empty vector if no path found.
    auto findPath(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld) -> bool;

    [[nodiscard]] auto path() const -> std::span<const Point2> { return m_path; }

private:

    std::vector<Point2> m_path;
    std::deque<Index>   m_queue;
};

auto getWaveColor(int value) -> uint32_t;

} // namespace reanaut
