#pragma once

#include "constants.hpp"
#include "elevation.hpp"
#include "grid.hpp"

#include <cstdint>

namespace reanaut
{

class TraversabilityGrid : public Grid
{
public:

    // Main Algorithm: Converts Elevation -> Cost (0.0 to 1.0)
    void update(const ElevationGrid& elevation);
};

// Visualization: 0.0 (Green/Safe) -> 1.0 (Red/Blocked)
auto getTraversabilityColor(RealType cost) -> uint32_t;

} // namespace reanaut
