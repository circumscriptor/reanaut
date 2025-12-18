#pragma once

#include "constants.hpp"
#include "depth.hpp"
#include "grid.hpp"

#include <cstdint>

namespace reanaut
{

class ElevationGrid : public Grid
{
public:

    using Real = RealType;

    void update(const DepthProcessor& depth);

protected:

    void updateCell(Real pointX, Real pointY, Real pointZ);
    void updateCellByIndex(Index index, Real measuredZ);
};

// Converts a value 0.0 (Low) to 1.0 (High) into 0xAABBGGRR (Little Endian for SDL)
auto getElevationColor(RealType value) -> uint32_t;

} // namespace reanaut
