#pragma once

#include "constants.hpp"
#include "depth.hpp"
#include "grid.hpp"

namespace reanaut
{

class ElevationGrid : public Grid
{
public:

    using Real = RealType;

    void update(const DepthProcessor& depth, const Pose& pose);

protected:

    void updateCell(Real pointX, Real pointY, Real pointZ);
};

} // namespace reanaut
