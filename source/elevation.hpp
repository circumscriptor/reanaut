#pragma once

#include "constants.hpp"
#include "depth.hpp"
#include "grid.hpp"

namespace reanaut
{

class ElevationGrid : public Grid
{
public:

    void update(const DepthProcessor& depth, const Pose& pose);
};

} // namespace reanaut
