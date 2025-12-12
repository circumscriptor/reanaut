#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"

#include <cmath>

namespace reanaut
{

void ElevationGrid::update(const DepthProcessor& depth, const Pose& pose)
{
    for (const auto& pt : depth.points()) {
        const double cosTheta = std::cos(pose.theta);
        const double sinTheta = std::sin(pose.theta);

        // 2a. Rotation (2D rotation around Z axis)
        auto xGlobal = pose.x + (pt.x * cosTheta - pt.y * sinTheta);
        auto yGlobal = pose.y + (pt.x * sinTheta + pt.y * cosTheta);
        auto zGlobal = pt.z; // Robot moves on plane, so Global Z = Local Z (assuming flat floor)

        // 2b. Update Elevation Map
        // PDF Page 9, Task 2 Step 5: "If measured value is higher than current cell value... set to current measured Z"

        // Assuming m_map has a method updateElevation(x, y, z)
        // If your map is currently just occupancy (0 or 1), you need to upgrade it to store floats (heights)
        setPoint({.x = xGlobal, .y = yGlobal}, zGlobal);
    }
}

} // namespace reanaut
