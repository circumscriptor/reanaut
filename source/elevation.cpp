#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"

#include <cmath>

namespace reanaut
{

void ElevationGrid::update(const DepthProcessor& depth, const Pose& pose)
{
    const double cosTheta = std::cos(pose.theta);
    const double sinTheta = std::sin(pose.theta);

    for (const auto& pt : depth.points()) {
        // 2a. Rotation (2D rotation around Z axis)
        const auto xGlobal = pose.x + (pt.x * cosTheta - pt.y * sinTheta);
        const auto yGlobal = pose.y + (pt.x * sinTheta + pt.y * cosTheta);
        const auto zGlobal = pt.z; // Robot moves on plane, so Global Z = Local Z (assuming flat floor)

        // 2b. Update Elevation Map
        // PDF Page 9, Task 2 Step 5: "If measured value is higher than current cell value... set to current measured Z"

        // Assuming m_map has a method updateElevation(x, y, z)
        // If your map is currently just occupancy (0 or 1), you need to upgrade it to store floats (heights)
        updateCell(xGlobal, yGlobal, zGlobal);
    }
}

void ElevationGrid::updateCell(Real pointX, Real pointY, Real pointZ)
{
    if (auto index = worldToGrid(Point2(pointX, pointY)); index) {
        auto& cell = at(*index);
        if (pointZ >= cell) {
            cell = pointZ;
        } else {
            cell *= 0.5;
        }
    }
}

} // namespace reanaut
