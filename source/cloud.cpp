#include "cloud.hpp"
#include "constants.hpp"
#include "laser.hpp"

#include <span>

namespace reanaut
{

void PointCloud::fromScans(const Pose& pose, std::span<LaserScan> scans)
{
    m_points.clear();
    m_points.reserve(scans.size());
    for (const auto& scan : scans) {
        auto point = scan.toWorldPointSafe(pose);
        if (point) {
            m_points.push_back(*point);
        }
    }
}

} // namespace reanaut
