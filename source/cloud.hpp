#pragma once

#include "constants.hpp"
#include "laser.hpp"

#include <span>
#include <vector>

namespace reanaut
{

class PointCloud
{
public:

    void fromScans(const Pose& pose, std::span<LaserScan> scans);

    [[nodiscard]] auto points() const noexcept -> std::span<const Point2> { return m_points; }

private:

    std::vector<Point2> m_points;
};

} // namespace reanaut
