#include "occupancy.hpp"
#include "tangent.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <span>

namespace reanaut
{

void TangentBug::findDiscontinuities(std::span<VirtualReading> scan, Real jumpThreshold)
{
    // Scan is circular, so handle wrap-around could be complex,
    // but typically we just iterate 0 to N-1

    m_discontinuities.clear();

    for (size_t i = 0; i < scan.size(); ++i) {
        // Get current and next point (wrapping around)
        const auto& curr = scan[i];
        const auto& next = scan[(i + 1) % scan.size()];

        Real diff = std::abs(curr.distance - next.distance);

        // If distance jumps significantly, we found an edge
        if (diff > jumpThreshold) {
            // Determine if it's a "Left" or "Right" tangent relative to the robot
            // (This logic depends on whether you are looking for the 'near' or 'far' point)
            if (curr.distance < next.distance) {
                // Current is the obstacle edge, Next is free space (background)
                m_discontinuities.push_back({curr.angle, curr.distance, Discontinuity::RightEdge});
            } else {
                // Current is background, Next is the obstacle edge
                m_discontinuities.push_back({next.angle, next.distance, Discontinuity::LeftEdge});
            }
        }
    }

    // Also add the point of minimum distance (closest obstacle point)
    // Tangent bug usually tracks this to switch modes
    auto minIt = std::min_element(scan.begin(), scan.end(), [](const auto& lhs, const auto& rhs) { return lhs.distance < rhs.distance; });

    // NOLINTNEXTLINE
    if (minIt != scan.end() && minIt->distance < 10.0) { // arbitrary max check
        m_discontinuities.push_back({minIt->angle, minIt->distance, Discontinuity::MinLocal});
    }
}

} // namespace reanaut
