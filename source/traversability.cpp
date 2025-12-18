#include "constants.hpp"
#include "elevation.hpp"
#include "traversability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <print>

namespace reanaut
{

void TraversabilityGrid::update(const ElevationGrid& elevation)
{
    const auto& elev     = elevation.grid();
    const Real  maxSlope = std::tan(kMaxSlopeAngle);

    // Iterate grid (Skipping 1-pixel border to avoid boundary checks)
    // This leaves the border cells as 0.0 (or you could set them to 1.0 safety)
    for (int y = 1; y < height() - 1; ++y) {
        for (int x = 1; x < width() - 1; ++x) {

            const size_t centerIdx = (size_t(y) * width()) + x;
            const Real   zCenter   = elev[centerIdx];

            // --- 1. Fetch Neighbors ---
            const Real zUp    = elev[(size_t(y - 1) * width()) + x];
            const Real zDown  = elev[(size_t(y + 1) * width()) + x];
            const Real zLeft  = elev[centerIdx - 1];
            const Real zRight = elev[centerIdx + 1];

            // --- 2. Metric 1: Max Height Difference (Step) ---
            const Real maxDiff = std::max({
                0.0,
                std::abs(zCenter - zUp),
                std::abs(zCenter - zDown),
                std::abs(zCenter - zLeft),
                std::abs(zCenter - zRight),
            });

            // Normalize Step Cost
            Real stepCost = Real(0.0);
            if (maxDiff >= kMaxStepHeight) {
                stepCost = Real(1.0); // Too high to climb
            } else {
                stepCost = maxDiff / kMaxStepHeight; // Linear difficulty
            }

            // --- 3. Metric 2: Slope (Tilt) ---
            // Central Difference
            const Real dzdx = (zRight - zLeft) / (Real(2.0) * resolution());
            const Real dzdy = (zDown - zUp) / (Real(2.0) * resolution());

            // Gradient Magnitude
            const Real slopeMag = std::hypot(dzdx, dzdy);

            Real slopeCost = Real(0.0);
            if (slopeMag >= maxSlope) {
                slopeCost = Real(1.0); // Too steep
            } else {
                slopeCost = slopeMag / maxSlope;
            }

            // --- 4. Combination (Max Rule) ---
            setAtOffset(centerIdx, std::max(stepCost, slopeCost));
            // setAtOffset(centerIdx, stepCost);
            if ((stepCost > 0 && stepCost < 1) || (slopeCost > 0 && stepCost < 1)) {
                std::println("step cost: {}, slope cost: {}", stepCost, slopeCost);
            }
        }
    }
}

// NOLINTBEGIN
auto getTraversabilityColor(RealType cost) -> uint32_t
{
    const auto t = std::clamp(cost, RealType(0), RealType(1));
    const auto r = static_cast<uint8_t>(t * 255);
    const auto g = static_cast<uint8_t>((RealType(1) - t) * 255);
    const auto b = uint8_t(0);
    return 0xFF000000 | (static_cast<uint32_t>(b) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(r);
}
// NOLINTEND

} // namespace reanaut
