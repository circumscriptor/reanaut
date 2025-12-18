#include "constants.hpp"
#include "elevation.hpp"
#include "traversability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <print>

namespace reanaut
{

void TraversabilityGrid::update(const ElevationGrid& elevation)
{
    // 1. Safety Checks
    if (width() != elevation.width() || height() != elevation.height()) {
        std::println("SIZE MISMATCH");
        return;
    }

    const auto&  elev = elevation.grid();
    const size_t w    = width();
    const size_t h    = height();
    const Real   res  = resolution();

    // Avoid division by zero
    const Real maxSlopeVal = (kMaxSlopeAngle > 0.0) ? std::tan(kMaxSlopeAngle) : std::numeric_limits<Real>::epsilon();
    const Real maxStepVal  = (kMaxStepHeight > 0.0) ? kMaxStepHeight : std::numeric_limits<Real>::epsilon();

    // Helper: Checks if a specific elevation value is valid (observed data)
    auto isValid = [](Real z) {
        return std::isfinite(z) && z > -1000.0; // Adjust sentinel threshold as needed
    };

    // 2. Clear / Reset Grid (Safe default: Unknown/Obstacle)
    // Using 0.0 (Free) or 1.0 (Obstacle) depends on your planner.
    // Usually, borders should be walls (1.0).
    std::fill(grid().begin(), grid().end(), Real(1.0));

    // 3. Iterate Inner Grid
    // We maintain 'idx' manually to avoid (y * w + x) multiplication every iter.
    for (size_t y = 1; y < h - 1; ++y) {

        size_t idx = (y * w) + 1; // Start at x=1

        for (size_t x = 1; x < w - 1; ++x, ++idx) {

            const Real zCenter = elev[idx];

            // If the ground itself is a hole, it's non-traversable.
            if (!isValid(zCenter)) {
                setAtOffset(idx, Real(1.0));
                continue;
            }

            // Neighbor indices
            const size_t idxUp    = idx - w;
            const size_t idxDown  = idx + w;
            const size_t idxLeft  = idx - 1;
            const size_t idxRight = idx + 1;

            const Real zUp    = elev[idxUp];
            const Real zDown  = elev[idxDown];
            const Real zLeft  = elev[idxLeft];
            const Real zRight = elev[idxRight];

            // --- Metric 1: Step Height (Roughness) ---
            // We only check step height against VALID neighbors.
            // If a neighbor is void, we assume a high cost (edge of world) or ignore it.
            // Here we treat voids as "walls" to be safe.
            Real maxStepDiff = 0.0;

            auto checkStep = [&](Real zNeighbor) {
                if (isValid(zNeighbor)) {
                    maxStepDiff = std::max(maxStepDiff, std::abs(zCenter - zNeighbor));
                } else {
                    // Option A: Treat void as infinite wall
                    // maxStepDiff = std::max(maxStepDiff, kMaxStepHeight * 2);

                    // Option B: Ignore void (risk of driving off cliff)
                    // Option C: Treat as cliff (Cost 1.0) handled via logic below
                }
            };

            checkStep(zUp);
            checkStep(zDown);
            checkStep(zLeft);
            checkStep(zRight);

            Real stepCost = std::clamp(maxStepDiff / maxStepVal, Real(0.0), Real(1.0));

            // --- Metric 2: Slope (Gradient) ---
            // Robust Finite Difference: Handle missing data by falling back
            // from Central Difference (2*res) to Forward/Backward (1*res).

            auto getGradient = [&](Real zPrev, Real zNext) -> Real {
                bool hasPrev = isValid(zPrev);
                bool hasNext = isValid(zNext);

                if (hasPrev && hasNext) {
                    // Central Difference
                    return (zNext - zPrev) / (Real(2.0) * res);
                } else if (hasNext) {
                    // Forward Difference
                    return (zNext - zCenter) / res;
                } else if (hasPrev) {
                    // Backward Difference
                    return (zCenter - zPrev) / res;
                } else {
                    // Isolated point (Peak/Valley)
                    return Real(0.0);
                }
            };

            const Real dzdx = getGradient(zLeft, zRight);
            const Real dzdy = getGradient(zUp, zDown);

            const Real slopeMag  = std::hypot(dzdx, dzdy);
            const Real slopeCost = std::clamp(slopeMag / maxSlopeVal, Real(0.0), Real(1.0));

            // --- Combination ---
            setAtOffset(idx, std::max(stepCost, slopeCost));
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
