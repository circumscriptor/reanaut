#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"
#include "traversability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

namespace reanaut
{

void TraversabilityGrid::update(const ElevationGrid& elevation)
{
    // Safety check
    if (width() != elevation.width() || height() != elevation.height()) {
        // resize(elevation.width(), elevation.height());
        return;
    }

    const auto& rawElev = elevation.grid();

    // 1. PRE-PROCESSING: Smooth the elevation map
    std::vector<Real> smoothElev;
    smoothElev.resize(rawElev.size());
    std::copy(rawElev.begin(), rawElev.end(), smoothElev.begin()); // Copy

    for (int y = 1; y < height() - 1; ++y) {
        for (int x = 1; x < width() - 1; ++x) {
            auto i = (y * width()) + x;
            // Simple 5-point Plus Kernel Average
            // (Center + Up + Down + Left + Right) / 5
            Real sum      = rawElev[i] + rawElev[i - 1] + rawElev[i + 1] + rawElev[i - width()] + rawElev[i + width()];
            smoothElev[i] = (sum * 0.2); // - DepthProcessor::kCameraHeight; // - 0.18;
        }
    }

    const Real kSafeStepHeight = 0.05;
    const Real kCritStepHeight = 0.20;
    const Real kSafeAngleRad   = 20.0 * (std::numbers::pi / 180.0);
    const Real kCritAngleRad   = 45.0 * (std::numbers::pi / 180.0);
    const Real kSafeSlope      = std::tan(kSafeAngleRad);
    const Real kCritSlope      = std::tan(kCritAngleRad);

    for (int y = 2; y < height() - 2; ++y) {
        for (int x = 2; x < width() - 2; ++x) {

            const size_t idx     = (size_t(y) * width()) + x;
            const Real   zCenter = smoothElev[idx];

            // Check immediate neighbors (1 cell away) for sharp steps
            Real maxStep = 0.0;
            maxStep      = std::max(maxStep, std::abs(zCenter - smoothElev[idx - 1]));
            maxStep      = std::max(maxStep, std::abs(zCenter - smoothElev[idx + 1]));
            maxStep      = std::max(maxStep, std::abs(zCenter - smoothElev[idx - width()]));
            maxStep      = std::max(maxStep, std::abs(zCenter - smoothElev[idx + width()]));

            Real stepCost = 0.0;
            if (maxStep <= kSafeStepHeight) {
                stepCost = 0.0;
            } else if (maxStep >= kCritStepHeight) {
                stepCost = 1.0;
            } else {
                stepCost = (maxStep - kSafeStepHeight) / (kCritStepHeight - kSafeStepHeight);
            }

            // Use STRIDE=2 (check 2 cells away).
            // This reduces noise by measuring slope over a wider base (e.g. 10cm vs 5cm).
            const size_t stride = 2;
            const Real   zL     = smoothElev[idx - stride];
            const Real   zR     = smoothElev[idx + stride];
            const Real   zU     = smoothElev[(size_t(y - stride) * width()) + x];
            const Real   zD     = smoothElev[(size_t(y + stride) * width()) + x];

            // Run = 2 * stride * resolution
            const Real dist = (2.0 * static_cast<Real>(stride)) * resolution();
            const Real dzdx = (zR - zL) / dist;
            const Real dzdy = (zD - zU) / dist;

            // Gradient Magnitude (Tangent of angle)
            const Real currentSlope = std::hypot(dzdx, dzdy);

            Real slopeCost = 0.0;
            if (currentSlope <= kSafeSlope) {
                slopeCost = 0.0;
            } else if (currentSlope >= kCritSlope) {
                slopeCost = 1.0;
            } else {
                slopeCost = (currentSlope - kSafeSlope) / (kCritSlope - kSafeSlope);
            }

            // --- AGGREGATE ---
            Real totalCost = std::max(stepCost, slopeCost);

            // Hard clamp for safety
            totalCost = std::clamp(totalCost, 0.0, 1.0);

            setAtOffset(idx, totalCost);
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
