#include "constants.hpp"
#include "depth.hpp"
#include "grid.hpp"
#include "laser.hpp"

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <print>

namespace reanaut
{

// NOLINTNEXTLINE
void DepthProcessor::process(const cv::Mat& depthImage, const Pose& pose, const GridBase& grid, const LaserLookup& lookup)
{
    static constexpr Real kMaxDistance       = 3.0;
    static constexpr int  kMaxPixelValue     = kMaxDistance * 50.0;
    static constexpr Real kEdgeJumpThreshold = 0.2;

    if (depthImage.empty()) {
        return;
    }

    const int  rows     = depthImage.rows;
    const int  cols     = depthImage.cols;
    const Real cosTheta = std::cos(pose.theta);
    const Real sinTheta = std::sin(pose.theta);

    m_observations.clear();
    // heuristic reserve
    m_observations.reserve(size_t(rows * cols) / 10); // NOLINT

    for (int u = 0; u < cols; ++u) {
        const Real xC       = (static_cast<Real>(u) - kCu) / kFu;
        const Real angleRad = std::atan2(-xC, 1.0);
        const int  angleDeg = static_cast<int>(std::round(angleRad * (180.0 / std::numbers::pi)));
        const Real allowedD = lookup.get(angleDeg);
        if (allowedD > kMaxDistance) {
            continue;
        }

        Real lastValidDepth = 0.0;
        bool floorFound     = false;

        for (int v = rows - 1; v >= 0; --v) {
            const auto*   rowPtr     = depthImage.ptr<uint8_t>(v);
            const uint8_t pixelValue = rowPtr[static_cast<ptrdiff_t>(u)];

            if (pixelValue < 1 || pixelValue > kMaxPixelValue) {
                continue;
            }

            // Convert to meters
            const Real depth = static_cast<Real>(pixelValue) / 50.0;

            if (floorFound) {
                // Check if this new pixel is a massive jump away from the previous one.
                if ((depth - lastValidDepth) > kEdgeJumpThreshold) {
                    // STOP! We hit the edge of the ramp.
                    // Everything above this is likely the back wall or ceiling
                    break;
                }
            } else {
                // This is the first valid pixel starting from the bottom
                floorFound = true;
            }

            // Update tracker
            lastValidDepth = depth;

            const Real yC = (static_cast<Real>(v) - kCv) / kFv;
            const Real pX = depth + kOffsetX;
            const Real pY = -xC * depth;
            const Real pZ = (-yC * depth) + kCameraHeight;

            if (pZ < -0.1 || pZ > 1.0) { // NOLINT
                continue;
            }

            const Real xGlobal = pose.x + (pX * cosTheta - pY * sinTheta);
            const Real yGlobal = pose.y + (pX * sinTheta + pY * cosTheta);
            if (const auto index = grid.worldToGrid(Point2(xGlobal, yGlobal))) {
                const size_t szIndex = index->pack();

                auto it = m_observations.find(szIndex);
                if (it == m_observations.end()) {
                    m_observations.emplace(szIndex, pZ);
                } else {
                    it->second = std::max(pZ, it->second);
                }
            }
        }
    }

    // SOMEWHAT WORKING

    // for (int v = 0; v < rows; ++v) {
    //     const auto* rowPtr = depthImage.ptr<uint8_t>(v);
    //     for (int u = 0; u < cols; ++u) {

    //         const uint8_t pixelValue = rowPtr[static_cast<ptrdiff_t>(u)];

    //         // 1a. Range Filtering
    //         // if (pixelValue < 5 || pixelValue > 100) { // NOLINT
    //         //     continue;
    //         // }

    //         // 100 -> 2 meters
    //         if (pixelValue < 1 || pixelValue > 100) { // NOLINT
    //             continue;
    //         }

    //         // 1b. Pixel -> Camera Coordinates
    //         const Real depth = (static_cast<Real>(pixelValue) / Real(50.0)) + 0.001;
    //         const Real xC    = (static_cast<Real>(u) - kCu) / kFu;
    //         const Real yC    = (static_cast<Real>(v) - kCv) / kFv;

    //         const Real angleRad = std::atan2(-xC, 1.0);
    //         const int  angleDeg = static_cast<int>(std::round(angleRad * (180.0 / std::numbers::pi)));
    //         const Real allowedD = lookup.get(angleDeg);
    //         if (allowedD > 2.0) { // 2 meters
    //             continue;         // skip measurements too far
    //         }

    //         // 1c. Camera -> Robot Coordinates (Based on PDF Page 8 Eq 11)
    //         // Rotations: Z(cam) -> X(rob), -X(cam) -> Y(rob), -Y(cam) -> Z(rob)
    //         const Real pX = depth + kOffsetX;
    //         const Real pY = -xC * depth;
    //         // pZ is -yC * depth (negative downwards from camera).
    //         const Real pZ = (-yC * depth); //- kCameraHeight; // We add kCameraHeight to get Z relative to Floor (0.0).

    //         // Filter Ceiling / Floor noise (Optional)
    //         if (pZ > 1.0) {
    //             continue; // Ignore things higher than 1m
    //         }
    //         if (pZ < 0) {
    //             continue; // Ignore holes deeper than 20cm (sensor noise)
    //         }

    //         // 1d. Robot -> Global Coordinates
    //         const Real xGlobal = robotX + (pX * cosTheta - pY * sinTheta);
    //         const Real yGlobal = robotY + (pX * sinTheta + pY * cosTheta);

    //         // 1e. Aggregate
    //         if (const auto index = grid.worldToGrid(Point2(xGlobal, yGlobal)); index) {
    //             const size_t szIndex = index->pack();
    //             auto         it      = m_observations.find(szIndex);
    //             if (it == m_observations.end()) {
    //                 m_observations.emplace(szIndex, pZ);
    //             } else {
    //                 // Keep the highest point in the cell (Conservative obstacle mapping)
    //                 it->second = std::max(pZ, it->second);
    //             }
    //         }
    //     }
    // }
}

} // namespace reanaut
