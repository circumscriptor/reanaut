#include "constants.hpp"
#include "depth.hpp"
#include "grid.hpp"

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <print>

namespace reanaut
{

void DepthProcessor::process(const cv::Mat& depthImage, const Pose& pose, const GridBase& grid)
{
    if (depthImage.empty()) {
        return;
    }

    const int  rows     = depthImage.rows;
    const int  cols     = depthImage.cols; // 3-channel generic container for 8-bit depth
    const Real cosTheta = std::cos(pose.theta);
    const Real sinTheta = std::sin(pose.theta);
    const Real robotX   = pose.x;
    const Real robotY   = pose.y;

    m_observations.clear();
    // heuristic reserve
    m_observations.reserve(size_t(rows * cols) / 10); // NOLINT

    std::println("camera image: {}x{}", rows, cols);

    for (int v = 0; v < rows; ++v) {
        const auto* rowPtr = depthImage.ptr<uint8_t>(v);
        for (int u = 0; u < cols; ++u) {

            const uint8_t pixelValue = rowPtr[static_cast<ptrdiff_t>(u)];

            // 1a. Range Filtering
            // if (pixelValue < 5 || pixelValue > 100) { // NOLINT
            //     continue;
            // }

            if (pixelValue > 100) {
                continue;
            }

            // 1b. Pixel -> Camera Coordinates
            const Real depth = (static_cast<Real>(pixelValue) / Real(50.0)) + 0.001;
            const Real xC    = (static_cast<Real>(u) - kCu) / kFu;
            const Real yC    = (static_cast<Real>(v) - kCv) / kFv;

            // 1c. Camera -> Robot Coordinates (Based on PDF Page 8 Eq 11)
            // Rotations: Z(cam) -> X(rob), -X(cam) -> Y(rob), -Y(cam) -> Z(rob)
            const Real pX = depth + kOffsetX;
            const Real pY = -xC * depth;
            // pZ is -yC * depth (negative downwards from camera).
            const Real pZ = (-yC * depth); //- kCameraHeight; // We add kCameraHeight to get Z relative to Floor (0.0).

            // Filter Ceiling / Floor noise (Optional)
            if (pZ > 1.0) {
                continue; // Ignore things higher than 1m
            }
            if (pZ < 0) {
                continue; // Ignore holes deeper than 20cm (sensor noise)
            }

            // 1d. Robot -> Global Coordinates
            const Real xGlobal = robotX + (pX * cosTheta - pY * sinTheta);
            const Real yGlobal = robotY + (pX * sinTheta + pY * cosTheta);

            // 1e. Aggregate
            if (const auto index = grid.worldToGrid(Point2(xGlobal, yGlobal)); index) {
                const size_t szIndex = index->pack();
                auto         it      = m_observations.find(szIndex);
                if (it == m_observations.end()) {
                    m_observations.emplace(szIndex, pZ);
                } else {
                    // Keep the highest point in the cell (Conservative obstacle mapping)
                    it->second = std::max(pZ, it->second);
                }
            }
        }
    }
}

} // namespace reanaut
