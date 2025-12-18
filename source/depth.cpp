#include "depth.hpp"

// We need the full definitions here
#include "constants.hpp" // For Pose struct
#include "grid.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace reanaut
{

void DepthProcessor::process(const cv::Mat& depthImage, const Pose& pose, const GridBase& grid)
{
    if (depthImage.empty()) {
        return;
    }

    const int rows = depthImage.rows;
    const int cols = depthImage.cols;

    // Precompute Pose Transforms
    const Real cosTheta = std::cos(pose.theta);
    const Real sinTheta = std::sin(pose.theta);
    const Real robotX   = pose.x;
    const Real robotY   = pose.y;

    // Heuristic reserve: assuming we hit ~10-20% unique cells vs pixels
    m_observations.clear();
    m_observations.reserve(size_t(rows * cols) / 16);

    for (int v = 0; v < rows; ++v) {
        const auto* rowPtr = depthImage.ptr<uint8_t>(v);

        // Optimization: Step 3 pixels (reduce CPU load)
        for (int u = 0; u < cols; u += 3) {
            const uint8_t pixelValue = rowPtr[u];

            // 1a. Range Filtering
            if (pixelValue < 5 || pixelValue > 100) { // NOLINT
                continue;
            }

            // 1b. Pixel -> Camera Coordinates
            const Real zCam = static_cast<Real>(pixelValue) / Real(50.0);
            const Real xCam = (static_cast<Real>(u) - kCu) * zCam / kFu;
            const Real yCam = (static_cast<Real>(v) - kCv) * zCam / kFv;

            // 1c. Camera -> Robot Coordinates
            const Real pX = zCam + kOffsetX;       // Forward
            const Real pY = -xCam;                 // Left
            const Real pZ = -yCam + kCameraHeight; // Up

            // Filter points that are underground (noise)
            if (pZ < 0) {
                continue;
            }

            // 1d. Robot -> Global Coordinates (Rotation + Translation)
            const Real xGlobal = robotX + (pX * cosTheta - pY * sinTheta);
            const Real yGlobal = robotY + (pX * sinTheta + pY * cosTheta);

            // 1e. Aggregate
            // We ask the map for the index, but we don't update the cell yet.
            if (const auto index = grid.worldToGrid(Point2(xGlobal, yGlobal)); index) {
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
}

// NOLINTBEGIN
auto getHeatmapColor(RealType value) -> uint32_t
{
    RealType   t = std::clamp(value, RealType(0), RealType(1));
    const auto r = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(3)), RealType(0), RealType(1));
    const auto g = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(2)), RealType(0), RealType(1));
    const auto b = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(1)), RealType(0), RealType(1));
    return 0xFF000000 | (static_cast<uint32_t>(b * 255) << 16) | (static_cast<uint32_t>(g * 255) << 8) | static_cast<uint32_t>(r * 255);
}
// NOLINTEND

} // namespace reanaut
