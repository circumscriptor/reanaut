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
    const int cols = depthImage.cols / 3;

    // Precompute Pose Transforms
    const Real cosTheta = std::cos(pose.theta * 2.0);
    const Real sinTheta = std::sin(pose.theta * 2.0);
    const Real robotX   = pose.x;
    const Real robotY   = pose.y;

    // Heuristic reserve: assuming we hit ~10-20% unique cells vs pixels
    m_observations.clear();
    m_observations.reserve(size_t(rows * cols) / 10); // NOLINT

    for (int v = 0; v < rows; ++v) {
        const auto* rowPtr = depthImage.ptr<uint8_t>(v);
        for (int u = 0; u < cols; ++u) {
            const uint8_t pixelValue = rowPtr[u * 3]; // NOLINT

            // 1a. Range Filtering
            if (pixelValue < 5 || pixelValue > 150) { // NOLINT
                continue;
            }

            // 1b. Pixel -> Camera Coordinates
            const Real depth = static_cast<Real>(pixelValue) / Real(50.0);
            const Real xC    = (static_cast<Real>(u) - kCu) / kFu;
            const Real yC    = (static_cast<Real>(v) - kCv) / kFv;

            // 1c. Camera -> Robot Coordinates
            const Real pX = depth + kOffsetX; // Forward
            const Real pY = -xC * depth;      // Left
            const Real pZ = -yC * depth;      // Up

            // Filter points that are underground (noise)
            // if (pZ < 0) {
            //     continue;
            // }

            // 1d. Robot -> Global Coordinates (Rotation + Translation)
            const Real xGlobal = robotX + (pX * cosTheta - pY * sinTheta);
            const Real yGlobal = robotY + (pX * sinTheta + pY * cosTheta);

            // const auto xGlobal = pX;
            // const auto yGlobal = pY;

            // 1e. Aggregate
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

} // namespace reanaut
