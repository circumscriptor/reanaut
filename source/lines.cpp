#include "constants.hpp"
#include "lines.hpp" // Ensure this header now defines 'struct Polygon'
#include "occupancy.hpp"

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace reanaut
{

void GridImage::update(const OccupancyGrid& grid)
{
    // cv::Mat::create only reallocates if size or type is different.
    m_image.create(grid.height(), grid.width(), CV_8UC1);
    m_resolution = grid.resolution();
    m_origin     = grid.origin();

    // Note: cv::Mat is row-major.
    // Optimization: Get the raw data pointer for the whole block if continuous
    if (m_image.isContinuous()) {
        const auto numPixels = static_cast<const size_t>(grid.width()) * grid.height();
        auto*      pixelPtr  = m_image.ptr<uint8_t>(0);

        for (size_t i = 0; i < numPixels; ++i) {
            // We map linear index back to grid coordinates for the 'get' check
            // (Only needed if 'get' relies strictly on x/y struct)
            // If OccupancyGrid has linear access, this could be even faster.
            Index idx{.x = static_cast<int>(i % grid.width()), .y = static_cast<int>(i / grid.width())};

            if (auto val = grid.get(idx); val && OccupancyGrid::logOddsNormalize(*val) > 0.5) {
                pixelPtr[i] = 255; // Occupied NOLINT
            } else {
                pixelPtr[i] = 0; // Free
            }
        }
    } else {
        // Fallback for non-continuous memory (rare for fresh create())
        for (int y = 0; y < grid.height(); ++y) {
            auto* rowPtr = m_image.ptr<uint8_t>(y);
            for (int x = 0; x < grid.width(); ++x) {
                Index idx{.x = x, .y = y};
                if (auto val = grid.get(idx); val && OccupancyGrid::logOddsNormalize(*val) > 0.5) {
                    rowPtr[x] = 255; // NOLINT
                } else {
                    rowPtr[x] = 0;
                }
            }
        }
    }
}

void TurboUberSuperDetector::extractObstacles(const GridImage& image, const Params& params)
{
    // 1. Prepare
    m_obstacles.clear(); // This is now std::vector<Polygon>

    // We clone because findContours modifies the source image
    cv::Mat processingMap = image.mat().clone();

    // 2. Find Contours
    m_contours.clear();
    // RETR_EXTERNAL: We only care about the outer hull of obstacles for navigation.
    // CHAIN_APPROX_SIMPLE: Compresses horizontal/vertical segments (memory optimization).
    cv::findContours(processingMap, m_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 3. Process & Simplify
    m_approx.clear(); // Reusable buffer

    const Real   resolution = image.resolution();
    const Point2 origin     = image.origin();

    for (const auto& contour : m_contours) {
        // Filter out tiny noise blobs
        if (cv::contourArea(contour) < params.minArea) {
            continue;
        }

        // Ramer-Douglas-Peucker simplification
        // epsilon ~ 1.0 to 2.0 ensures we keep the general shape without pixel-stepping artifacts
        cv::approxPolyDP(contour, m_approx, params.epsilon, true);

        // We need at least 3 points to form a closed polygon (triangle).
        // If walls can be single thin lines (2 points), change to >= 2.
        if (m_approx.size() < 3) {
            continue;
        }

        // Convert OpenCV points (Grid Pixels) -> reanaut::Point2 (World Meters)
        std::vector<Point2> worldVertices;
        worldVertices.reserve(m_approx.size());

        for (const auto& point : m_approx) {
            worldVertices.emplace_back((static_cast<Real>(point.x) * resolution) + origin.x, //
                                       (static_cast<Real>(point.y) * resolution) + origin.y);
        }

        // Store as a Polygon object
        m_obstacles.emplace_back(worldVertices);
    }

    // Note: m_segments (lines) generation is removed because Tangent Bug uses Polygons directly.
    // If you still need lines for visualization, you can generate them from m_obstacles
    // in the draw loop or here.
}

} // namespace reanaut
