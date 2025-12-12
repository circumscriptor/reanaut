#include "constants.hpp"
#include "lines.hpp"
#include "occupancy.hpp"

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdint>
#include <print>
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
    uint8_t* rowPtr;
    for (int y = 0; y < grid.height(); ++y) {
        rowPtr = m_image.ptr<uint8_t>(y);
        for (int x = 0; x < grid.width(); ++x) {
            Index idx{.x = x, .y = y};

            // If get() returns nullopt (out of bounds), we treat as free.
            if (auto val = grid.get(idx); val && OccupancyGrid::logOddsNormalize(*val) > 0.5) {
                rowPtr[x] = 255; // Occupied NOLINT
            } else {
                rowPtr[x] = 0; // Free
            }
        }
    }
}

void TurboUberSuperDetector::extractSegments(const GridImage& image, const Params& params)
{
    m_lines.clear();

    cv::Mat processingMap = image.mat().clone();
    // cv::Mat kernel        = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Remove salt-and-pepper noise (Speckles)
    // cv::morphologyEx(processingMap, processingMap, cv::MORPH_OPEN, kernel);
    // Fill small holes/gaps in walls
    // cv::morphologyEx(processingMap, processingMap, cv::MORPH_CLOSE, kernel);

    m_contours.clear();
    // B. Find Contours
    // RETR_LIST: Get all contours (don't care about hierarchy)
    // CHAIN_APPROX_SIMPLE: Compresses horizontal/vertical segments
    cv::findContours(processingMap, m_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // C. Simplify & Store
    for (const auto& contour : m_contours) {
        // Filter out tiny noise blobs (e.g. less than 15 pixels area)
        if (cv::contourArea(contour) < params.minArea) {
            continue;
        }

        m_approx.clear();
        // Ramer-Douglas-Peucker algorithm
        // epsilon: Maximum distance from original curve to simplified line.
        // Smaller = follows grid jaggedness (staircase). Larger = smoother lines.
        // 2.0 pixels is usually a good sweet spot for grid maps.
        cv::approxPolyDP(contour, m_approx, params.epsilon, true);

        // Convert the polygon loop into individual line segments for rendering
        for (size_t i = 0; i < m_approx.size(); ++i) {
            cv::Point p1 = m_approx[i];
            cv::Point p2 = m_approx[(i + 1) % m_approx.size()]; // Wrap around to close loop

            // Store as Vec4i [x1, y1, x2, y2]
            m_lines.emplace_back(p1.x, p1.y, p2.x, p2.y);
        }
    }

    // cv::HoughLinesP(image.mat(), m_lines, params.rho, params.theta, params.threshold, params.minLineLength, params.maxLineGap);

    // std::println("Lines: {}", m_lines.size());

    m_segments.clear();
    m_segments.reserve(m_lines.size());
    for (const auto& line : m_lines) {
        // line = [x1, y1, x2, y2] in pixels
        const Point2 p1{.x = (line[0] * image.resolution()) + image.origin().x, .y = (line[1] * image.resolution()) + image.origin().y};
        const Point2 p2{.x = (line[2] * image.resolution()) + image.origin().x, .y = (line[3] * image.resolution()) + image.origin().y};
        m_segments.push_back({p1, p2});
    }
}

} // namespace reanaut
