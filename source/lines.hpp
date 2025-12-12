#pragma once

#include "constants.hpp"
#include "occupancy.hpp"

#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

#include <span>
#include <vector>

namespace reanaut
{

struct LineSegment
{
    Point2 start{};
    Point2 end{};
};

class GridImage
{
public:

    using Real = RealType;

    GridImage() = default;

    // Updates the internal image from the occupancy grid.
    // Reallocates memory only if grid dimensions change.
    void update(const OccupancyGrid& grid);

    // Accessor for the extractor
    [[nodiscard]] auto mat() const -> const cv::Mat& { return m_image; }
    [[nodiscard]] auto resolution() const noexcept -> Real { return m_resolution; }
    [[nodiscard]] auto origin() const noexcept -> Point2 { return m_origin; }

private:

    cv::Mat m_image;
    Real    m_resolution;
    Point2  m_origin;
};

class TurboUberSuperDetector
{
public:

    using Real = RealType;

    // Params for HoughLinesP
    // rho: Distance resolution (1 pixel)
    // theta: Angle resolution (1 degree)
    // threshold: Accumulator threshold (min votes to be a line)
    // minLineLength: Minimum length of line (in pixels/cells)
    // maxLineGap: Maximum allowed gap between points to be considered same line
    // NOLINTBEGIN(readability-magic-numbers)
    struct Params
    {
        Real minArea{15.0};
        Real epsilon{2.0};
    };
    // NOLINTEND(readability-magic-numbers)

    // struct Params
    // {
    //     size_t maxIterations{500};
    //     size_t minInliers{15};
    //     Real   distThreshold{0.05};
    //     Real   minLength{0.2};
    // };

    // LineExtractor(const Params& params)
    //     : m_maxIterations(params.maxIterations), m_minInliers(params.minInliers), m_distThreshold(params.distThreshold), m_minLength(params.minLength)
    // {
    // }

    // void extractLines(const OccupancyGrid& grid)
    // {
    //     findOccupiedPoints(grid);
    //     //

    //     while (m_points.size() >= m_minInliers) {
    //     }
    // }

    [[nodiscard]] auto lines() const -> std::span<const LineSegment> { return m_segments; }

    void extractSegments(const GridImage& image, const Params& params);

protected:

    // void findOccupiedPoints(const OccupancyGrid& grid)
    // {
    //     m_points.clear();
    //     m_points.reserve(grid.width() * grid.height() / 10); // NOLINT

    //     for (int y = 0; y < grid.height(); ++y) {
    //         for (int x = 0; x < grid.width(); ++x) {
    //             Point2 pt;
    //             grid.gridToWorld({.x = x, .y = y}, pt);
    //             if (grid.isOccupied(Index(x, y))) {
    //                 m_points.push_back(pt);
    //             }
    //         }
    //     }
    // }

private:

    std::vector<LineSegment>            m_segments;
    std::vector<cv::Vec4i>              m_lines;
    std::vector<cv::Point>              m_approx;
    std::vector<std::vector<cv::Point>> m_contours;
    // std::vector<Point2>      m_points;

    // size_t m_maxIterations;
    // size_t m_minInliers;
    // Real   m_distThreshold;
    // Real   m_minLength;
};

} // namespace reanaut
