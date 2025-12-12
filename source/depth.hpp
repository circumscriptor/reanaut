#pragma once

#include "constants.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace reanaut
{

struct Point3
{
    using Real = RealType;

    Real x; // Robot Frame Forward
    Real y; // Robot Frame Left
    Real z; // Robot Frame Up
};

class DepthProcessor
{
public:

    using Real = RealType;

    // Constants from PDF Page 8, Equation (10)
    static constexpr Real kFu = Real(430.443939208984);
    static constexpr Real kFv = Real(430.443939208984);
    static constexpr Real kCu = Real(422.133117675781);
    static constexpr Real kCv = Real(234.030456542969);

    // Translation from PDF Page 8: 12cm offset in Robot X
    static constexpr Real kOffsetX = 0.12F;

    [[nodiscard]] auto points() const -> std::span<const Point3> { return m_points; }

    // Convert raw cv::Mat to Robot-Frame Points
    void process(const cv::Mat& depthImage);

private:

    std::vector<Point3> m_points;
};

// Converts a value 0.0 (Low) to 1.0 (High) into 0xAABBGGRR (Little Endian for SDL)
auto getHeatmapColor(RealType value) -> uint32_t;

} // namespace reanaut
