#include "constants.hpp"
#include "depth.hpp"

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace reanaut
{

void DepthProcessor::process(const cv::Mat& depthImage)
{
    m_points.clear();
    m_points.reserve(size_t(depthImage.rows) * depthImage.cols);

    // Iterate over the image
    // Assuming CV_8UC1 based on PDF description "1 channel... 8 bit resolution"
    for (int v = 0; v < depthImage.rows; ++v) {
        const auto* rowPtr = depthImage.ptr<uint8_t>(v);

        for (int u = 0; u < depthImage.cols; ++u) {
            uint8_t pixelValue = rowPtr[u];

            // Ignore "too close" or "invalid" readings (usually 0)
            if (pixelValue == 0) {
                continue;
            }

            // 1. Calculate Depth (Z in Camera Frame)
            // PDF Page 7: "To transform the pixel value to depth it is necessary to divide it by 50"
            const Real zCam = static_cast<Real>(pixelValue) / 50.0F;

            // 2. Calculate X/Y in Camera Frame (Equation 9)
            // Note: Standard pinhole model is x = (u - cu) * z / f
            const Real xCam = (static_cast<Real>(u) - kCu) * zCam / kFu;
            const Real yCam = (static_cast<Real>(v) - kCv) * zCam / kFv;

            // 3. Transform to Robot Frame (Equation 11 & Logic)
            // Camera coords: X=Right, Y=Down, Z=Forward
            // Robot coords:  X=Forward, Y=Left, Z=Up

            // Rotation:
            // Robot X = Camera Z
            // Robot Y = -Camera X (Camera X is Right, Robot Y is Left)
            // Robot Z = -Camera Y (Camera Y is Down, Robot Z is Up)
            // Add the 12cm translation
            const Point3 point(zCam + kOffsetX, -xCam, -yCam);
            m_points.push_back(point);
        }
    }
}

// NOLINTBEGIN
auto getHeatmapColor(RealType value) -> uint32_t
{
    // Clamp value between 0.0 and 1.0
    RealType t = std::clamp(value, RealType(0), RealType(1));

    // "Jet" Colormap approximation
    // 4.0 * t - 1.5 shifts the range for each channel
    RealType r = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(3)), RealType(0), RealType(1));
    RealType g = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(2)), RealType(0), RealType(1));
    RealType b = std::clamp(RealType(1.5) - std::abs((RealType(4) * t) - RealType(1)), RealType(0), RealType(1));

    // Convert to 0-255
    auto R = static_cast<uint8_t>(r * RealType(255));
    auto G = static_cast<uint8_t>(g * RealType(255));
    auto B = static_cast<uint8_t>(b * RealType(255));

    // SDL_GPU expects R8G8B8A8, but usually packaged in uint32 as ABGR (Little Endian)
    // or RGBA (Big Endian). Assuming Little Endian (x86/ARM):
    // Byte order in memory: R, G, B, A
    return 0xFF000000 | (static_cast<uint32_t>(B) << 16) | (static_cast<uint32_t>(G) << 8) | static_cast<uint32_t>(R);
}
// NOLINTEND

} // namespace reanaut
