#include "camera.hpp"
#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <cstdint>
#include <print>
#include <string_view>

namespace reanaut
{

Camera::Camera(std::string_view url) : m_url(url), m_capture(m_url) { std::println("Camera stream opened: {}", m_url); }

auto Camera::capture() -> const cv::Mat&
{
    m_capture >> m_buffer;
    cv::flip(m_buffer, m_buffer, -1);
    return m_buffer;
}

// Expected camera resolution is 848x480
CameraTexture::CameraTexture(SDL_GPUDevice* device) : TransferTexture(device, 848, 480) // NOLINT
{
}

void CameraTexture::update(const cv::Mat& sourceMat)
{
    if (sourceMat.empty()) {
        return;
    }

    // If your camera changes resolution dynamically, you would need to add a resize()
    // method to your base TransferTexture class to recreate resources.
    if (static_cast<uint32_t>(sourceMat.cols) != width() || static_cast<uint32_t>(sourceMat.rows) != height()) {
        std::println(stderr, "Error: Camera frame size ({}x{}) does not match Texture size ({}x{})", sourceMat.cols, sourceMat.rows, width(), height());
        return;
    }

    // SDL Texture is RGBA (4 bytes). We must convert input to this format.
    if (sourceMat.type() == CV_8UC3) {
        // Standard Webcam (BGR) -> RGBA
        cv::cvtColor(sourceMat, m_staging, cv::COLOR_BGR2RGBA);
    } else if (sourceMat.type() == CV_8UC4) {
        // Already BGRA or RGBA (just copy to ensure contiguous data if needed)
        // Assuming input is BGRA (OpenCV default), convert to RGBA
        cv::cvtColor(sourceMat, m_staging, cv::COLOR_BGRA2RGBA);
    } else if (sourceMat.type() == CV_8UC1) {
        // Grayscale / IR -> RGBA
        cv::cvtColor(sourceMat, m_staging, cv::COLOR_GRAY2RGBA);
    } else if (sourceMat.type() == CV_16UC1) {
        // Depth Camera (16-bit) -> Visualization
        // We normalize the 16-bit data to 8-bit and apply a colormap for visibility

        cv::Mat adjMap;

        // Normalize: 0 to ~4 meters (4000mm) mapped to 0-255.
        // Adjust alpha (scale) based on your depth range.
        sourceMat.convertTo(adjMap, CV_8UC1, 255.0 / 4000.0); // NOLINT

        // Optional: Apply ColorMap (needs headers) or just Grayscale
        // cv::applyColorMap(adjMap, m_staging, cv::COLORMAP_JET);

        // Simple Grayscale fallback for speed:
        cv::cvtColor(adjMap, m_staging, cv::COLOR_GRAY2RGBA);
    } else {
        // Fallback for unknown types (e.g., float)
        std::println(stderr, "Unsupported Mat type for texture upload");
        return;
    }
}

void CameraTexture::upload(SDL_GPUCommandBuffer* commandBuffer)
{
    if (m_staging.empty()) {
        std::println(stderr, "Error: Cannot upload empty texture");
        return;
    }

    size_t dataSize = m_staging.total() * m_staging.elemSize();
    TransferTexture::upload(commandBuffer, m_staging.data, dataSize);
}

void CameraTexture::draw(SDL_GPUCommandBuffer* commandBuffer)
{
    upload(commandBuffer);

    if (ImGui::Begin("Camera")) {
        const ImVec2 size = ImGui::GetContentRegionAvail();
        if (size.x > 0 && size.y > 0 && width() > 0 && height() > 0) {

            float aspectWindow = size.x / size.y;
            float aspectImage  = static_cast<float>(width()) / static_cast<float>(height());

            ImVec2 imageSize;
            if (aspectWindow > aspectImage) {
                // Window is wider than image -> Fit to Height
                imageSize.y = size.y;
                imageSize.x = imageSize.y * aspectImage;
            } else {
                // Window is taller than image -> Fit to Width
                imageSize.x = size.x;
                imageSize.y = imageSize.x / aspectImage;
            }

            const float  xOffset = (size.x - imageSize.x) * 0.5F;
            const float  yOffset = (size.y - imageSize.y) * 0.5F;
            const ImVec2 cursor  = ImGui::GetCursorPos();
            const ImVec2 pos(cursor.x + xOffset, cursor.y + yOffset);
            ImGui::SetCursorPos(pos);
            ImGui::Image((ImTextureID)texture(), imageSize);

            ImDrawList* drawList = ImGui::GetWindowDrawList();
            drawList->AddRect(pos, ImVec2(pos.x + imageSize.x, pos.y + imageSize.y), IM_COL32(255, 0, 255, 255), 0.0F, 0, 2.0F);
        }

        // const float vw = ImGui::GetContentRegionAvail().x;
        // const float vh = vw * (float(height()) / float(width()));
        // ImGui::SetCursorScreenPos(ImVec2()); // center image
        // ImGui::Image((ImTextureID)texture(), ImVec2(vw, vh));
    }
    ImGui::End();
}

} // namespace reanaut
