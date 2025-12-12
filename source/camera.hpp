#pragma once

#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <string>
#include <string_view>
namespace reanaut
{

class Camera
{
public:

    Camera(std::string_view url);

    auto capture() -> const cv::Mat&;

    [[nodiscard]] auto buffer() const -> const cv::Mat& { return m_buffer; }

private:

    std::string      m_url;
    cv::VideoCapture m_capture;
    cv::Mat          m_buffer;
};

class CameraTexture : public TransferTexture
{
public:

    using TransferTexture::TransferTexture;

    CameraTexture(SDL_GPUDevice* device);

    void update(const cv::Mat& sourceMat);
    void upload(SDL_GPUCommandBuffer* commandBuffer);

    void draw(SDL_GPUCommandBuffer* commandBuffer);

private:

    // Intermediate buffer to hold RGBA data before upload
    cv::Mat m_staging;
};

} // namespace reanaut
