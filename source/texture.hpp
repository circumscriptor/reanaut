#pragma once

#include <SDL3/SDL_gpu.h>

#include <cstdint>
#include <span>
#include <vector>

namespace reanaut
{

class TransferTexture
{
public:

    TransferTexture(SDL_GPUDevice* device, uint32_t width, uint32_t height);

    ~TransferTexture();

    [[nodiscard]] auto texture() const noexcept -> SDL_GPUTexture* { return m_texture; }
    [[nodiscard]] auto width() const noexcept -> uint32_t { return m_width; }
    [[nodiscard]] auto height() const noexcept -> uint32_t { return m_height; }
    [[nodiscard]] auto pixels() -> std::span<uint32_t> { return m_pixels; }
    [[nodiscard]] auto pixels() const -> std::span<const uint32_t> { return m_pixels; }

    auto setPixel(int px, int py, uint32_t color) -> bool;

    auto upload(SDL_GPUCommandBuffer* commandBuffer) -> bool;

protected:

    auto copyToTransferBuffer() -> bool;
    auto uploadToGPUTexture(SDL_GPUCommandBuffer* commandBuffer) -> bool;

private:

    SDL_GPUDevice*         m_device{};
    SDL_GPUTexture*        m_texture{};
    SDL_GPUTransferBuffer* m_transferBuffer{};
    uint32_t               m_width{};
    uint32_t               m_height{};
    std::vector<uint32_t>  m_pixels;
};

} // namespace reanaut
