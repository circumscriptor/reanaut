#include "texture.hpp"

#include <SDL3/SDL_gpu.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace reanaut
{

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
TransferTexture::TransferTexture(SDL_GPUDevice* device, uint32_t width, uint32_t height) : m_device(device), m_width(width), m_height(height)
{
    const SDL_GPUTextureCreateInfo textureCreateInfo{
        .type                 = SDL_GPU_TEXTURETYPE_2D,
        .format               = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
        .usage                = SDL_GPU_TEXTUREUSAGE_SAMPLER,
        .width                = m_width,
        .height               = m_height,
        .layer_count_or_depth = 1,
        .num_levels           = 1,
        .sample_count         = SDL_GPU_SAMPLECOUNT_1,
        .props                = 0,
    };

    m_texture = SDL_CreateGPUTexture(m_device, &textureCreateInfo);

    const SDL_GPUTransferBufferCreateInfo bufferCreateInfo{
        .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
        .size  = m_width * m_height * 4,
        .props = 0,
    };

    m_transferBuffer = SDL_CreateGPUTransferBuffer(m_device, &bufferCreateInfo);

    m_pixels.resize(size_t(m_width) * m_height);
}

TransferTexture::~TransferTexture()
{
    if (m_transferBuffer != nullptr) {
        SDL_ReleaseGPUTransferBuffer(m_device, m_transferBuffer);
    }
    if (m_texture != nullptr) {
        SDL_ReleaseGPUTexture(m_device, m_texture);
    }
}

auto TransferTexture::upload(SDL_GPUCommandBuffer* commandBuffer) -> bool { return copyToTransferBuffer() && uploadToGPUTexture(commandBuffer); }

auto TransferTexture::copyToTransferBuffer() -> bool
{
    void* ptr = SDL_MapGPUTransferBuffer(m_device, m_transferBuffer, false);
    if (ptr == nullptr) {
        return false;
    }

    std::memcpy(ptr, m_pixels.data(), m_pixels.size() * sizeof(uint32_t));

    SDL_UnmapGPUTransferBuffer(m_device, m_transferBuffer);
    return true;
}

auto TransferTexture::uploadToGPUTexture(SDL_GPUCommandBuffer* commandBuffer) -> bool
{
    const SDL_GPUTextureTransferInfo src{
        .transfer_buffer = m_transferBuffer,
        .offset          = 0,
        .pixels_per_row  = m_width,
        .rows_per_layer  = m_height,
    };

    const SDL_GPUTextureRegion dst{
        .texture   = m_texture,
        .mip_level = 0,
        .layer     = 0,
        .x         = 0,
        .y         = 0,
        .z         = 0,
        .w         = m_width,
        .h         = m_height,
        .d         = 1,
    };

    SDL_GPUCopyPass* copyPass = SDL_BeginGPUCopyPass(commandBuffer);
    if (copyPass == nullptr) {
        return false;
    }

    SDL_UploadToGPUTexture(copyPass, &src, &dst, false);

    SDL_EndGPUCopyPass(copyPass);
    return true;
}

auto TransferTexture::setPixel(int px, int py, uint32_t color) -> bool
{
    if (px >= 0 && uint32_t(px) <= m_width && py >= 0 && uint32_t(py) <= m_height) {
        m_pixels[size_t(py * m_width) + px] = color;
        return true;
    }
    return false;
}

} // namespace reanaut
