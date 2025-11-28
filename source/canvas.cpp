#include "canvas.hpp"

#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_timer.h>
#include <SDL3/SDL_video.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlgpu3.h>
#include <implot.h>

#include <cstdint>
#include <format>
#include <print>
#include <stdexcept>

namespace reanaut
{

Canvas::Canvas(int32_t width, int32_t height)
{
    if (not SDL_InitSubSystem(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        throw std::runtime_error(std::format("Could not initialize SDL - {}", SDL_GetError()));
    }

    openWindow(width, height);
    openDevice();
    openImGui();
}

Canvas::~Canvas()
{
    SDL_WaitForGPUIdle(m_device);
    ImGui_ImplSDL3_Shutdown();
    ImGui_ImplSDLGPU3_Shutdown();
    ImPlot::DestroyContext(m_implot);
    ImGui::DestroyContext(m_imgui);

    SDL_ReleaseWindowFromGPUDevice(m_device, m_window);
    SDL_DestroyGPUDevice(m_device);
    SDL_DestroyWindow(m_window);
    SDL_QuitSubSystem(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD);
}

void Canvas::close() noexcept { m_shouldClose = true; }

void Canvas::run(const DrawCallback& callback)
{
    while (not m_shouldClose) {
        ImGui::SetCurrentContext(m_imgui);

        while (SDL_PollEvent(&m_event)) {
            switch (m_event.type) {
                case SDL_EVENT_QUIT:
                case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
                    m_shouldClose = true;
                    break;
                default:
                    ImGui_ImplSDL3_ProcessEvent(&m_event);
                    break;
            }
        }

        if ((SDL_GetWindowFlags(m_window) & SDL_WINDOW_MINIMIZED) != 0U) {
            SDL_Delay(8);
            continue;
        }

        ImGui_ImplSDLGPU3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

        if (callback != nullptr) {
            callback();
        }

        ImGui::Render();
        ImDrawData* drawData = ImGui::GetDrawData();

        SDL_GPUCommandBuffer* commandBuffer = SDL_AcquireGPUCommandBuffer(m_device);
        SDL_GPUTexture*       swapchainTexture;
        SDL_WaitAndAcquireGPUSwapchainTexture(commandBuffer, m_window, &swapchainTexture, nullptr, nullptr);

        if (swapchainTexture != nullptr) {
            ImGui_ImplSDLGPU3_PrepareDrawData(drawData, commandBuffer);

            SDL_GPUColorTargetInfo targetInfo{};
            targetInfo.texture              = swapchainTexture;
            targetInfo.clear_color          = SDL_FColor{0.F, 0.F, 0.F, 1.F};
            targetInfo.load_op              = SDL_GPU_LOADOP_CLEAR;
            targetInfo.store_op             = SDL_GPU_STOREOP_STORE;
            targetInfo.mip_level            = 0;
            targetInfo.layer_or_depth_plane = 0;
            targetInfo.cycle                = false;

            SDL_GPURenderPass* renderPass = SDL_BeginGPURenderPass(commandBuffer, &targetInfo, 1, nullptr);
            ImGui_ImplSDLGPU3_RenderDrawData(drawData, commandBuffer, renderPass);
            SDL_EndGPURenderPass(renderPass);
        }

        SDL_SubmitGPUCommandBuffer(commandBuffer);
    }
}

void Canvas::openWindow(int32_t width, int32_t height)
{
    const auto flags   = SDL_WINDOW_HIDDEN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY;
    const auto display = SDL_GetPrimaryDisplay();
    m_scale            = SDL_GetDisplayContentScale(display);

    if (m_window = SDL_CreateWindow("Reanaut", int32_t(float(width) * m_scale), int32_t(float(height) * m_scale), flags); m_window == nullptr) {
        throw std::runtime_error(std::format("Could not create window - {}", SDL_GetError()));
    }

    if (not SDL_SetWindowPosition(m_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED)) {
        std::println(stderr, "Could not set window position - {}", SDL_GetError());
    }

    if (not SDL_ShowWindow(m_window)) {
        throw std::runtime_error(std::format("Could not show window - {}", SDL_GetError()));
    }
}

void Canvas::openDevice()
{
    const auto flags = SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL | SDL_GPU_SHADERFORMAT_METALLIB;

    if (m_device = SDL_CreateGPUDevice(flags, true, nullptr); m_device == nullptr) {
        throw std::runtime_error(std::format("Could not create GPU device - {}", SDL_GetError()));
    }

    if (not SDL_ClaimWindowForGPUDevice(m_device, m_window)) {
        throw std::runtime_error(std::format("Could not claim window for GPU device - {}", SDL_GetError()));
    }

    if (not SDL_SetGPUSwapchainParameters(m_device, m_window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_VSYNC)) {
        std::println(stderr, "Could not set swapchain parameters - {}", SDL_GetError());
    }
}

void Canvas::openImGui()
{
    IMGUI_CHECKVERSION();
    if (m_imgui = ImGui::CreateContext(); m_imgui == nullptr) {
        throw std::runtime_error("Could not create ImGui context");
    }

    if (m_implot = ImPlot::CreateContext(); m_implot == nullptr) {
        throw std::runtime_error("Could not create ImPlot context");
    }

    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    auto& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(&style);
    style.ScaleAllSizes(m_scale);
    style.FontScaleDpi = m_scale;

    ImGui_ImplSDL3_InitForSDLGPU(m_window);
    ImGui_ImplSDLGPU3_InitInfo initInfo{};
    initInfo.Device               = m_device;
    initInfo.ColorTargetFormat    = SDL_GetGPUSwapchainTextureFormat(m_device, m_window);
    initInfo.MSAASamples          = SDL_GPU_SAMPLECOUNT_1;
    initInfo.SwapchainComposition = SDL_GPU_SWAPCHAINCOMPOSITION_SDR;
    initInfo.PresentMode          = SDL_GPU_PRESENTMODE_VSYNC;
    ImGui_ImplSDLGPU3_Init(&initInfo);
}

} // namespace reanaut
