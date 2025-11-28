#pragma once

#include <SDL3/SDL.h>
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gpu.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_video.h>
#include <imgui.h>
#include <implot.h>

#include <cstdint>
#include <functional>

namespace reanaut
{

class Canvas
{
public:

    static constexpr int32_t kDefaultWidth  = 1280;
    static constexpr int32_t kDefaultHeight = 720;

    using DrawCallback = std::function<void()>;

    Canvas(int32_t width = kDefaultWidth, int32_t height = kDefaultHeight);
    ~Canvas();

    void close() noexcept;

    void run(const DrawCallback& callback);

protected:

    void openWindow(int32_t width, int32_t height);
    void openDevice();
    void openImGui();

private:

    SDL_Window*    m_window{};
    SDL_GPUDevice* m_device{};
    SDL_Event      m_event{};
    ImGuiContext*  m_imgui{};
    ImPlotContext* m_implot{};
    float          m_scale{};
    bool           m_shouldClose{};
};

} // namespace reanaut
