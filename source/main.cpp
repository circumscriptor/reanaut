#include "canvas.hpp"

#include <imgui.h>
#include <implot.h>

auto main(int argc, char** argv) -> int
{
    (void)argc;
    (void)argv;

    reanaut::Canvas canvas;
    canvas.run([]() {
        ImGui::ShowDemoWindow();
        ImPlot::ShowDemoWindow();
    });
    return 0;
}
