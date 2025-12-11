#include "constants.hpp"
#include "map.hpp"
#include "occupancy.hpp"
#include "particle.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

#include <algorithm>
#include <cstddef>

namespace reanaut
{

Map::Map(SDL_GPUDevice* device) : m_texture(device, kMapWidth, kMapHeight) {}

void Map::update(const OccupancyGrid& occupancy)
{
    auto grid = occupancy.grid();
    auto map  = m_texture.pixels();

    const size_t size = std::min(grid.size(), map.size());
    for (size_t i = 0; i < size; ++i) {
        map[i] = OccupancyGrid::logOddsToColor(grid[i]);
    }
}

void Map::update(const ParticleFilter& filter)
{
    for (const auto& particle : filter.particles()) {
        Index index;
        if (worldToGrid(particle, index)) {
            m_texture.setPixel(index.x, index.y, 0xFF0000FF); // NOLINT
        }
    }
}

void Map::draw(SDL_GPUCommandBuffer* commandBuffer)
{
    m_texture.upload(commandBuffer);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Map");

    ImVec2 canvasPos  = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize = ImGui::GetContentRegionAvail();

    const float side = std::min(canvasSize.x, canvasSize.y);

    ImVec2 pos = ImVec2(canvasPos.x + ((canvasSize.x - side) * 0.5F), canvasPos.y + ((canvasSize.y - side) * 0.5F));

    ImGui::SetCursorScreenPos(pos);
    ImGui::Image((ImTextureID)m_texture.texture(), ImVec2(side, side));

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRect(pos, ImVec2(pos.x + side, pos.y + side), IM_COL32(255, 255, 255, 255), 0.0F, 0, 2.0F);

    ImGui::End();
    ImGui::PopStyleVar();
}

} // namespace reanaut
