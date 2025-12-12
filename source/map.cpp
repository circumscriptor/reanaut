#include "cloud.hpp"
#include "constants.hpp"
#include "lines.hpp"
#include "map.hpp"
#include "occupancy.hpp"
#include "particle.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace reanaut
{

Map::Map(SDL_GPUDevice* device) : m_texture(device, kMapWidth, kMapHeight) {}

void Map::update(const OccupancyGrid& occupancy, bool gradient)
{
    auto grid = occupancy.grid();
    auto map  = m_texture.pixels();

    const size_t size = std::min(grid.size(), map.size());

    if (gradient) {
        for (size_t i = 0; i < size; ++i) {
            map[i] = OccupancyGrid::logOddsToColor(grid[i]);
        }
    } else {
        for (size_t i = 0; i < size; ++i) {
            const auto odds = OccupancyGrid::logOddsNormalize(grid[i]);

            if (odds > 0.5) {
                map[i] = 0xFF000000; // NOLINT
            } else {
                map[i] = 0xFFFFFFFF; // NOLINT
            }
        }
    }
}

void Map::update(const ParticleFilter& filter, Real resolution, bool enableFilter)
{
    if (enableFilter) {
        for (const auto& particle : filter.particles()) {
            Index index;
            if (worldToGrid(particle, index)) {
                m_texture.setPixel(index.x, index.y, 0xFF0000FF); // NOLINT
            }
        }
    }

    const auto bestEstimate = filter.getBestEstimate();
    m_bestEstimate.emplace(Point2(bestEstimate.x / resolution, bestEstimate.y / resolution), bestEstimate.theta);
    m_robotSize = 0.5 / resolution;

    if (enableFilter) {
        Index index;
        if (worldToGrid(bestEstimate, index)) {
            m_texture.setPixel(index.x, index.y, 0xFF00FF00); // NOLINT
        }
    }
}

void Map::update(const PointCloud& cloud)
{
    for (const auto& point : cloud.points()) {
        if (auto index = worldToGrid(point); index) {
            m_texture.setPixel(index->x, index->y, 0xFF0088FF); // NOLINT
        }
    }
}

void Map::update(const TurboUberSuperDetector& detector, Real resolution)
{
    m_obstacles.clear();
    m_obstacles.reserve(detector.obstacles().size());

    // Convert World Polygons to Grid Coordinates (ImVec2) for rendering
    m_obstacles.resize(detector.obstacles().size());
    size_t counter = 0;
    for (const auto& poly : detector.obstacles()) {
        auto& gridPoly = m_obstacles[counter++];

        gridPoly.clear();
        for (size_t i = 0; i < poly.size(); ++i) {
            Point2 pt = poly.getVertex(i);

            // Transform World -> Grid Index (Float)
            // Assuming origin is (0,0). If OccupancyGrid has an origin offset,
            // you should subtract it here: (pt.x - origin.x) / resolution
            gridPoly.emplace_back(static_cast<float>(pt.x / resolution), static_cast<float>(pt.y / resolution));
        }
    }
}

void Map::draw(SDL_GPUCommandBuffer* commandBuffer)
{
    m_texture.upload(commandBuffer);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Map");

    const ImVec2 canvasPos  = ImGui::GetCursorScreenPos();
    const ImVec2 canvasSize = ImGui::GetContentRegionAvail();

    const float side = std::min(canvasSize.x, canvasSize.y);

    const ImVec2 pos(canvasPos.x + ((canvasSize.x - side) * 0.5F), canvasPos.y + ((canvasSize.y - side) * 0.5F));

    ImGui::SetCursorScreenPos(pos);
    ImGui::Image((ImTextureID)m_texture.texture(), ImVec2(side, side));

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawList->AddRect(pos, ImVec2(pos.x + side, pos.y + side), IM_COL32(255, 0, 255, 255), 0.0F, 0, 2.0F);

    const auto scale = side / float(m_texture.width());
    drawObstacles(drawList, pos, side, scale);
    drawRobot(drawList, pos, side, scale);

    // for (const auto& [p1, p2] : m_lines) {
    //     const ImVec2 ip1(pos.x + (side * 0.5F) + (p1.x * scale), pos.y + (side * 0.5F) + (p1.y * scale));
    //     const ImVec2 ip2(pos.x + (side * 0.5F) + (p2.x * scale), pos.y + (side * 0.5F) + (p2.y * scale));
    //     drawList->AddLine(ip1, ip2, IM_COL32(255, 128, 128, 255), 2.0F);
    // }

    ImGui::End();
    ImGui::PopStyleVar();
}

void Map::drawObstacles(ImDrawList* drawList, ImVec2 pos, float side, float scale)
{
    for (const auto& poly : m_obstacles) {
        if (poly.empty()) {
            continue;
        }

        // Transform cached Grid coordinates to Screen coordinates for this frame
        // We do this here so it responds to window resizing/movement correctly
        m_screenPoints.clear();
        m_screenPoints.reserve(poly.size());

        for (const auto& point : poly) {
            m_screenPoints.emplace_back(pos.x + (side * 0.5F) + (point.x * scale), pos.y + (side * 0.5F) + (point.y * scale));
        }

        // Draw Closed Polyline (Thick Yellow)
        drawList->AddPolyline(m_screenPoints.data(), static_cast<int>(m_screenPoints.size()), IM_COL32(255, 255, 0, 255), ImDrawFlags_Closed, 2.0F);

        // Draw Vertices (Red Dots) to visualize corners
        for (const auto& point : m_screenPoints) {
            drawList->AddCircleFilled(point, 3.0F, IM_COL32(255, 0, 0, 255)); // NOLINT
        }
    }
}

void Map::drawRobot(ImDrawList* drawList, ImVec2 pos, float side, float scale)
{
    if (m_bestEstimate) {
        const auto   diameter = float(m_robotSize) * scale;
        const auto   radius   = diameter * 0.5F;
        const ImVec2 center(pos.x + (side * 0.5F) + float(m_bestEstimate->x * scale), pos.y + (side * 0.5F) + float(m_bestEstimate->y * scale));

        drawList->AddCircleFilled(center, radius, IM_COL32(0, 255, 0, 255));  // NOLINT
        drawList->AddCircle(center, radius, IM_COL32(0, 0, 0, 255), 0, 1.5f); // NOLINT

        ImVec2 headingEnd(center.x + float(std::cos(m_bestEstimate->theta) * diameter), //
                          center.y + float(std::sin(m_bestEstimate->theta) * diameter));

        drawList->AddLine(center, headingEnd, IM_COL32(255, 0, 0, 255), 2.0F);
    }
}

} // namespace reanaut
