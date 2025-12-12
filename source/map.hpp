#include "cloud.hpp"
#include "constants.hpp"
#include "elevation.hpp"
#include "grid.hpp"
#include "lines.hpp"
#include "occupancy.hpp"
#include "particle.hpp"
#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace reanaut
{

class Map : public GridBase
{
public:

    Map(SDL_GPUDevice* device);

    [[nodiscard]] auto numObstacles() const noexcept -> size_t { return m_obstacles.size(); }

    void update(const OccupancyGrid& occupancy, bool gradient = false);
    void update(const ElevationGrid& elevation);
    void update(const ParticleFilter& filter, Real resolution, bool enableFilter);
    void update(const PointCloud& cloud);
    void update(const TurboUberSuperDetector& detector, Real resolution);

    void draw(SDL_GPUCommandBuffer* commandBuffer);

protected:

    void drawObstacles(ImDrawList* drawList, ImVec2 pos, float side, float scale);
    void drawRobot(ImDrawList* drawList, ImVec2 pos, float side, float scale);

private:

    TransferTextureStorage m_texture;
    // std::vector<std::pair<ImVec2, ImVec2>> m_lines;
    std::vector<std::vector<ImVec2>> m_obstacles;
    std::vector<ImVec2>              m_screenPoints;
    std::optional<Pose>              m_bestEstimate;
    Real                             m_robotSize{};
};
} // namespace reanaut
