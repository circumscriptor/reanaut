#include "cloud.hpp"
#include "grid.hpp"
#include "lines.hpp"
#include "occupancy.hpp"
#include "particle.hpp"
#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace reanaut
{

class Map : public GridBase
{
public:

    Map(SDL_GPUDevice* device);

    [[nodiscard]] auto numLines() const noexcept -> size_t { return m_lines.size(); }

    void update(const OccupancyGrid& occupancy, bool gradient = false);
    void update(const ParticleFilter& filter);
    void update(const PointCloud& cloud);
    void update(const TurboUberSuperDetector& detector, Real resolution);

    void draw(SDL_GPUCommandBuffer* commandBuffer);

private:

    TransferTexture                        m_texture;
    std::vector<std::pair<ImVec2, ImVec2>> m_lines;
};
} // namespace reanaut
