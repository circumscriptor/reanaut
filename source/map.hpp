#include "cloud.hpp"
#include "grid.hpp"
#include "occupancy.hpp"
#include "particle.hpp"
#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

namespace reanaut
{

class Map : public GridBase
{
public:

    Map(SDL_GPUDevice* device);

    void update(const OccupancyGrid& occupancy);
    void update(const ParticleFilter& filter);
    void update(const PointCloud& cloud);

    void draw(SDL_GPUCommandBuffer* commandBuffer);

private:

    TransferTexture m_texture;
};
} // namespace reanaut
