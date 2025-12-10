#include "occupancy.hpp"
#include "texture.hpp"

#include <SDL3/SDL_gpu.h>
#include <imgui.h>

namespace reanaut
{

class Map
{
public:

    Map(SDL_GPUDevice* device);

    void update(const OccupancyGrid& occupancy);

    void draw(SDL_GPUCommandBuffer* commandBuffer);

private:

    TransferTexture m_texture;
};
} // namespace reanaut
