#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace reanaut
{

static constexpr size_t kMaxLaserScans = 1000;

struct LaserScan
{
    uint32_t quality;
    float    angleDeg;
    float    distanceMm;
    uint32_t timestamp;
};

} // namespace reanaut
