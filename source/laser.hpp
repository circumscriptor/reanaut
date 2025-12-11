#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace reanaut
{

static constexpr size_t kMaxLaserScans = 1000;

struct LaserScan
{
    uint32_t quality;   //!< ?
    float    angle;     //!< 0-360 [deg]
    float    distance;  //!< 0=out-of-range [mm]
    uint32_t timestamp; //!< ?
};

} // namespace reanaut
