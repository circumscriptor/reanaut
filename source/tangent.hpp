#pragma once

#include "constants.hpp"
#include "occupancy.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace reanaut
{

struct Discontinuity
{
    using Real = RealType;

    Real angle;
    Real distance;
    enum Type : uint8_t
    {
        LeftEdge,
        RightEdge,
        MinLocal
    } type;
};

class TangentBug
{
public:

    using Real = RealType;

    [[nodiscard]] auto discontinuities() const -> std::span<const Discontinuity> { return m_discontinuities; }

    void findDiscontinuities(std::span<VirtualReading> scan, Real jumpThreshold = 0.5);

private:

    std::vector<Discontinuity> m_discontinuities;
};

} // namespace reanaut
