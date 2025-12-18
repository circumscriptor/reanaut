#pragma once

#include "constants.hpp"
#include "laser.hpp"
#include "navigator.hpp"
#include "particle.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace reanaut
{

class TangentBug
{
public:

    using Real = RealType;

    TangentBug();

    void setDestination(Real destinationX, Real destinationY)
    {
        m_destination    = {.x = destinationX, .y = destinationY};
        m_state          = State::FollowDestination;
        m_isDoneDoneDone = false;
    };

    auto process(const std::vector<LaserScan>& scans, Particle position, RealType dt) -> std::pair<uint16_t, uint16_t>;

    [[nodiscard]] auto isDoneDoneDone() const -> bool { return m_isDoneDoneDone; }

private:

    enum class State : uint8_t
    {
        FollowDestination,
        DecideWallFollow,
        FollowWallR,
        FollowWallL,
        DoneDoneDone,
        Invalid,
    };

    State m_state = State::Invalid;

    Point2 m_destination, m_locationBeforeDiversion;

    Real m_shortestDistanceToDest = std::numeric_limits<Real>::max();
    bool m_wallLocked             = false;

    bool m_isDoneDoneDone = false;

    Navigator m_pointFollower;

    [[nodiscard]] static auto lengthSq(Real x, Real y) noexcept -> Real { return (x * x) + (y * y); }
    [[nodiscard]] static auto length(Real x, Real y) noexcept -> Real { return std::sqrt(lengthSq(x, y)); }
    [[nodiscard]] static auto distanceSq(Real x1, Real y1, Real x2, Real y2) noexcept -> Real { return lengthSq(x2 - x1, y2 - y1); }
    [[nodiscard]] static auto distance(Point2 A, Point2 B) noexcept -> Real { return distance(A.x, A.y, B.x, B.y); }
    [[nodiscard]] static auto distance(Real x1, Real y1, Real x2, Real y2) noexcept -> Real { return std::sqrt(distanceSq(x1, y1, x2, y2)); }

    [[nodiscard]] auto isPathToDestinationClear(const std::vector<LaserScan>& measurement) const -> bool;

    auto map(auto value, auto inMin, auto inMax, auto outMin, auto outMax) { return ((value - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin; }

    [[nodiscard]] auto angleToTarget(Point2 location) const -> Real;

}; // namespace reanaut
} // namespace reanaut
