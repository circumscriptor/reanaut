#pragma once

#include "constants.hpp"
#include "laser.hpp"
#include "navigator.hpp"
#include "particle.hpp"
#include "polygon.hpp"

#include <cstdint>
#include <limits>
#include <span>
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

    auto process(const std::vector<LaserScan>& scans, Particle position, RealType dt, std::span<const Polygon> wallPolygons) -> std::pair<uint16_t, uint16_t>;

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

    // Use `Point2` member functions (`operator-`, `length`, `lengthSq`, `distance`) instead
    // of duplicating vector math helpers here.

    [[nodiscard]] auto isPathToDestinationClear(const std::vector<LaserScan>& measurement) const -> bool;
    auto               decideFollowDirection(const std::vector<LaserScan>& measurement, std::span<const Polygon> wallPolygons, Point2 robotPos) -> State;

    auto map(auto value, auto inMin, auto inMax, auto outMin, auto outMax) { return ((value - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin; }

    [[nodiscard]] auto angleToTarget(Point2 location) const -> Real;

}; // namespace reanaut
} // namespace reanaut
