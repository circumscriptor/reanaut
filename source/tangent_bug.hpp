#pragma once

#include "constants.hpp"
#include "kalman.hpp"
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

    TangentBug();

    void setDestination(double destinationX, double destinationY)
    {
        m_destination = {.x = destinationX, .y = destinationY};
        m_state       = State::FollowDestination;
    };

    auto process(const std::vector<LaserScan>& scans, Particle position, RealType dt) -> std::pair<uint16_t, uint16_t>;

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

    double m_shortestDistanceToDest = std::numeric_limits<double>::max();
    bool   m_wallLocked             = false;

    Navigator m_pointFollower;

    [[nodiscard]] static auto lengthSq(double x, double y) noexcept -> double { return (x * x) + (y * y); }
    [[nodiscard]] static auto length(double x, double y) noexcept -> double { return std::sqrt(lengthSq(x, y)); }
    [[nodiscard]] static auto distanceSq(double x1, double y1, double x2, double y2) noexcept -> double { return lengthSq(x2 - x1, y2 - y1); }
    [[nodiscard]] static auto distance(Point2 A, Point2 B) noexcept -> double { return distance(A.x,A.y,B.x,B.y); }
    [[nodiscard]] static auto distance(double x1, double y1, double x2, double y2) noexcept -> double { return std::sqrt(distanceSq(x1, y1, x2, y2)); }

    [[nodiscard]] auto isPathToDestinationClear(const std::vector<LaserScan>& measurement) const -> bool;

    auto map(auto x, auto in_min, auto in_max, auto out_min, auto out_max) { return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

    auto angleToTarget(Point2 location) -> double;

}; // namespace reanaut
} // namespace reanaut
