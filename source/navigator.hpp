#pragma once

#include "constants.hpp"
#include "controller.hpp"
#include "movement.hpp"

#include <cstdint>
#include <optional>
#include <utility>

namespace reanaut
{

struct Velocity
{
    using Real = RealType;

    Real linear{};
    Real angular{};

    [[nodiscard]] auto computeControl() const -> std::pair<uint16_t, uint16_t>;
};

class Navigator
{
public:

    using Real = RealType;

    static constexpr Real kDefaultDistanceTolerance = 0.05;
    static constexpr Real kDefaultAngleTolerance    = 0.1;
    static constexpr Real kDefaultMaxVelocity       = 0.5;
    static constexpr Real kDefaultMaxOmega          = 1.5;

    using StateType = Movement::StateType;

    Navigator(const Gains& translate, const Gains& rotate);

    [[nodiscard]] auto isActive() const noexcept -> bool;
    [[nodiscard]] auto targetReached() const noexcept -> bool;

    void setGoal(const Point2& goal);
    void updateGoal(const Point2& goal);
    void setTolerance(Real distanceTolerance, Real angleTolerance);
    void setMaxSpeeds(Real maxV, Real maxOmega);
    void setMaxAcceleration(Real linearAccelMax, Real angularAccelMax);
    void stop();

    [[nodiscard]] auto update(const StateType& current, Real dt) -> std::optional<Velocity>;
    [[nodiscard]] auto update(const Pose& pose, Real dt) -> std::optional<Velocity>;

private:

    Controller m_translate;
    Controller m_rotate;

    Real m_targetX{};
    Real m_targetY{};
    bool m_active{};
    bool m_targetReached{false};

    Real m_distanceTolerance{kDefaultDistanceTolerance};
    Real m_angleTolerance{kDefaultAngleTolerance};
    Real m_maxV{kDefaultMaxVelocity};  // m/s
    Real m_maxOmega{kDefaultMaxOmega}; // rad/s
};

} // namespace reanaut
