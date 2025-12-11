#include "constants.hpp"
#include "controller.hpp"
#include "navigator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <utility>

namespace reanaut
{

auto Velocity::computeControl() const -> std::pair<uint16_t, uint16_t>
{
    const bool hasLinear  = not isZero(linear);
    const bool hasAngular = not isZero(angular);

    if (not hasLinear and hasAngular) {
        const auto speed = angular * kWheelbaseDistance * 1000.0 * 0.5;
        return {static_cast<uint32_t>(speed) & kMaskU16, 1};
    }

    if (hasLinear and not hasAngular) {
        const auto speed = linear * 1000.0;
        return {static_cast<uint32_t>(speed) & kMaskU16, 0};
    }

    if (hasLinear and hasAngular) {
        const auto radius = linear * 1000.0 / angular;
        const auto speed  = linear * 1000.0 * ((radius + std::copysign(kWheelbaseDistance * 1000.0, radius)) * 0.5) / radius;
        return {static_cast<uint32_t>(speed) & kMaskU16, static_cast<uint32_t>(radius) & kMaskU16};
    }

    return {0, 0};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Navigator::Navigator(const Gains& translate, const Gains& rotate) : m_translate(translate), m_rotate(rotate) { m_rotate.setRange(2.0 * std::numbers::pi); }

auto Navigator::isActive() const noexcept -> bool { return m_active; }

void Navigator::setGoal(const Point2& goal)
{
    m_targetX = goal.x;
    m_targetY = goal.y;
    m_active  = true;
    m_translate.reset();
    m_rotate.reset();
}

void Navigator::updateGoal(const Point2& goal)
{
    m_targetX = goal.x;
    m_targetY = goal.y;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void Navigator::setTolerance(Real distanceTolerance, Real angleTolerance)
{
    m_distanceTolerance = distanceTolerance;
    m_angleTolerance    = angleTolerance;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void Navigator::setMaxSpeeds(Real maxV, Real maxOmega)
{
    m_maxV     = maxV;
    m_maxOmega = maxOmega;
}

void Navigator::setMaxAcceleration(Real linearAccelMax, Real angularAccelMax)
{
    m_translate.setRateLimit(linearAccelMax);
    m_rotate.setRateLimit(angularAccelMax);
}

void Navigator::stop()
{
    m_active = false;
    m_translate.reset();
    m_rotate.reset();
}

auto Navigator::update(const StateType& current, Real dt) -> std::optional<Velocity>
{
    return update(Pose({.x = current.x(), .y = current.y()}, current.theta()), dt);
}

auto Navigator::update(const Pose& pose, Real dt) -> std::optional<Velocity>
{
    if (not m_active) {
        return std::nullopt;
    }

    const Real dx       = m_targetX - pose.x;
    const Real dy       = m_targetY - pose.y;
    const Real distance = std::sqrt((dx * dx) + (dy * dy));

    if (distance < m_distanceTolerance) {
        m_active = false;
        return std::nullopt;
    }

    const Real heading      = std::atan2(dy, dx);
    const Real newOmega     = m_rotate.computeInRange(heading, pose.theta, dt);
    const Real headingError = m_rotate.getError();
    const Real projection   = std::max(std::cos(headingError), 0.0);
    const Real newV         = m_translate.compute(distance, 0.0, dt) * projection; // Projection Scaling

    return Velocity{
        .linear  = std::clamp(newV, -m_maxV, m_maxV),
        .angular = std::clamp(newOmega, -m_maxOmega, m_maxOmega),
    };
}

} // namespace reanaut
