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
    if (isZero(linear) and not isZero(angular)) {
        const auto speed = angular * kWheelbaseDistance * 1000.0 * 0.5;
        return {static_cast<uint32_t>(speed) & kMaskU16, 1};
    }

    if (not isZero(linear) and not isZero(angular)) {
        const auto speed = linear * 1000.0;
        return {static_cast<uint32_t>(speed) & kMaskU16, 0};
    }

    const auto radius = linear * 1000.0 / angular;
    const auto speed  = linear * 1000.0 * ((radius + std::copysign(kWheelbaseDistance * 1000.0, radius)) * 0.5) / radius;
    return {static_cast<uint32_t>(speed) & kMaskU16, static_cast<uint32_t>(radius) & kMaskU16};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Navigator::Navigator(const Gains& translate, const Gains& rotate) : m_translate(translate), m_rotate(rotate) { m_rotate.setRange(2.0 * std::numbers::pi); }

auto Navigator::isActive() const noexcept -> bool { return m_active; }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters,readability-identifier-length)
void Navigator::setGoal(Real x, Real y)
{
    m_targetX = x;
    m_targetY = y;
    m_active  = true;
    m_translate.reset();
    m_rotate.reset();
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters,readability-identifier-length)
void Navigator::updateGoal(Real x, Real y)
{
    m_targetX = x;
    m_targetY = y;
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
    if (not m_active) {
        return std::nullopt;
    }

    const Real dx       = m_targetX - current.x();
    const Real dy       = m_targetY - current.y();
    const Real distance = std::sqrt((dx * dx) + (dy * dy));

    if (distance < m_distanceTolerance) {
        m_active = false;
        return std::nullopt;
    }

    const Real heading      = std::atan2(dy, dx);
    const Real newOmega     = m_rotate.computeInRange(heading, current.theta(), dt);
    const Real headingError = m_rotate.getError();
    const Real projection   = std::max(std::cos(headingError), 0.0);
    const Real newV         = m_translate.compute(distance, 0.0, dt) * projection; // Projection Scaling

    return Velocity{
        .linear  = std::clamp(newV, -m_maxV, m_maxV),
        .angular = std::clamp(newOmega, -m_maxOmega, m_maxOmega),
    };
}

} // namespace reanaut
