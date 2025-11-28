#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reanaut
{

Controller::Controller(const Gains& gains) : Controller(gains.kP, gains.kI, gains.kD) {}

Controller::Controller(Real kP, Real kI, Real kD)
    : m_integralMin(std::numeric_limits<Real>::min()), //
      m_integralMax(std::numeric_limits<Real>::max()), //
      m_rateLimit(std::numeric_limits<Real>::max()),   //
      m_kP(kP), m_kI(kI), m_kD(kD)
{
}

void Controller::reset()
{
    m_prevOutput = 0.0;
    m_prevError  = 0.0;
    m_integral   = 0.0;
    m_derivative = 0.0;
}

void Controller::setRange(Real range) { m_errorRange = range; }

void Controller::setRateLimit(Real limit) { m_rateLimit = limit; }

auto Controller::compute(Real target, Real measured, Real dt) -> Real { return limitRate(step(target - measured, dt), dt); }

// NOLINTNEXTLINE
auto Controller::computeInRange(Real target, Real measured, Real dt) -> Real { return limitRate(step(wrapError(target - measured), dt), dt); }

auto Controller::wrapError(Real error) const -> Real
{
    if (m_errorRange <= std::numeric_limits<Real>::epsilon()) {
        return error;
    }
    return std::remainder(error, m_errorRange);
}

// NOLINTNEXTLINE
auto Controller::limitRate(Real output, Real dt) -> Real
{
    const Real maxDelta = m_rateLimit * dt;

    Real delta = output - m_prevOutput;
    if (delta > maxDelta) {
        delta = maxDelta;
    } else if (delta < -maxDelta) {
        delta = -maxDelta;
    }

    const Real limited = m_prevOutput + delta;

    m_prevOutput = limited;
    return limited;
}

auto Controller::step(Real error, Real dt) -> Real
{
    m_integral   = std::clamp(m_integral + (error * dt), m_integralMin, m_integralMax);
    m_derivative = (error - m_prevError) / dt;
    m_prevError  = error;
    return (m_kP * error) + (m_kI * m_integral) + (m_kD * m_derivative);
}

} // namespace reanaut
