#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace reanaut
{

Controller::Controller(double kP, double kI, double kD)
    : m_integralMin(std::numeric_limits<double>::min()), //
      m_integralMax(std::numeric_limits<double>::max()), //
      m_rateLimit(std::numeric_limits<double>::max()),   //
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

void Controller::setRange(double range) { m_errorRange = range; }

void Controller::setRateLimit(double limit) { m_rateLimit = limit; }

auto Controller::compute(double target, double measured, double dt) -> double { return limitRate(step(target - measured, dt), dt); }

// NOLINTNEXTLINE
auto Controller::computeInRange(double target, double measured, double dt) -> double { return limitRate(step(wrapError(target - measured), dt), dt); }

auto Controller::wrapError(double error) const -> double
{
    if (m_errorRange <= std::numeric_limits<double>::epsilon()) {
        return error;
    }
    return std::remainder(error, m_errorRange);
}

// NOLINTNEXTLINE
auto Controller::limitRate(double output, double dt) -> double
{
    const double maxDelta = m_rateLimit * dt;

    double delta = output - m_prevOutput;
    if (delta > maxDelta) {
        delta = maxDelta;
    } else if (delta < -maxDelta) {
        delta = -maxDelta;
    }

    const double limited = m_prevOutput + delta;

    m_prevOutput = limited;
    return limited;
}

auto Controller::step(double error, double dt) -> double
{
    m_integral   = std::clamp(m_integral + (error * dt), m_integralMin, m_integralMax);
    m_derivative = (error - m_prevError) / dt;
    m_prevError  = error;
    return (m_kP * error) + (m_kI * m_integral) + (m_kD * m_derivative);
}

} // namespace reanaut
