#include "constants.hpp"
#include "kobuki.hpp"
#include "movement.hpp"

#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>

namespace reanaut
{

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Odometry::Odometry(Real tickToMeter, Real tickToDegree) : m_tickToMeter(tickToMeter), m_tickToDegree(tickToDegree) {}

auto Odometry::process(const Feedback& feedback, Real dt) -> std::optional<OdometryFeedback>
{
    const auto& sensors  = feedback.getBasicSensors();
    const auto& inertial = feedback.getInertial();
    return update(sensors.leftEncoder, sensors.rightEncoder, inertial.angle, inertial.angleRate, dt);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto Odometry::update(int32_t encoderLeft, int32_t encoderRight, int32_t angle, int32_t angleRate, Real dt) -> std::optional<OdometryFeedback>
{
    if (dt <= std::numeric_limits<Real>::epsilon()) {
        return std::nullopt;
    }

    if (not m_initialized) {
        m_prevLeft    = encoderLeft;
        m_prevRight   = encoderRight;
        m_prevAngle   = angle;
        m_initialized = true;
        return std::nullopt;
    }

    const auto leftDiff  = static_cast<int16_t>(encoderLeft - m_prevLeft);
    const auto rightDiff = static_cast<int16_t>(encoderRight - m_prevRight);
    const auto angleDiff = static_cast<int16_t>(angle - m_prevAngle);
    m_prevLeft           = encoderLeft;
    m_prevRight          = encoderRight;
    m_prevAngle          = angle;

    const auto wheelLeft  = Real(leftDiff * m_tickToMeter) / dt;
    const auto wheelRight = Real(rightDiff * m_tickToMeter) / dt;

#if defined(SIMULATION)
    const auto inertialRate = (angleDiff * m_tickToDegree) / 180.0 * std::numbers::pi / dt;
#else
    (void)angleDiff;
    const auto inertialRate = (angleRate * m_tickToDegree) / 180.0 * std::numbers::pi;
#endif

    return OdometryFeedback{
        .wheelLeft  = wheelLeft,
        .wheelRight = wheelRight,
        .angular    = inertialRate,
        .linear     = (wheelLeft + wheelRight) / 2.0,
    };
}

void Odometry::reset()
{
    m_initialized = false;
    m_prevLeft    = 0;
    m_prevRight   = 0;
}

Movement::Movement() : m_odometry(kTickToMeter), m_measure(kWheelbaseDistance)
{
    StateType x0;
    x0.setZero();
    m_filter.init(x0);

    // NOLINTBEGIN(readability-identifier-naming,readability-identifier-length,readability-magic-numbers)
    StateCovarianceType Q;
    Q.setIdentity();
    Q(StateType::kX, StateType::kX)         = 0.001;
    Q(StateType::kY, StateType::kY)         = 0.001;
    Q(StateType::kTheta, StateType::kTheta) = 0.001;
    Q(StateType::kV, StateType::kV)         = 0.1;
    Q(StateType::kOmega, StateType::kOmega) = 0.1;
    m_system.setCovariance(Q);

    MeasurementCovarianceType R;
    R.setIdentity();
    R(MeasurementType::kVl, MeasurementType::kVl)     = 0.05;
    R(MeasurementType::kVr, MeasurementType::kVr)     = 0.05;
    R(MeasurementType::kGyro, MeasurementType::kGyro) = 0.01;
    m_measure.setCovariance(R);
    // NOLINTEND(readability-identifier-naming,readability-identifier-length,readability-magic-numbers)
}

auto Movement::getState() const -> const StateType& { return m_filter.getState(); }

void Movement::process(const Feedback& feedback, Real dt)
{
    const auto& sensors  = feedback.getBasicSensors();
    const auto& inertial = feedback.getInertial();

    auto wheels = m_odometry.update(sensors.leftEncoder, sensors.rightEncoder, inertial.angle, inertial.angleRate, dt);
    if (not wheels) {
        return;
    }

    m_control.dt() = dt;
    m_filter.predict(m_system, m_control);

    // NOLINTNEXTLINE(readability-identifier-length)
    MeasurementType z;
    z.vl()   = wheels->wheelLeft;
    z.vr()   = wheels->wheelRight;
    z.gyro() = inertial.angleRate;

    m_filter.update(m_measure, z);
}

// NOLINTNEXTLINE(readability-identifier-length,bugprone-easily-swappable-parameters)
void Movement::reset(Real x, Real y, Real theta)
{
    StateType state;
    state.setZero();
    state.x()     = x;
    state.y()     = y;
    state.theta() = theta;

    m_filter.init(state);
    m_odometry.reset();
}

} // namespace reanaut
