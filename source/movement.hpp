#pragma once

#include "constants.hpp"
#include "kalman.hpp"
#include "kobuki.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/Types.hpp>

#include <cstdint>
#include <optional>

namespace reanaut
{

struct OdometryFeedback
{
    using Real = RealType;

    Real wheelLeft{};
    Real wheelRight{};
    Real angular{};
    Real linear{};
};

class Odometry
{
public:

    using Real = RealType;

    explicit Odometry(Real tickToMeter = kTickToMeter, Real tickToDegree = kTickToDegree);

    [[nodiscard]] auto process(const Feedback& feedback, Real dt) -> std::optional<OdometryFeedback>;
    [[nodiscard]] auto update(int32_t encoderLeft, int32_t encoderRight, int32_t angle, int32_t angleRate, Real dt) -> std::optional<OdometryFeedback>;

    void reset();

private:

    Real m_tickToMeter;
    Real m_tickToDegree;
    bool m_initialized{};

    int32_t m_prevLeft{};
    int32_t m_prevRight{};
    int32_t m_prevAngle{};
};

class Movement
{
public:

    using Real = RealType;

    using ControlType               = Control<Real>;
    using FilterType                = Kalman::ExtendedKalmanFilter<State<Real>>;
    using MeasurementType           = Measurement<Real>;
    using MeasurementCovarianceType = Kalman::Covariance<Measurement<Real>>;
    using MeasureType               = MeasurementModel<Real>;
    using StateCovarianceType       = Kalman::Covariance<State<Real>>;
    using StateType                 = State<Real>;
    using SystemType                = SystemModel<Real>;

    Movement();

    [[nodiscard]] auto getState() const -> const StateType&;

    void process(const Feedback& feedback, Real dt);

    // NOLINTNEXTLINE(readability-identifier-length)
    void reset(Real x, Real y, Real theta);

private:

    Odometry    m_odometry;
    SystemType  m_system;
    MeasureType m_measure;
    ControlType m_control;
    FilterType  m_filter;
};

} // namespace reanaut
