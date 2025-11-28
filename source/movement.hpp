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

struct WheelVelocities
{
    using Real = RealType;

    Real left{};
    Real right{};
};

class Odometry
{
public:

    using Real = RealType;

    explicit Odometry(Real tickToMeter = kTickToMeter) : m_tickToMeter(tickToMeter) {}

    [[nodiscard]] auto update(int32_t encoderLeft, int32_t encoderRight, Real dt) -> std::optional<WheelVelocities>;

    void reset();

private:

    Real m_tickToMeter;
    bool m_initialized{};

    int32_t m_prevLeft{};
    int32_t m_prevRight{};
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
