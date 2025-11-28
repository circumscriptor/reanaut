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
    double left{};
    double right{};
};

class Odometry
{
public:

    explicit Odometry(double tickToMeter = kTickToMeter) : m_tickToMeter(tickToMeter) {}

    [[nodiscard]] auto update(int32_t encoderLeft, int32_t encoderRight, double dt) -> std::optional<WheelVelocities>;

    void reset();

private:

    double m_tickToMeter;
    bool   m_initialized{};

    int32_t m_prevLeft{};
    int32_t m_prevRight{};
};

class Movement
{
public:

    using ControlType               = Control<double>;
    using FilterType                = Kalman::ExtendedKalmanFilter<State<double>>;
    using MeasurementType           = Measurement<double>;
    using MeasurementCovarianceType = Kalman::Covariance<Measurement<double>>;
    using MeasureType               = MeasurementModel<double>;
    using StateCovarianceType       = Kalman::Covariance<State<double>>;
    using StateType                 = State<double>;
    using SystemType                = SystemModel<double>;

    Movement();

    [[nodiscard]] auto getState() const -> const StateType&;

    void process(const Feedback& feedback, double dt);

    // NOLINTNEXTLINE(readability-identifier-length)
    void reset(double x, double y, double theta);

private:

    Odometry    m_odometry;
    SystemType  m_system;
    MeasureType m_measure;
    ControlType m_control;
    FilterType  m_filter;
};

} // namespace reanaut
