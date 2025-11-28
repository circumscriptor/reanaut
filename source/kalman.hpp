#pragma once

#include "constants.hpp"

#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/LinearizedSystemModel.hpp>
#include <kalman/Matrix.hpp>
#include <kalman/StandardBase.hpp>
#include <kalman/SystemModel.hpp>

#include <cstddef>

namespace reanaut
{

template <typename T>
class State : public Kalman::Vector<T, 3>
{
public:

    KALMAN_VECTOR(State, T, 3)

    static constexpr size_t kX     = 0; //! X-position
    static constexpr size_t kY     = 1; //! Y-Position
    static constexpr size_t kTheta = 2; //! Orientation
    static constexpr size_t kV     = 3; //!< Linear velocity
    static constexpr size_t kOmega = 4; //!< Angular velocity

    auto x() const -> T { return (*this)[kX]; }
    auto y() const -> T { return (*this)[kY]; }
    auto theta() const -> T { return (*this)[kTheta]; }
    auto v() const -> T { return (*this)[kV]; }
    auto omega() const -> T { return (*this)[kOmega]; }

    auto x() -> T& { return (*this)[kX]; }
    auto y() -> T& { return (*this)[kY]; }
    auto theta() -> T& { return (*this)[kTheta]; }
    auto v() -> T& { return (*this)[kV]; }
    auto omega() -> T& { return (*this)[kOmega]; }
};

template <typename T>
class Control : public Kalman::Vector<T, 1>
{
public:

    KALMAN_VECTOR(Control, T, 1)

    static constexpr size_t kDt = 0;

    auto dt() const -> T { return (*this)[kDt]; }
    auto dt() -> T& { return (*this)[kDt]; }
};

template <typename T, template <typename> typename CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:

    using S = State<T>;
    using C = Control<T>;

    // NOLINTNEXTLINE(readability-identifier-length)
    auto f(const S& x, const C& u) const -> S
    {
        S xp;
        xp.x()     = x.x() + x.v() * std::cos(x.theta()) * u.dt();
        xp.y()     = x.y() + x.v() * std::sin(x.theta()) * u.dt();
        xp.theta() = x.theta() + x.omega() * u.dt();
        xp.v()     = x.v();
        xp.omega() = x.omega();
        return xp;
    }

protected:

    // NOLINTNEXTLINE(readability-identifier-length)
    void updateJacobians(const S& x, const C& u)
    {
        this->F.setIdentity();
        this->F(S::kX, S::kTheta)     = -x.v() * std::sin(x.theta()) * u.dt(); // dx/dtheta = -v * sin(theta) * dt
        this->F(S::kX, S::kV)         = std::cos(x.theta()) * u.dt();          // dx/dv = cos(theta) * dt
        this->F(S::kY, S::kTheta)     = x.v() * std::cos(x.theta()) * u.dt();  // dy/dtheta = v * cos(theta) * dt
        this->F(S::kY, S::kV)         = std::sin(x.theta()) * u.dt();          // dy/dv = sin(theta) * dt
        this->F(S::kTheta, S::kOmega) = u.dt();                                // dtheta/domega = dt

        // W matrix (Process Noise Jacobian) - Simplified identity
        this->W.setIdentity();
    }
};

//
// Measurement
//

template <typename T>
class Measurement : public Kalman::Vector<T, 3>
{
public:

    KALMAN_VECTOR(Measurement, T, 3)

    static constexpr size_t kVl   = 0; // Left Wheel Linear Velocity
    static constexpr size_t kVr   = 1; // Right Wheel Linear Velocity
    static constexpr size_t kGyro = 2; // Gyro Angular Rate

    auto vl() const -> T { return (*this)[kVl]; }
    auto vr() const -> T { return (*this)[kVr]; }
    auto gyro() const -> T { return (*this)[kGyro]; }

    auto vl() -> T& { return (*this)[kVl]; }
    auto vr() -> T& { return (*this)[kVr]; }
    auto gyro() -> T& { return (*this)[kGyro]; }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:

    using S = State<T>;
    using M = Measurement<T>;

    T wheelBase; // Distance between wheels (meters)

    MeasurementModel(T wheelBase = kWheelbaseDistance) : wheelBase(wheelBase)
    {
        // Setup fixed Jacobians (since the measurement model is linear wrt state)
        this->H.setZero();
        this->V.setIdentity();

        // VL = v - (L/2)*omega
        this->H(M::kVl, S::kV)     = 1.0;
        this->H(M::kVl, S::kOmega) = -wheelBase / 2.0;

        // VR = v + (L/2)*omega
        this->H(M::kVr, S::kV)     = 1.0;
        this->H(M::kVr, S::kOmega) = wheelBase / 2.0;

        // Gyro = omega
        this->H(M::kGyro, S::kOmega) = 1.0;
    }

    // NOLINTNEXTLINE(readability-identifier-length)
    auto h(const S& x) const -> M
    {
        M measurement;

        // Calculate expected measurements from state
        measurement.vl()   = x.v() - (wheelBase / 2.0) * x.omega();
        measurement.vr()   = x.v() + (wheelBase / 2.0) * x.omega();
        measurement.gyro() = x.omega();

        return measurement;
    }

protected:

    // NOLINTNEXTLINE(readability-identifier-length)
    void updateJacobians(const S& x)
    {
        // H is already set in constructor and is constant for this model.
        // If wheelBase changed dynamically, we would update it here.
        (void)x;
    }
};

} // namespace reanaut
