#pragma once

#include "constants.hpp"

namespace reanaut
{

struct Gains
{
    using Real = RealType;

    Real kP{};
    Real kI{};
    Real kD{};
};

class Controller
{
public:

    using Real = RealType;

    explicit Controller(const Gains& gains);
    Controller(Real kP, Real kI, Real kD);

    void reset();

    [[nodiscard]] auto getError() const -> Real { return m_prevError; }
    [[nodiscard]] auto getIntegral() const -> Real { return m_integral; }
    [[nodiscard]] auto getDerivative() const -> Real { return m_derivative; }

    /// Set radial range [-range/2,range/2]
    void setRange(Real range);
    void setRateLimit(Real limit);

    auto compute(Real target, Real measured, Real dt) -> Real;
    auto computeInRange(Real target, Real measured, Real dt) -> Real;

protected:

    [[nodiscard]] auto wrapError(Real error) const -> Real;
    [[nodiscard]] auto limitRate(Real output, Real dt) -> Real;

    auto step(Real error, Real dt) -> Real;

private:

    Real m_prevOutput{};
    Real m_prevError{};
    Real m_integral{};
    Real m_derivative{};

    Real m_errorRange{};
    Real m_integralMin;
    Real m_integralMax;
    Real m_rateLimit;

    Real m_kP;
    Real m_kI;
    Real m_kD;
};

} // namespace reanaut
