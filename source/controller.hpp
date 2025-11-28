#pragma once

namespace reanaut
{

class Controller
{
public:

    Controller(double kP, double kI, double kD);

    void reset();

    [[nodiscard]] auto getError() const -> double { return m_prevError; }
    [[nodiscard]] auto getIntegral() const -> double { return m_integral; }
    [[nodiscard]] auto getDerivative() const -> double { return m_derivative; }

    /// Set radial range [-range/2,range/2]
    void setRange(double range);
    void setRateLimit(double limit);

    auto compute(double target, double measured, double dt) -> double;
    auto computeInRange(double target, double measured, double dt) -> double;

protected:

    [[nodiscard]] auto wrapError(double error) const -> double;
    [[nodiscard]] auto limitRate(double output, double dt) -> double;

    auto step(double error, double dt) -> double;

private:

    double m_prevOutput{};
    double m_prevError{};
    double m_integral{};
    double m_derivative{};

    double m_errorRange{};
    double m_integralMin;
    double m_integralMax;
    double m_rateLimit;

    double m_kP;
    double m_kI;
    double m_kD;
};

} // namespace reanaut
