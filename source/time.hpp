#pragma once

#include "constants.hpp"
#include "kobuki.hpp"

#include <cstdint>

namespace reanaut
{

class Time
{
public:

    using Real = RealType;

    [[nodiscard]] auto getDeltaTime() const noexcept -> Real;

    void process(const Feedback& feedback);

private:

    bool     m_initialized{};
    Real     m_deltaTime{};
    uint16_t m_prevTimestamp{};
};

} // namespace reanaut
