#include "constants.hpp"
#include "kobuki.hpp"
#include "time.hpp"

namespace reanaut
{

auto Time::getDeltaTime() const noexcept -> Real { return m_deltaTime; }

void Time::process(const Feedback& feedback)
{
    const auto& sensors = feedback.getBasicSensors();

    if (!m_initialized) {
        m_prevTimestamp = sensors.timestamp;
        m_initialized   = true;
    }

    // Time
#if defined(SIMULATOR)
    int delta_time = 20; // For simulator
#else
    int deltaTime = static_cast<int>(sensors.timestamp) - m_prevTimestamp;
    if (deltaTime < 0) {
        deltaTime += (kMaskU16 + 1);
    }
    m_prevTimestamp = sensors.timestamp;
#endif
    m_deltaTime = static_cast<Real>(deltaTime) * kMillisecondToSecond;
}

} // namespace reanaut
