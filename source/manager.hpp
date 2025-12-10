#pragma once

#include "canvas.hpp"
#include "connections.hpp"
#include "kobuki.hpp"
#include "laser.hpp"
#include "map.hpp"
#include "movement.hpp"
#include "navigator.hpp"
#include "time.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/steady_timer.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace reanaut
{

class Manager
{
public:

    static constexpr std::chrono::milliseconds kLoopInterval{1000 / 40}; // 40 Hz

    struct Options
    {
        std::string cameraUrl;
        std::string robotIp;
        uint16_t    kobukiHostPort{};
        uint16_t    kobukiTargetPort{};
        uint16_t    laserHostPort{};
        uint16_t    laserTargetPort{};
    };

    explicit Manager(const Options& options);

    void run();

    void pause();
    void resume();
    void stopMotor();

    [[nodiscard]] auto isPaused() const noexcept -> bool { return m_isPaused; }

protected:

    void waitForSignal();
    void scheduleNextUpdate();
    void update();

private:

    boost::asio::io_context   m_ioContext;
    boost::asio::signal_set   m_signals;
    boost::asio::steady_timer m_timer;
    std::atomic<bool>         m_isPaused{true};
    std::vector<LaserScan>    m_scans;

    Canvas           m_canvas;
    KobukiConnection m_kobuki;
    LaserConnection  m_laser;
    Feedback         m_feedback;
    Command          m_command;
    Time             m_time;
    Movement         m_movement;
    Navigator        m_navigator;
    Map              m_map;
    // camera _camera_receiver;
};
} // namespace reanaut
