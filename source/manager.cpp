#include "canvas.hpp"
#include "kobuki.hpp"
#include "manager.hpp"

#include <SDL3/SDL_gpu.h>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/detail/error_code.hpp>
#include <imgui.h>

#include <csignal>
#include <print>
#include <random>

namespace reanaut
{

Manager::Manager(const Options& options)
    : m_signals(m_ioContext, SIGINT, SIGTERM), // Listen for Ctrl+C and termination signals
      m_timer(m_ioContext), m_kobuki(m_ioContext, options.kobukiHostPort, options.robotIp, options.kobukiTargetPort),
      m_laser(m_ioContext, options.laserHostPort, options.robotIp, options.laserTargetPort), //
      m_navigator({}, {}),                                                                   // TODO
      m_map(m_canvas.device()),                                                              //
      m_filter(std::random_device()())
{
    waitForSignal();
    scheduleNextUpdate();

    m_kobuki.asyncRecv();
    m_laser.asyncRecv();

    m_filter.init({});
    // _movement.set_target(DEFAULT_NAVIGATE_TO_X, DEFAULT_NAVIGATE_TO_Y);
}

void Manager::waitForSignal()
{
    m_signals.async_wait([this](const boost::system::error_code& error, int signum) {
        if (error) {
            return; // Error or operation aborted
        }

        std::println("\nSignal {} received. Shutting down.", signum);
        m_canvas.close();
    });
}

void Manager::scheduleNextUpdate()
{
    m_timer.expires_after(kLoopInterval);
    m_timer.async_wait([this](const boost::system::error_code& error) {
        if (error.value() == boost::asio::error::operation_aborted) {
            m_canvas.close();
            std::println("update timer aborted");
            return; // cancelled, stop
        }
        update();
    });

    // _timer.cancel();
}

void Manager::pause()
{
    m_isPaused = true;
    stopMotor();
}

void Manager::resume() { m_isPaused = false; }

void Manager::stopMotor()
{
    // Send immediate stop command with zero speed and radius
    m_command.reset();
    m_command.baseControl(0, 0); // 0 mm/s speed, 0 mm radius
    m_kobuki.asyncSend(m_command);
    std::println("Stop command sent to robot.");
}

void Manager::update()
{
    if (m_isPaused) {
        scheduleNextUpdate();
        return;
    }

    m_command.reset();

    if (m_kobuki.getLatestFeedback(m_feedback)) {
        m_time.process(m_feedback);
        // m_movement.process(m_feedback, m_time.getDeltaTime());

        auto result = m_odometry.process(m_feedback, m_time.getDeltaTime());
        if (result) {
            m_filter.prediction(result->linear, result->angular, m_time.getDeltaTime());
        }
    }

    if (m_laser.getLatestSweep(m_scans)) {
        // _navigation.process_lidar_scans(_full_scan);
        m_occupancy.updateFromScans({}, m_scans);
        m_filter.updateWeights(m_scans, m_occupancy);
        m_filter.resample();
        m_map.update(m_occupancy);
        m_map.update(m_filter);
    }

    // if (m_kobuki.getLatestFeedback(m_feedback)) {
    //     // auto velocity = m_navigator.updconst OccupancyGrid &mapate(m_movement.getState(), m_time.getDeltaTime());
    //     // if (velocity) {
    //     //     auto [speed, radius] = velocity->computeControl();
    //     //     m_command.baseControl(speed, radius);
    //     // } else {
    //     //     m_command.baseControl(0, 0);
    //     // }

    //     // const auto current_x_m       = static_cast<float>(_movement.get_x());
    //     // const auto current_y_m       = static_cast<float>(_movement.get_y());
    //     // const auto current_theta_rad = static_cast<float>(_movement.get_angle());

    //     // // --- 3. Determine control strategy based on navigation state ---
    //     // const auto nav_state = _navigation.get_navigation_state();

    //     // // Use navigation command for wall-following behavior (tangent bug algorithm)
    //     // const auto motion_command = _navigation.get_navigation_command(current_x_m, current_y_m, current_theta_rad);

    //     // if (nav_state == navigation_state::MOTION_TO_GOAL) {
    //     //     // Check if goal has changed (e.g., user clicked new goal on map)
    //     //     if (_navigation.has_goal_changed()) {
    //     //         // New goal set - initialize movement targeting
    //     //         _movement.set_target(_navigation.get_current_goal_x(), _navigation.get_current_goal_y());
    //     //         std::println("Goal changed - calling set_target()\n";
    //     //     }
    //     //     // Use approach_target for smooth path to goal, but also respect navigation constraints
    //     //     _movement.approach_target(_time.get_delta_time_s());
    //     // } else if (nav_state == navigation_state::BOUNDARY_FOLLOWING) {
    //     //     // Apply navigation commands from wall follower
    //     //     _movement.set_forward_speed(motion_command.linear_velocity_m_s);
    //     //     _movement.set_rotation_speed(motion_command.angular_velocity_rad_s);
    //     // }
    // }

    if (m_command.size() > 0) {
        m_kobuki.asyncSend(m_command);
    }

    scheduleNextUpdate();
}

void Manager::run()
{
    m_canvas.run([this](SDL_GPUCommandBuffer* commandBuffer) {
        m_ioContext.poll();
        m_map.draw(commandBuffer);
    });
    stopMotor();
}

} // namespace reanaut
