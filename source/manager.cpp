#include "canvas.hpp"
#include "constants.hpp"
#include "kobuki.hpp"
#include "manager.hpp"

#include <SDL3/SDL_gpu.h>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/detail/error_code.hpp>
#include <imgui.h>

#include <cmath>
#include <csignal>
#include <print>
#include <random>

namespace reanaut
{

Manager::Manager(const Options& options)
    : m_signals(m_ioContext, SIGINT, SIGTERM), // Listen for Ctrl+C and termination signals
      m_timer(m_ioContext), m_kobuki(m_ioContext, options.kobukiHostPort, options.robotIp, options.kobukiTargetPort),
      m_laser(m_ioContext, options.laserHostPort, options.robotIp, options.laserTargetPort),                                    //
      m_navigator({.kP = kTranslateP, .kI = kTranslateI, .kD = kTranslateD}, {.kP = kRotateP, .kI = kRotateI, .kD = kRotateD}), // TODO
      m_map(m_canvas.device()),                                                                                                 //
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
            m_bestEstimate = m_filter.getBestEstimate();
        }
    } else {
        std::println("SKIPPED FEEDBACK, VERY BAD!");
        scheduleNextUpdate();
        return;
    }

    if (m_laser.getLatestSweep(m_scans)) {
        m_cloud.fromScans(m_bestEstimate, m_scans);
        m_occupancy.updateFromCloud(m_bestEstimate, m_cloud);
        m_filter.updateFromScans(m_scans, m_occupancy);
        // m_filter.updateFromCloud(m_cloud, m_occupancy);
        m_filter.resample();
        m_map.update(m_occupancy);
        m_map.update(m_filter);
        m_map.update(m_cloud);
        // _navigation.process_lidar_scans(_full_scan);
    } else {
        std::println("SKIPPED LASER SWEEP, SOMEWHAT BAD!");
        scheduleNextUpdate();
        return;
    }

    // const Point2 goal{
    //     .x = m_bestEstimate.x + (m_velocity.linear * std::cos(m_velocity.angular)),
    //     .y = m_bestEstimate.y + (m_velocity.linear * std::sin(m_velocity.angular)),
    // };

    // if (not m_navigator.isActive()) {
    //     m_navigator.setGoal(goal);
    // } else {
    //     m_navigator.updateGoal(goal);
    // }

    // auto velocity = m_navigator.update(m_bestEstimate, m_time.getDeltaTime());
    // if (velocity) {
    //     auto [speed, radius] = velocity->computeControl();
    //     m_command.baseControl(speed, radius);
    // } else {
    //     m_command.baseControl(0, 0);
    // }

    auto [speed, radius] = m_velocity.computeControl();
    m_command.baseControl(speed, radius);

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

        processKeyboard();

        if (ImGui::Begin("Pose")) {
            ImGui::Text("Position: %f %f", m_bestEstimate.x, m_bestEstimate.y);
            ImGui::Text("Rotation: %f", m_bestEstimate.theta);
            ImGui::End();
        }

        if (ImGui::Begin("Feedback")) {
            const auto& sensors  = m_feedback.getBasicSensors();
            const auto& inertial = m_feedback.getInertial();
            ImGui::Text("Encoder: %d %d", sensors.leftEncoder, sensors.rightEncoder);
            ImGui::Text("Inertial: %d (%d)", inertial.angle, inertial.angleRate);
            ImGui::Text("Battery: %d", sensors.battery);
            ImGui::End();
        }

        m_map.draw(commandBuffer);
    });
    stopMotor();
}

void Manager::processKeyboard()
{
    const double kManualLinearSpeed  = 0.3; // m/s
    const double kManualAngularSpeed = 1.2; // rad/s

    // Check Linear (Forward/Backward)
    if (ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_UpArrow)) {
        m_velocity.linear = kManualLinearSpeed;
    } else if (ImGui::IsKeyDown(ImGuiKey_S) || ImGui::IsKeyDown(ImGuiKey_DownArrow)) {
        m_velocity.linear = -kManualLinearSpeed;
    } else {
        m_velocity.linear = 0.0;
    }

    // Check Angular (Left/Right)
    if (ImGui::IsKeyDown(ImGuiKey_A) || ImGui::IsKeyDown(ImGuiKey_LeftArrow)) {
        m_velocity.angular = kManualAngularSpeed;
    } else if (ImGui::IsKeyDown(ImGuiKey_D) || ImGui::IsKeyDown(ImGuiKey_RightArrow)) {
        m_velocity.angular = -kManualAngularSpeed;
    } else {
        m_velocity.angular = 0.0;
    }
}

} // namespace reanaut
