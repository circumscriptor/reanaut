#include "canvas.hpp"
#include "constants.hpp"
#include "kobuki.hpp"
#include "laser.hpp"
#include "manager.hpp"

#include <SDL3/SDL_gpu.h>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/system/detail/error_code.hpp>
#include <imgui.h>

// #include <algorithm>
#include <algorithm>
#include <csignal>
// #include <limits>
// #include <numbers>
#include <limits>
#include <print>
#include <random>

namespace reanaut
{

Manager::Manager(const Options& options)
    : m_signals(m_ioContext, SIGINT, SIGTERM),                                                  // Listen for Ctrl+C and termination signals
      m_timer(m_ioContext),                                                                     //
      m_cameraTimer(m_ioContext),                                                               //
      m_kobuki(m_ioContext, options.kobukiHostPort, options.robotIp, options.kobukiTargetPort), //
      m_laser(m_ioContext, options.laserHostPort, options.robotIp, options.laserTargetPort),    //
      m_navigator({.kP = kTranslateP, .kI = kTranslateI, .kD = kTranslateD}, {.kP = kRotateP, .kI = kRotateI, .kD = kRotateD}), //
      m_map(m_canvas.device()),                                                                                                 //
      m_filter(std::random_device()()),                                                                                         //
      m_camera(options.cameraUrl),                                                                                              //
      m_cameraTexture(m_canvas.device())
{
    waitForSignal();
    scheduleNextUpdate();
    scheduleNextCapture();

    m_tangentBug.setDestination(1, 0);

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

void Manager::scheduleNextCapture()
{
    m_cameraTimer.expires_after(kCaptureInterval);
    m_cameraTimer.async_wait([this](const boost::system::error_code& error) {
        if (error.value() == boost::asio::error::operation_aborted) {
            std::println("camera timer aborted");
            return;
        }
        updateCamera();
    });
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
        ++m_skippedFeedbackCounter;
        // scheduleNextUpdate();
        // return;
    }

    if (m_laser.getLatestSweep(m_scans)) {
        m_cloud.fromScans(m_bestEstimate, m_scans);

        m_occupancy.updateFromCloud(m_bestEstimate, m_cloud);
        m_image.update(m_occupancy);

        // m_filter.updateFromCloud(m_cloud, m_occupancy);
        m_filter.updateFromScans(m_scans, m_occupancy);
        m_filter.resample();

        m_map.update(m_occupancy, m_enableMapGradient);
        if (m_enableVisualizeElevation) {
            m_map.update(m_elevation);
        }
        m_map.update(m_filter, m_occupancy.resolution(), m_enableVisualizeFilter);

        if (m_enableVisualizeCloud) {
            m_map.update(m_cloud);
        }
        // _navigation.process_lidar_scans(_full_scan);

        m_detector.extractObstacles(m_image, m_detectorParams);
        if (m_enableVisualizeObstacles) {
            m_map.update(m_detector, m_image.resolution());
        }
    } else {
        ++m_skippedLaserScanCounter;
        // scheduleNextUpdate();
        // return;
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

    if (m_useManualNavigation) {
        auto [speed, radius] = m_velocity.computeControl();

        m_command.baseControl(speed, radius);

        if (m_command.size() > 0) {
            m_kobuki.asyncSend(m_command);
        }
    } else {
        auto [speed, radius] = m_tangentBug.process(m_scans, m_bestEstimate, m_time.getDeltaTime());
        std::println("[Manager] tangentbug returned: speed={}, radius={}", speed, radius);

        // Safety
        static bool s_informed       = false;
        const auto  shortestDistance = findShortestMeasurement(m_scans).distance;
        if (shortestDistance < 250 && shortestDistance > 0.0) { // NOLINT
            if (!s_informed) {
                std::println("\033[31m!!!SAFETY STOP!!!\033[0m");
                s_informed = true;
            }
            stopMotor();

        } else {
            s_informed = false;

            m_command.baseControl(speed, radius);

            if (m_command.size() > 0) {
                m_kobuki.asyncSend(m_command);
            }
        }
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
        }
        ImGui::End();

        if (ImGui::Begin("Feedback")) {
            const auto& sensors  = m_feedback.getBasicSensors();
            const auto& inertial = m_feedback.getInertial();
            ImGui::Text("Encoder: %d %d", sensors.leftEncoder, sensors.rightEncoder);
            ImGui::Text("Inertial: %d (%d)", inertial.angle, inertial.angleRate);
            ImGui::Text("Battery: %d", sensors.battery);
        }
        ImGui::End();

        if (ImGui::Begin("Debug")) {
            ImGui::Text("Detected obstacles: %zu", m_map.numObstacles());
            ImGui::Text("Skipped feedback: %zu", m_skippedFeedbackCounter);
            ImGui::Text("Skipped laser scan: %zu", m_skippedLaserScanCounter);
            ImGui::Checkbox("Map gradient", &m_enableMapGradient);
            ImGui::Checkbox("Visualize filter", &m_enableVisualizeFilter);
            ImGui::Checkbox("Visualize cloud", &m_enableVisualizeCloud);
            ImGui::Checkbox("Visualize obstacles", &m_enableVisualizeObstacles);
            ImGui::Checkbox("Visualize elevation", &m_enableVisualizeElevation);
            ImGui::Checkbox("Manual control", &m_useManualNavigation);
        }
        ImGui::End();

        if (ImGui::Begin("Obstacle Detector")) {
            static constexpr double kStep     = 0.01;
            static constexpr double kStepFast = 0.1;
            ImGui::InputDouble("Epsilon", &m_detectorParams.epsilon, kStep, kStepFast);
            ImGui::InputDouble("Minimum area", &m_detectorParams.minArea, kStep, kStepFast);

            m_detectorParams.epsilon = std::clamp(m_detectorParams.epsilon, kStep, std::numeric_limits<double>::max());
            m_detectorParams.minArea = std::clamp(m_detectorParams.minArea, kStep, std::numeric_limits<double>::max());
        }
        ImGui::End();

        m_map.draw(commandBuffer);
        m_cameraTexture.draw(commandBuffer);
    });
    stopMotor();
}

void Manager::updateCamera()
{
    const auto& capture = m_camera.capture();
    m_cameraTexture.update(capture);

    if (not capture.empty()) {
        m_depth.process(capture);
        m_elevation.update(m_depth, m_bestEstimate);
    }

    scheduleNextCapture();
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
