#pragma once

#include "camera.hpp"
#include "canvas.hpp"
#include "cloud.hpp"
#include "connections.hpp"
#include "constants.hpp"
#include "depth.hpp"
#include "elevation.hpp"
#include "kobuki.hpp"
#include "laser.hpp"
#include "lines.hpp"
#include "map.hpp"
#include "movement.hpp"
#include "navigator.hpp"
#include "occupancy.hpp"
#include "particle.hpp"
#include "tangent_bug.hpp"
#include "time.hpp"
#include "traversability.hpp"
#include "wavefront.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/asio/steady_timer.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace reanaut
{

class Manager
{
public:

    static constexpr std::chrono::milliseconds kLoopInterval{1000 / 40};    // 40 Hz
    static constexpr std::chrono::milliseconds kCaptureInterval{1000 / 10}; // 10 Hz

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
    void scheduleNextCapture();
    void update();
    void updateCamera();
    void processKeyboard();

private:

    boost::asio::io_context   m_ioContext;
    boost::asio::signal_set   m_signals;
    boost::asio::steady_timer m_timer;
    boost::asio::steady_timer m_cameraTimer;
    std::atomic<bool>         m_isPaused{false};
    std::vector<LaserScan>    m_scans;
    Velocity                  m_velocity;
    size_t                    m_skippedFeedbackCounter{};
    size_t                    m_skippedLaserScanCounter{};
    bool                      m_enableMapGradient{true};
    bool                      m_enableVisualizeFilter{};
    bool                      m_enableVisualizeCloud{true};
    bool                      m_enableVisualizeObstacles{true};
    bool                      m_enableVisualizeElevation{true};
    bool                      m_enableVisualizeTraversability{true};
    bool                      m_enableVisualizeWavefront{true};
    bool                      m_returnToHome{false};
    bool                      m_updatePath{true};
    Point2                    m_start;
    Point2                    m_target;

    Canvas                         m_canvas;
    KobukiConnection               m_kobuki;
    LaserConnection                m_laser;
    Feedback                       m_feedback;
    Command                        m_command;
    Time                           m_time;
    Navigator                      m_navigator;
    OccupancyGrid                  m_occupancy;
    Map                            m_map;
    Odometry                       m_odometry;
    ParticleFilter                 m_filter;
    Particle                       m_bestEstimate;
    PointCloud                     m_cloud;
    GridImage                      m_image;
    TurboUberSuperDetector         m_detector;
    TurboUberSuperDetector::Params m_detectorParams;
    TangentBug                     m_tangentBug;
    Camera                         m_camera;
    CameraTexture                  m_cameraTexture;
    DepthProcessor                 m_depth;
    ElevationGrid                  m_elevation;
    TraversabilityGrid             m_traversability;
    WavefrontPlanner               m_wavefront;
};

} // namespace reanaut
