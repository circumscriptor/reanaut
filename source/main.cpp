#include "manager.hpp"

#include <imgui.h>
#include <implot.h>

#include <cstdint>
#include <memory>
#include <string>

auto main(int argc, char** argv) -> int
{
    (void)argc;
    (void)argv;

    static constexpr uint16_t kKobukiHostPort   = 53000;
    static constexpr uint16_t kKobukiTargetPort = 5300;
    static constexpr uint16_t kLaserHostPort    = 52999;
    static constexpr uint16_t kLaserTargetPort  = 5299;

    const std::string cameraUrl = "http://192.168.1.16:8080/video.mjpeg";
    const std::string robotIp   = "192.168.1.16";

    reanaut::Manager::Options opts;
    opts.cameraUrl        = cameraUrl;
    opts.kobukiHostPort   = kKobukiHostPort;
    opts.kobukiTargetPort = kKobukiTargetPort;
    opts.laserHostPort    = kLaserHostPort;
    opts.laserTargetPort  = kLaserTargetPort;
    opts.robotIp          = robotIp;

    std::make_unique<reanaut::Manager>(opts)->run();
    return 0;
}
