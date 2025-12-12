#include "manager.hpp"
#include "navigator.hpp"

#include <imgui.h>
#include <implot.h>

#include <cstdint>
#include <memory>
#include <print>
#include <string>

void printSampleVelocityControl()
{
    std::print("{:7}", ' ');
    for (auto angular : {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0}) {
        std::print("{:5}{:4}{:6}", ' ', angular, ' ');
    }
    std::println();

    for (double linear : {-3.0, -2.0, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 2.0, 3.0}) {
        std::print("{:5} | ", linear);
        for (double angular : {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0}) {
            auto [speed, radius] = reanaut::Velocity{.linear = linear, .angular = angular}.computeControl();
            std::print("{:5}, {:5} | ", int16_t(speed), int16_t(radius));
        }
        std::println();
    }
}

auto main(int argc, char** argv) -> int
{
    (void)argc;
    (void)argv;

    printSampleVelocityControl();

    static constexpr uint16_t kKobukiHostPort   = 53000;
    static constexpr uint16_t kKobukiTargetPort = 5300;
    static constexpr uint16_t kLaserHostPort    = 52999;
    static constexpr uint16_t kLaserTargetPort  = 5299;

    const std::string cameraUrl = "http://192.168.1.16:8889/";
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
