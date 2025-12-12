#include "constants.hpp"
#include "laser.hpp"
#include "navigator.hpp"
#include "particle.hpp"
#include "tangent_bug.hpp"

#include <cstdint>
#include <optional>
#include <print>
#include <utility>
#include <vector>

#define robotStop           \
    robotSpeed.angular = 0; \
    robotSpeed.linear  = 0

namespace reanaut
{
TangentBug::TangentBug() : m_pointFollower({.kP = kTranslateP, .kI = kTranslateI, .kD = kTranslateD}, {.kP = kRotateP, .kI = kRotateI, .kD = kRotateD}) {};

auto TangentBug::process(const std::vector<LaserScan>& scans, Particle robotPosition, RealType dt) -> std::pair<uint16_t, uint16_t>
{
    Velocity robotSpeed;

    auto currentDistToDest = distance(m_destination.x, m_destination.y, robotPosition.x, robotPosition.y);

    switch (m_state) {
        case State::FollowDestination: {

            std::println("Is active {}", m_pointFollower.isActive());
            if (not m_pointFollower.isActive()) {
                m_pointFollower.setGoal(m_destination);
                std::println("STATE: FollowDestination - Setting target.");
            }

            const auto shortestDistance = findShortestMeasurementInRange(scans, -45.0, 45.0).distance;
            std::println("shortest scan distance: {:.2f}", shortestDistance);

            if (shortestDistance < kObstacleDetectDistanceMm && shortestDistance > 0.0) {
                std::println("STATE: FollowDestination -> Obstacle detected! Switching to Decision state.");
                robotStop;
                m_locationBeforeDiversion.x = robotPosition.x;
                m_locationBeforeDiversion.y = robotPosition.y;
                m_shortestDistanceToDest    = distance(m_destination.x, m_destination.y, m_locationBeforeDiversion.x, m_locationBeforeDiversion.y);
                m_state                     = State::DecideWallFollow;
            } else {
                std::println("APPROACHING TARGET");
                auto temp = m_pointFollower.update(robotPosition, dt);
                if (temp) {
                    robotSpeed = *temp;
                } else {
                    std::println("Navigator update returned invalid data");
                }
            }

            // if (m_pointFollower.isActive()) {
            //     robotStop;
            //     m_state = State::DoneDoneDone;
            // }

            break;
        }

        case State::DecideWallFollow: {
            // _state      = decideFollowDirection(measurement);
            m_state      = State::FollowWallL;
            m_wallLocked = false;
            break;
        }
        case State::FollowWallL:
        case State::FollowWallR: {
            // bool isRightSide = m_state == State::FollowWallR;

            std::println("\n\nRobot: X:{:.2f}, Y:{:.2f}, R:{:.2f}\nTarget: X:{:.2f}, Y:{:.2f}\nDistance to target {:.2f}\n", robotPosition.x, robotPosition.y,
                         robotPosition.theta, m_destination.x, m_destination.y, currentDistToDest);

            // #define ENABLE_MOVEMENT
            // #define MAX_SCAN_TO_LOCK 600 // mm

            //             klobuky::KLaserData shortLidar{};
            //             if (!_wallLocked) {
            //                 shortLidar = measurement.findShortestMeasurement();
            //                 if (shortLidar.scanDistance < MAX_SCAN_TO_LOCK) {
            //                     if (isRightSide) {
            //                         if (shortLidar.isBetween(70, 110)) {
            //                             _wallLocked = true;
            //                         }
            //                     } else {
            //                         if (shortLidar.isBetween(-110, -70)) {
            //                             _wallLocked = true;
            //                         }
            //                     }
            //                 }
            //             } else {
            //                 if (isRightSide) {
            //                     shortLidar = measurement.findShortestMeasurementInRange(-10, 180);
            //                 } else {
            //                     shortLidar = measurement.findShortestMeasurementInRange(-180, 10);
            //                 }
            //                 if (shortLidar.scanDistance > MAX_SCAN_TO_LOCK) {
            //                     _wallLocked = false;
            //                 }
            //             }
            //             std::println("_wallLocked: {}", _wallLocked);
            //             std::println("shortLidar.scanAngle: {:.2f}", shortLidar.scanAngle);
            //             std::println("shortLidar.scanDistance: {:.2f}", shortLidar.scanDistance);

            //             auto dist_err = shortLidar.scanDistance - KConstants::kDesiredWallDistanceMM;
            //             std::println("dist_err: {:.2f}", dist_err);
            //             auto dist_y = 0.3 * dist_err; // dist_kp
            //             std::println("dist_y: {:.2f}", dist_y);

            //             auto phi_correctedLidar = (shortLidar.scanAngle > 180) ? (shortLidar.scanAngle - 360) : (shortLidar.scanAngle);

            //             auto phi_robot = (isRightSide ? 90 : -90) - phi_correctedLidar;

            //             double phi_err = phi_robot;
            //             if (isRightSide) {
            //                 phi_err = phi_robot - dist_y;
            //             } else {
            //                 phi_err = phi_robot + dist_y;
            //             }
            //             std::println("phi_err: {:.2f}", phi_err);
            //             auto phi_y = 2 * phi_err; // phi_kp
            //             std::println("phi_y: {:.2f}", phi_y);
            //             auto _rotationSpeed = phi_y * KConstants::kMaxRotationSpeed;
            //             std::println("Rotation speed: {:.2f}", _rotationSpeed toRad);
            // #ifdef ENABLE_MOVEMENT
            //             _robot.setRotationSpeed(_rotationSpeed toRad);
            // #endif

            //             // --- MAINTAIN SPEED (dist_error adjust forward speed based on distance) ---
            //             double distFrontLidar = measurement.findClosestSampleFromAngle(0).scanDistance;
            //             distFrontLidar        = std::clamp(distFrontLidar, 0.0, 1.0);
            //             std::println("distFrontLidar: {:.2f}", distFrontLidar);

            //             double distFrontSaturation =
            //                 distFrontLidar; // * std::clamp(1 - fabs(dist_err), 0., 1.0); //map(distFrontLidar,
            //                 KConstants::kDesiredWallDistanceMM, 1, 0.25, 1);

            //             // If there's a warning, reduce speed
            //             if (std::fabs(shortLidar.scanAngle) <= 60) {
            //                 distFrontSaturation = 0;
            //             }
            //             std::println("distFrontSaturation: {:.2f}", distFrontSaturation);
            //             auto _forwardSpeed = distFrontSaturation * KConstants::kMaxForwardSpeed;

            //             // if (!_wallLocked) {
            //             //     _forwardSpeed = 0;
            //             // }
            //             std::println("Forward speed: {:.2f}", _forwardSpeed);
            // #ifdef ENABLE_MOVEMENT
            //             _robot.setForwardSpeed(_forwardSpeed); // Move forward at a constant speed
            // #endif

            //             std::println();
            //             std::println("currentDistToDest: {:.2f} < _shortestDistanceToDest {:.2f} = {}", currentDistToDest,
            //             _shortestDistanceToDest,
            //                          currentDistToDest < _shortestDistanceToDest);
            //             std::println("angleToTarget: {:.2f}", angleToTarget());
            //             std::println("measurement.findClosestSampleFromAngle(angleToTarget()).scanDistance: {:.2f}",
            //                          measurement.findClosestSampleFromAngle(angleToTarget()).scanDistance);

            //             if (currentDistToDest < _shortestDistanceToDest && measurement.findClosestSampleFromAngle(angleToTarget()).scanDistance
            //             > 500 &&
            //                 measurement.findShortestMeasurementInRange(-90, 90).scanDistance > KConstants::kDesiredWallDistanceMM + 100) {
            //                 std::println("STATE: Switching to FollowDestination");
            //                 _robot.stop();
            //                 _locationBeforeDiversionX = _robot.getX();
            //                 _locationBeforeDiverionY  = _robot.getY();
            //                 _shortestDistanceToDest   = KMath::distance(_destinationX, _destinationY, _locationBeforeDiversionX,
            //                 _locationBeforeDiverionY);

            //                 _state = State::FollowDestination;
            //                 //_state = State::Invalid;
            //             }
            //             break;
        }

        case State::DoneDoneDone:
            std::println("\033[95mDONE! DONE! DONE!\033[0m");
            break;

        default:
            static bool s_informed = false;
            if (!s_informed) {
                std::println("\033[31mReached invalid state\033[0m");
                s_informed = true;
            }
            //_robot.stop();
            break;
    }
    return robotSpeed.computeControl();
}
} // namespace reanaut
