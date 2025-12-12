#include "constants.hpp"
#include "laser.hpp"
#include "navigator.hpp"
#include "particle.hpp"
#include "tangent_bug.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <utility>
#include <vector>

#define robotStop           \
    robotSpeed.angular = 0; \
    robotSpeed.linear  = 0

#define toRad *std::numbers::pi / 180
#define toDeg *180 / std::numbers::pi

namespace reanaut
{

TangentBug::TangentBug() : m_pointFollower({.kP = kTranslateP, .kI = kTranslateI, .kD = kTranslateD}, {.kP = kRotateP, .kI = kRotateI, .kD = kRotateD}) {};

auto TangentBug::process(const std::vector<LaserScan>& scans, Particle robotPosition, RealType dt) -> std::pair<uint16_t, uint16_t>
{
    Velocity robotSpeed;
    robotSpeed.angular = 0;
    robotSpeed.linear  = 0;

    auto currentDistToDest = distance(m_destination.x, m_destination.y, robotPosition.x, robotPosition.y);

    if (!(dt <= std::numeric_limits<RealType>::epsilon())) { // returns
        switch (m_state) {
            case State::FollowDestination: {

                std::println("\t[Tangentbug] Is active {}", m_pointFollower.isActive());
                if (not m_pointFollower.isActive()) {
                    m_pointFollower.setGoal(m_destination);
                    std::println("\t[Tangentbug] STATE: FollowDestination - Setting target.");
                }

                const auto shortestDistance = findShortestMeasurementInRange(scans, -45.0, 45.0).distance;
                std::println("\t[Tangentbug] shortest scan distance: {:.2f}", shortestDistance);

                if (shortestDistance < kObstacleDetectDistanceMm && shortestDistance > 0.0) {
                    std::println("\t[Tangentbug] STATE: FollowDestination -> Obstacle detected! Switching to Decision state.");
                    robotStop;
                    m_locationBeforeDiversion.x = robotPosition.x;
                    m_locationBeforeDiversion.y = robotPosition.y;
                    m_shortestDistanceToDest    = distance(m_destination.x, m_destination.y, m_locationBeforeDiversion.x, m_locationBeforeDiversion.y);
                    m_state                     = State::DecideWallFollow;
                } else {
                    std::println("\t[Tangentbug] APPROACHING TARGET");
                    auto temp = m_pointFollower.update(robotPosition, dt);

                    // std::println("\t[Tangentbug] navigator ({}) returned: lin={}, angular={}", m_pointFollower.isActive() ? "active" : "inactive",
                    // temp->linear,temp->angular);
                    if (temp) {
                        robotSpeed = *temp;
                    } else {
                        std::println("\t[Tangentbug] Navigator update returned invalid data");
                    }
                }

                if (m_pointFollower.targetReached()) {
                    robotStop;
                    m_state = State::DoneDoneDone;
                }

                break;
            }

            case State::DecideWallFollow: {
                // _state      = decideFollowDirection(scans);
                m_state      = State::FollowWallL;
                m_wallLocked = false;
                break;
            }
            case State::FollowWallL:
            case State::FollowWallR: {
                bool isRightSide = m_state == State::FollowWallR;

                std::println("\t[Tangentbug] \n\nRobot: X:{:.2f}, Y:{:.2f}, R:{:.2f}\nTarget: X:{:.2f}, Y:{:.2f}\nDistance to target {:.2f}\n", robotPosition.x,
                             robotPosition.y, robotPosition.theta, m_destination.x, m_destination.y, currentDistToDest);

#define MAX_SCAN_TO_LOCK 600 // mm

                LaserScan shortLidar{};
                if (!m_wallLocked) {
                    shortLidar = findShortestMeasurement(scans);
                    if (shortLidar.distance < MAX_SCAN_TO_LOCK) {
                        if (isRightSide) {
                            if (shortLidar.isBetween(70, 110)) {
                                m_wallLocked = true;
                            }
                        } else {
                            if (shortLidar.isBetween(-110, -70)) {
                                m_wallLocked = true;
                            }
                        }
                    }
                } else {
                    if (isRightSide) {
                        shortLidar = findShortestMeasurementInRange(scans, -10, 180);
                    } else {
                        shortLidar = findShortestMeasurementInRange(scans, -180, 10);
                    }
                    if (shortLidar.distance > MAX_SCAN_TO_LOCK) {
                        m_wallLocked = false;
                    }
                }
                std::println("\t[Tangentbug] m_wallLocked: {}", m_wallLocked);
                std::println("\t[Tangentbug] shortLidar.angle: {:.2f}", shortLidar.angle);
                std::println("\t[Tangentbug] shortLidar.distance: {:.2f}", shortLidar.distance);

                auto distErr = shortLidar.distance - kDesiredWallDistanceMm;
                std::println("\t[Tangentbug] dist_err: {:.2f}", distErr);
                auto distY = 0.3 * distErr; // dist_kp
                std::println("\t[Tangentbug] dist_y: {:.2f}", distY);

                auto phiCorrectedLidar = (shortLidar.angle > 180) ? (shortLidar.angle - 360) : (shortLidar.angle);

                auto phiRobot = (isRightSide ? 90 : -90) - phiCorrectedLidar;

                double phiErr = phiRobot;
                if (isRightSide) {
                    phiErr = phiRobot - distY;
                } else {
                    phiErr = phiRobot + distY;
                }
                std::println("\t[Tangentbug] phi_err: {:.2f}", phiErr);
                auto phiY = 2 * phiErr; // phi_kp
                std::println("\t[Tangentbug] phi_y: {:.2f}", phiY);
                auto rotationSpeed = phiY * kMaxRotationSpeedRadps;
                rotationSpeed      = std::clamp(rotationSpeed, -7.0 toDeg, 7.0 toDeg);
                std::println("\t[Tangentbug] Rotation speed: {:.2f}", rotationSpeed toRad);

                robotSpeed.angular = rotationSpeed toRad;

                // --- MAINTAIN SPEED (dist_error adjust forward speed based on distance) ---
                double distFrontLidar         = findShortestMeasurementInRange(scans, -20, 20).distance - 400;
                distFrontLidar                = std::clamp(distFrontLidar, 0.0, 200.0);
                auto distFrontLidarMultiplier = map(distFrontLidar, 0, 200, 0.05, 1);
                std::println("\t[Tangentbug] distFrontLidarMultiplier: {:.2f}", distFrontLidarMultiplier);

                auto wallDistanceMultiplier = std::clamp(fabs(shortLidar.distance - kDesiredWallDistanceMm), 0.0, 200.0);
                // wallDistanceMultiplier -= 400;
                wallDistanceMultiplier = map(wallDistanceMultiplier, 0, 200, 0.5, 1);
                std::println("\t[Tangentbug] wallDistanceMultiplier: {:.2f}", wallDistanceMultiplier);

                auto   wallLockedMultiplier   = m_wallLocked ? 0.8 : 0.4;
                double forwardSpeedMulitplyer = distFrontLidarMultiplier * wallDistanceMultiplier * wallLockedMultiplier;

                // If there's a warning, reduce speed
                if (std::fabs(shortLidar.angle) <= 60) {
                    forwardSpeedMulitplyer = 0;
                }
                std::println("\t[Tangentbug] linear speed multiplier: {:.2f}", forwardSpeedMulitplyer);
                auto forwardSpeed = kMaxForwardSpeed * forwardSpeedMulitplyer; // * fabs(1 - map(phiRobot,0,360,0,1));

                // if (!m_wallLocked) {
                //     _forwardSpeed = 0;
                // }
                std::println("\t[Tangentbug] Forward speed: {:.2f}", forwardSpeed);

                robotSpeed.linear = forwardSpeed;

                std::println();
                std::println("\t[Tangentbug] currentDistToDest: {:.2f} < _shortestDistanceToDest {:.2f} = {}", currentDistToDest, m_shortestDistanceToDest,
                             currentDistToDest < m_shortestDistanceToDest);
                std::println("\t[Tangentbug] angleToTarget: {:.2f}", angleToTarget(robotPosition));
                std::println("\t[Tangentbug] findClosestSampleFromAngle(angleToTarget()).distance: {:.2f}",
                             findClosestSampleFromAngle(scans, angleToTarget(robotPosition)).distance);

                if (currentDistToDest < m_shortestDistanceToDest && findClosestSampleFromAngle(scans, angleToTarget(robotPosition)).distance > 500 &&
                    findShortestMeasurementInRange(scans, -90, 90).distance > kDesiredWallDistanceMm + 100) {
                    std::println("\t[Tangentbug] STATE: Switching to FollowDestination");
                    robotStop;
                    m_locationBeforeDiversion = robotPosition;
                    m_shortestDistanceToDest  = distance(m_destination, m_locationBeforeDiversion);

                    m_state = State::FollowDestination;
                    // m_state = State::Invalid;
                }
                break;
            }

            case State::DoneDoneDone:
                std::println("\t[Tangentbug] \033[95mDONE! DONE! DONE!\033[0m");
                break;

            default:
                static bool s_informed = false;
                if (!s_informed) {
                    std::println("\t[Tangentbug] \033[31mReached invalid state\033[0m");
                    s_informed = true;
                }
                //_robot.stop();
                break;
        }
    }

    std::println("\t[Tangentbug] \t[Tangentbug] returning: linear={}, angular={}", robotSpeed.linear, robotSpeed.angular);
    return robotSpeed.computeControl();
}

auto TangentBug::angleToTarget(Point2 location) -> double
{
    float dx = m_destination.x - location.x;
    float dy = m_destination.y - location.y;

    // atan2 returns radians, convert to degrees
    // (Assuming standard math coordinates: 0 is East/Right, CCW)
    float angleRad = std::atan2(dy, dx);
    auto  angleDeg = normalizeAngle360(angleRad toDeg);

    return angleDeg;
}

} // namespace reanaut
