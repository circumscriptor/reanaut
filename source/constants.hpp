#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>

namespace reanaut
{

using RealType  = double;
using IndexType = std::size_t;
using DiffType  = std::ptrdiff_t;

static constexpr double    kCalibrationTime             = 2.0;
static constexpr double    kDefaultNavigateToX          = -0.5;
static constexpr double    kDefaultNavigateToY          = 5.0;
static constexpr double    kDesiredWallDistance         = 0.4; // meters
static constexpr double    kDesiredWallDistanceMm       = kDesiredWallDistance * 1000.0;
static constexpr double    kDistanceThresholdLower      = 0.2;
static constexpr double    kDistanceThresholdUpper      = 2.0;
static constexpr double    kForwardSpeedMultiplier      = 1000.0;
static constexpr double    kMapTileSize                 = 0.05;
static constexpr double    kMaxDistanceError            = 0.15;
static constexpr double    kMaxForwardSpeed             = 0.5;
static constexpr double    kMaxRotationError            = 0.5;
static constexpr double    kMaxRotationSpeedRadps       = std::numbers::pi / 2.0;
static constexpr double    kMillisecondToSecond         = 0.001;
static constexpr double    kMinDistanceError            = 0.25;
static constexpr double    kMinRotationError            = 0.05;
static constexpr double    kNoteConversionFactor        = 0.00000275;
static constexpr double    kObstacleDetectDistance      = 0.5; // meters
static constexpr double    kObstacleDetectDistanceMm    = kObstacleDetectDistance * 1000.0;
static constexpr double    kPathClearanceDistance       = 1.5; // meters
static constexpr double    kPathClearanceDistanceMm     = kPathClearanceDistance * 1000.0;
static constexpr double    kRadiusThreshold             = 0.2;
static constexpr double    kRotateD                     = 0.0;
static constexpr double    kRotateI                     = 0.1;
static constexpr double    kRotateP                     = 1.0;
static constexpr double    kTickToMeter                 = 0.000085292090497737556558;
static constexpr double    kTranslateD                  = 0.01;
static constexpr double    kTranslateI                  = 0.2;
static constexpr double    kTranslateP                  = 0.4;
static constexpr double    kWallFollowSpeed             = 10.0; // mm/s
static constexpr double    kWheelbaseDistance           = 0.23; // [meters]
static constexpr double    kWheelbaseDistanceMm         = kWheelbaseDistance * 1000.0;
static constexpr float     kBreakpointDistanceMm        = 200.0F;
static constexpr float     kDistanceSpikeThreshold      = 0.25F;
static constexpr float     kLineFittingErrorThresholdMm = 100.0F; // 10cm
static constexpr float     kMaxLaserRangeMm             = 8000.F;
static constexpr float     kMinLaserRangeMm             = 150.F;
static constexpr IndexType kMapTilesX                   = 120;
static constexpr IndexType kMapTilesY                   = 120;
static constexpr int       kMedianWindowSize            = 2;
static constexpr int       kMinPointsForWall            = 10;
static constexpr int32_t   kUpdateIntervalMs            = 25;
static constexpr uint32_t  kMaskU16                     = 0xFFFF;
static constexpr uint32_t  kMaskU8                      = 0xFF;
static constexpr uint32_t  kNeighborThreshold           = 6;
static constexpr uint32_t  kQualityThreshold            = 100;
static constexpr uint32_t  kTileHitThreshold            = 3;
static constexpr uint32_t  kWallWeight                  = 9;
static constexpr uint32_t  kWeightThresholdLower        = 3;
static constexpr uint32_t  kWeightThresholdUpper        = 3;

// Robot/Sensor Specs
const double kLidarMaxRange = 10.0;
const double kLidarMinRange = 0.1;
const double kMapResolution = 0.05;  // 5cm per cell
const int    kMapWidth      = 400;   // 20 meters wide
const int    kMapHeight     = 400;   // 20 meters high
const double kMapOriginX    = -10.0; // Meters (Center the map)
const double kMapOriginY    = -10.0;

// Log Odds Parameters (Tuned for stability)
const double kLogOddsOccupied = 0.85; // Probability if hit
const double kLogOddsFree     = 0.4;  // Probability if passed through
const double kLogOddsMax      = 5.0;  // Clamping max
const double kLogOddsMin      = -5.0; // Clamping min

// Noise parameters
const double kStdX     = 0.05;
const double kStdY     = 0.05;
const double kStdTheta = 0.02;
const double kStdLidar = 0.2;

// from kobuki manual https://yujinrobot.github.io/kobuki/doxygen/enAppendixProtocolSpecification.html

constexpr auto isZero(double value) noexcept -> bool { return std::abs(value) <= std::numeric_limits<double>::epsilon(); }

struct Point2
{
    using Real = RealType;

    Real x{};
    Real y{};
};

struct Pose : Point2
{
    Real theta{};
};

} // namespace reanaut
