#pragma once

#include <cmath>
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
static constexpr double    kMaxRotationSpeedRadps       = std::numbers::pi / 4.0;
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
static constexpr double    kScanDistanceToWorld         = 0.001;
static constexpr double    kTickToDegree                = 0.01;
static constexpr double    kTickToMeter                 = 0.000085292090497737556558;
static constexpr double    kTranslateD                  = 0.0;
static constexpr double    kTranslateI                  = 0.0;
static constexpr double    kTranslateP                  = 1.0;
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
static constexpr double kLidarMaxRange = 20.0;
static constexpr double kLidarMinRange = 0.1;
static constexpr double kMapResolution = 0.05; // 5cm per cell
static constexpr double kMapLength     = 15.0; // 10 meters
static constexpr double kMapHalfLength = kMapLength / 2.0;
static constexpr int    kMapWidth      = int(kMapLength / kMapResolution);
static constexpr int    kMapHeight     = int(kMapLength / kMapResolution);
static constexpr double kMapOriginX    = -kMapHalfLength; // Meters (Center the map)
static constexpr double kMapOriginY    = -kMapHalfLength;

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

    auto operator-(const Point2& other) const noexcept -> Point2
    {
        return {
            .x = x - other.x,
            .y = y - other.y,
        };
    }

    [[nodiscard]] auto dot(const Point2& other) const noexcept -> Real { return (x * other.x) + (y * other.y); }
    [[nodiscard]] auto lengthSq() const noexcept -> Real { return dot(*this); }
    [[nodiscard]] auto length() const noexcept -> Real { return std::sqrt(lengthSq()); }
    [[nodiscard]] auto distanceSq(const Point2& other) const -> Real { return (*this - other).lengthSq(); }
    [[nodiscard]] auto distance(const Point2& other) const -> Real { return (*this - other).length(); }
};

struct Pose : Point2
{
    Real theta{};
};

struct Index
{
    using Type = int;

    union Helper {
        struct
        {
            int x;
            int y;
        };
        size_t v;
    };

    Type x{};
    Type y{};

    [[nodiscard]]
    auto pack() const -> size_t
    {
        Helper packer{.x = x, .y = y};
        return packer.v;
    }

    [[nodiscard]]
    static auto unpack(size_t index) -> Index
    {
        Helper unpacker{.v = index};
        return {
            .x = unpacker.x,
            .y = unpacker.y,
        };
    }

    auto operator==(const Index&) const -> bool = default;
};

} // namespace reanaut
