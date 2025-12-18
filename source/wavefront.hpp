#pragma once

#include "constants.hpp"
#include "grid.hpp"
#include "traversability.hpp"

#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace reanaut
{

class WavefrontPlanner : public IntegerGrid
{
public:

    static constexpr Real kObstacleThreshold = Real(0.9);

    auto update(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld) -> bool;

    [[nodiscard]] auto path() const -> std::span<const Point2> { return m_path; }

protected:

    void updateObstacles(const TraversabilityGrid& traversability);

private:

    // --- Helper Functions (void returns) ---

    [[nodiscard]] auto initializeSearch(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld, Index& startIdx, Index& goalIdx) -> bool;

    [[nodiscard]] auto propagateWave(Index startIdx, Index goalIdx) -> bool;

    // Orchestrates the path generation pipeline
    void reconstructPath(Index startIdx);

    // Step 1: Gradient descent from Start -> Goal (fills m_rawPath)
    void generateDensePath(Index startIdx);

    // Step 2: Extract corners (reads m_rawPath, fills m_checkpoints)
    void extractCorners();

    // Step 3: Raycast smoothing (reads m_checkpoints, fills m_path)
    void smoothPath();

    // Bresenham line check
    [[nodiscard]] auto checkLine(Index p0, Index p1) const -> bool;

    // --- Data ---
    // Final output (World coordinates)
    std::vector<Point2> m_path;

    // Reusable buffers (Grid coordinates) to avoid allocation
    std::vector<Index> m_rawPath;     // The step-by-step grid path
    std::vector<Index> m_checkpoints; // The reduced corner points
    std::deque<Index>  m_queue;       // BFS queue
};

auto getWaveColor(int value) -> uint32_t;

} // namespace reanaut
