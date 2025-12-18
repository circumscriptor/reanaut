#include "constants.hpp"
#include "traversability.hpp"
#include "wavefront.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>

namespace reanaut
{

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters,readability-function-cognitive-complexity)
auto WavefrontPlanner::findPath(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld) -> bool
{

    // 1. Convert World Points to Grid Points
    auto startOpt = worldToGrid(startWorld);
    auto goalOpt  = worldToGrid(goalWorld);

    if (!startOpt || !goalOpt) {
        return false; // Out of bounds
    }

    Index start = *startOpt;
    Index goal  = *goalOpt;

    if (map.width() != width() || map.height() != height()) {
        return false;
    }

    // --- PHASE 1: PRE-PROCESSING (Thresholding) ---
    // PDF Page 12: "If value > threshold ... set to one (non traversable)"
    // We also reset the grid here.
    const Real  kObstacleThreshold = Real(0.9); // Tune this!
    const auto& travCells          = map.grid();

    std::fill(grid().begin(), grid().end(), 0);

    for (size_t i = 0; i < travCells.size(); ++i) {
        if (travCells[i] > kObstacleThreshold) {
            grid()[i] = 1; // Obstacle
        } else {
            grid()[i] = 0; // Free
        }
    }

    // Safety check: Is Start or Goal inside an obstacle?
    if (get(goal) == 1) {
        return false; // Goal blocked
    }
    // Note: If Start is blocked, we might want to search anyway to get out,
    // but strictly, it's invalid. For now, assume we can't start in a wall.

    // --- PHASE 2: WAVE PROPAGATION (BFS) ---
    // "The target cell is rated 2."

    // Set Goal
    set(goal, 2);
    m_queue.clear();
    m_queue.push_back(goal);

    bool foundStart = false;

    // 8-Connectivity Neighbors (PDF Figure 14)
    static const auto kDx = std::to_array({0, 0, -1, 1, -1, -1, 1, 1});
    static const auto kDy = std::to_array({-1, 1, 0, 0, -1, 1, -1, 1});

    while (!m_queue.empty()) {
        auto current = m_queue.front();
        m_queue.pop_front();

        auto currentValue = get(current);
        if (not currentValue) {
            continue;
        }

        // Stop if we reached start (Optimization: partial map update)
        // If you want to visualize the FULL map, remove this check.
        if (current.x == start.x && current.y == start.y) {
            foundStart = true;
            break;
        }

        // Check Neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = current.x + kDx[i];
            int ny = current.y + kDy[i];

            if (auto neighborVal = get(Index(nx, ny)); neighborVal) {
                if (neighborVal == 0) {
                    neighborVal = *currentValue + 1;
                    set(Index(nx, ny), *currentValue + 1);
                    m_queue.push_back({nx, ny});
                }
            }
        }
    }

    if (not foundStart) {
        return false; // Path not possible
    }

    // --- PHASE 3: BACKTRACKING (Gradient Descent) ---
    // Start at 'Start', move to neighbor with lowest value until 'Goal' (val 2)

    auto current = start;
    m_path.push_back(gridToWorld(current)); // Add Start

    while (true) {
        int currentVal = at(current);
        if (currentVal == 2) {
            break; // Reached Goal
        }

        int  bestVal  = currentVal;
        auto bestNext = current;
        bool moved    = false;

        // Find neighbor with lowest value
        // PDF mentions "preferred direction" for tie-breaking.
        // Simple loop order acts as a fixed preference (e.g. prioritize Up/Left)
        for (int i = 0; i < 8; ++i) {
            int nx = current.x + kDx[i];
            int ny = current.y + kDy[i];

            if (auto val = get(Index(nx, ny)); val) {
                // Must be > 1 (not obstacle, not unvisited) and strictly smaller
                if (val > 1 && val < bestVal) {
                    bestVal  = *val;
                    bestNext = {.x = nx, .y = ny};
                    moved    = true;
                }
            }
        }

        if (!moved) {
            break; // Stuck (shouldn't happen if BFS found start)
        }

        current = bestNext;
        m_path.push_back(gridToWorld(current));
    }

    return true;
}

// NOLINTBEGIN
auto getWaveColor(int value) -> uint32_t
{
    if (value == 0) {
        return 0xFF000000; // Unreachable (Black)
    }
    if (value == 1) {
        return 0xFF0000FF; // Obstacle (Red)
    }
    if (value == 2) {
        return 0xFF00FF00; // Goal (Green)
    }

    // Gradient for the wave (Blue -> Cyan -> White)
    int     intensity = (value * 5) % 255;
    uint8_t v         = static_cast<uint8_t>(intensity);
    return 0xFFFF0000 | (static_cast<uint32_t>(v) << 8);
}
// NOLINTEND

} // namespace reanaut
