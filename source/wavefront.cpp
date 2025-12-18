#include "constants.hpp"
#include "traversability.hpp"
#include "wavefront.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace reanaut
{

namespace
{
constexpr auto kDx = std::to_array({0, 0, -1, 1, -1, -1, 1, 1});
constexpr auto kDy = std::to_array({-1, 1, 0, 0, -1, 1, -1, 1});
} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto WavefrontPlanner::update(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld) -> bool
{
    Index startIdx{};
    Index goalIdx{};

    // Phase 1: Preparation
    if (!initializeSearch(map, startWorld, goalWorld, startIdx, goalIdx)) {
        return false;
    }

    // Phase 2: Wave Propagation (BFS)
    if (!propagateWave(startIdx, goalIdx)) {
        return false;
    }

    // Phase 3: Path Reconstruction (Pipeline)
    reconstructPath(startIdx);

    return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto WavefrontPlanner::initializeSearch(const TraversabilityGrid& map, Point2 startWorld, Point2 goalWorld, Index& startIdx, Index& goalIdx) -> bool
{
    if (map.width() != width() || map.height() != height()) {
        return false;
    }

    auto startOpt = worldToGrid(startWorld);
    auto goalOpt  = worldToGrid(goalWorld);

    if (!startOpt || !goalOpt) {
        return false;
    }

    startIdx = *startOpt;
    goalIdx  = *goalOpt;

    updateObstacles(map);

    if (get(goalIdx) == 1) {
        return false;
    }

    // Clear output buffers
    m_path.clear();
    // Note: m_rawPath and m_checkpoints are cleared inside their respective generation functions

    return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto WavefrontPlanner::propagateWave(Index startIdx, Index goalIdx) -> bool
{
    set(goalIdx, 2);
    m_queue.clear();
    m_queue.push_back(goalIdx);

    bool foundStart = false;

    while (!m_queue.empty()) {
        const auto current = m_queue.front();
        m_queue.pop_front();

        const int currentValue = *get(current); // Safety: BFS logic ensures valid indices

        if (current.x == startIdx.x && current.y == startIdx.y) {
            foundStart = true;
            break;
        }

        for (size_t i = 0; i < kDx.size(); ++i) {
            Index next{current.x + kDx[i], current.y + kDy[i]};
            if (auto neighborVal = get(next); neighborVal && *neighborVal == 0) {
                set(next, currentValue + 1);
                m_queue.push_back(next);
            }
        }
    }
    return foundStart;
}

void WavefrontPlanner::reconstructPath(Index startIdx)
{
    // Execute pipeline
    generateDensePath(startIdx);
    extractCorners();
    smoothPath();
}

void WavefrontPlanner::generateDensePath(Index startIdx)
{
    m_rawPath.clear();

    auto current = startIdx;
    m_rawPath.push_back(current);

    while (true) {
        int currentVal = at(current);
        if (currentVal == 2) {
            break; // Goal
        }

        int   bestVal  = currentVal;
        Index bestNext = current;
        bool  moved    = false;

        for (size_t i = 0; i < kDx.size(); ++i) {
            const Index next{
                .x = current.x + kDx[i],
                .y = current.y + kDy[i],
            };

            // Fast path: direct access if you trust bounds, otherwise use get()
            if (auto valOpt = get(next); valOpt) {
                int val = *valOpt;
                if (val > 1 && val < bestVal) {
                    bestVal  = val;
                    bestNext = next;
                    moved    = true;
                }
            }
        }

        if (!moved) {
            break;
        }
        current = bestNext;
        m_rawPath.push_back(current);
    }
}

void WavefrontPlanner::extractCorners()
{
    m_checkpoints.clear();

    if (m_rawPath.empty()) {
        return;
    }

    // Start is always a checkpoint
    m_checkpoints.push_back(m_rawPath.front());

    if (m_rawPath.size() < 3) {
        if (m_rawPath.size() == 2) {
            m_checkpoints.push_back(m_rawPath.back());
        }
        return;
    }

    Index prev     = m_rawPath.front();
    int   prevDirX = 0;
    int   prevDirY = 0;

    // Detect direction changes
    for (size_t i = 1; i < m_rawPath.size(); ++i) {
        const auto& curr = m_rawPath[i];
        const int   dirX = curr.x - prev.x;
        const int   dirY = curr.y - prev.y;

        // If not the first segment, and direction changed
        if (i > 1 && (dirX != prevDirX || dirY != prevDirY)) {
            m_checkpoints.push_back(prev);
        }

        prev     = curr;
        prevDirX = dirX;
        prevDirY = dirY;
    }

    // End is always a checkpoint
    m_checkpoints.push_back(m_rawPath.back());
}

void WavefrontPlanner::smoothPath()
{
    // Output directly to m_path (World coordinates)
    // Input is m_checkpoints (Grid coordinates)

    if (m_checkpoints.empty()) {
        return;
    }

    // Always convert the first point
    m_path.push_back(gridToWorld(m_checkpoints.front()));

    if (m_checkpoints.size() == 1) {
        return;
    }

    Index from  = m_checkpoints.front();
    Index valid = m_checkpoints[1];

    // Raycast reduction ("String Pulling")
    for (size_t i = 1; i < m_checkpoints.size(); ++i) {
        const auto& candidate = m_checkpoints[i];

        if (!checkLine(from, candidate)) {
            // Line of sight broken: add the last valid point
            m_path.push_back(gridToWorld(valid));
            from = valid;
        }
        valid = candidate;
    }

    // Always add the final target
    m_path.push_back(gridToWorld(m_checkpoints.back()));
}

auto WavefrontPlanner::checkLine(Index p0, Index p1) const -> bool
{
    int x0 = p0.x;
    int y0 = p0.y;
    int x1 = p1.x;
    int y1 = p1.y;

    int dx  = std::abs(x1 - x0);
    int dy  = -std::abs(y1 - y0);
    int sx  = x0 < x1 ? 1 : -1;
    int sy  = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        // value_or(1) treats out-of-bounds as obstacle
        if (get(Index(x0, y0)).value_or(1) == 1) {
            return false;
        }

        if (x0 == x1 && y0 == y1) {
            break;
        }

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
    return true;
}

void WavefrontPlanner::updateObstacles(const TraversabilityGrid& traversability)
{
    auto travData = traversability.grid();
    for (size_t i = 0; i < travData.size(); ++i) {
        setAtOffset(i, (travData[i] > kObstacleThreshold) ? 1 : 0);
    }
}

// NOLINTBEGIN
auto getWaveColor(int value) -> uint32_t
{
    if (value == 0) {
        return 0xFF000000;
    }
    if (value == 1) {
        return 0xFF0000FF;
    }
    if (value == 2) {
        return 0xFF00FF00;
    }
    int intensity = (value * 5) % 255;
    return 0xFFFF0000 | (static_cast<uint32_t>(intensity) << 8);
}
// NOLINTEND

} // namespace reanaut
