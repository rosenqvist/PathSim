#include "AStar.hpp"

#include "AlgoUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <queue>
#include <vector>

namespace pathsim {
namespace {

// Priority queue node for A*. Stores position, f-cost (priority), and g-cost.
//
// f = g + h where:
//   g = actual accumulated cost from start to this node
//   h = heuristic estimate of remaining cost to end
//
// The priority queue orders by f-cost (lowest first). When two nodes have
// the same f-cost we prefer the one with higher g-cost. Same f but higher g
// means lower h, which means the node is estimated to be closer to the goal.
// This tie-breaking reduces unnecessary exploration in open areas where many
// nodes share the same f-value.
struct Node {
    Vec2i pos{};
    float priority{}; // f-cost = g + h
    float g_cost{};   // actual cost from start

    bool operator>(const Node& other) const {
        if (priority != other.priority) {
            return priority > other.priority;
        }
        // Tie-break: prefer higher g (closer to goal by estimate)
        return g_cost < other.g_cost;
    }
};

// Computes the heuristic estimate from position a to position b.
//
// For cardinal-only movement: Manhattan distance (|dx| + |dy|).
// Each step costs at minimum 1 (the lowest cell weight), so Manhattan
// distance never overestimates the true cost, keeping A* optimal.
//
// For diagonal movement: octile distance. The idea is that diagonal steps
// cost sqrt(2) while cardinal steps cost 1. So the cheapest way to cover
// dx and dy is to take min(dx,dy) diagonal steps and the rest as cardinal.
// Formula: min(dx,dy) * sqrt(2) + |dx - dy| (good read:
// https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html)
//
// Both heuristics assume minimum weight of 1. Since all cell weights are
// >= 1, the heuristic never overestimates, which is the acceptable
// requirement for A* to guarantee optimal paths.
float heuristic(Vec2i a, Vec2i b, bool diagonals) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);

    if (diagonals) {
        // Octile distance: diagonal steps cost sqrt(2) while cardinal steps cost 1
        int mn = std::min(dx, dy);
        int mx = std::max(dx, dy);
        return (static_cast<float>(mn) * std::numbers::sqrt2_v<float>)+static_cast<float>(mx - mn);
    }

    // Manhattan distance: only cardinal movement allowed
    return static_cast<float>(dx + dy);
}

} // namespace

// A* is Dijkstra's algorithm with a heuristic that biases expansion toward
// the goal. Instead of ordering the priority queue by g-cost alone (cost from
// start), A* uses f = g + h where h estimates the remaining cost to the goal.
//
// This means A* still guarantees optimal paths (as long as h is acceptable)
// but typically this results in visits to far fewer nodes than Dijkstra because it doesn't waste
// time exploring directions away from the goal.
//
// The implementation mirrors Dijkstra almost exactly. The only differences are:
//   1. The priority queue orders by f = g + h instead of just g
//   2. Tie-breaking prefers higher g (see Node::operator> above)
//   3. The heuristic function is called when computing each node's priority
//
// Data structures: same as Dijkstra — flat vectors for came_from and
// cost_so_far, min-heap priority queue. See Dijkstra.cpp for detailed
// explanations of the stale entry handling and flat vector layout.
//
// Time:  O((V + E) log V) worst case, but typically much less than Dijkstra
//        because the heuristic trims away large portions of the search space
// Space: O(V) for the flat vectors and the priority queue

PathResult a_star(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "A*";

    int w = grid.width();
    int h = grid.height();
    auto total = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    // Flat vectors matching Grid's internal layout (y * width + x).
    // Same rationale as Dijkstra: avoids hashing overhead, better cache
    // locality, and consistent with how the Grid stores its own data.
    std::vector<Vec2i> came_from(total, algo_utils::kNoParent);
    std::vector<float> cost_so_far(total, std::numeric_limits<float>::infinity());

    // Seed the search. Initial priority = h(start, end) since g = 0.
    int start_idx = algo_utils::flat_index(start, w);
    came_from[start_idx] = start;
    cost_so_far[start_idx] = 0.0F;

    bool diag = grid.allow_diagonals();

    std::priority_queue<Node, std::vector<Node>, std::greater<>> frontier;
    frontier.push({.pos = start, .priority = heuristic(start, end, diag), .g_cost = 0.0F});

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        int current_idx = algo_utils::flat_index(current.pos, w);

        // Stale entry check: if we already found a cheaper route to this
        // position, skip this outdated queue entry. This happens because
        // std::priority_queue doesn't support decrease-key, so we push new
        // entries and lazily discard old ones here.
        if (current.g_cost > cost_so_far[current_idx]) {
            continue;
        }

        // Record as visited for playback (skip start/end markers)
        if (current.pos != start && current.pos != end) {
            result.steps.push_back({.position = current.pos, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        // Goal has been reached so reconstruct the path and return
        if (current.pos == end) {
            algo_utils::record_path(result, came_from, grid, start, end);
            return result;
        }

        // Expand neighbors. Same as Dijkstra but priority includes the heuristic.
        for (const auto& neighbor : grid.neighbors(current.pos)) {
            float new_cost = cost_so_far[current_idx] + grid.move_cost(current.pos, neighbor);
            int neighbor_idx = algo_utils::flat_index(neighbor, w);

            // Update if first visit (cost was infinity) or found a cheaper path
            if (new_cost < cost_so_far[neighbor_idx]) {
                cost_so_far[neighbor_idx] = new_cost;
                came_from[neighbor_idx] = current.pos;

                // f = g + h. The heuristic biases expansion toward the goal,
                // which is the key difference from Dijkstra.
                float priority = new_cost + heuristic(neighbor, end, diag);
                frontier.push({.pos = neighbor, .priority = priority, .g_cost = new_cost});

                // Record as frontier for the blue "wave" animation
                if (neighbor != end) {
                    result.steps.push_back(
                        {.position = neighbor, .new_state = CellState::Frontier});
                }
            }
        }

        // Track peak frontier size for the stats panel
        int frontier_size = static_cast<int>(frontier.size());
        result.max_frontier_size = std::max(frontier_size, result.max_frontier_size);
    }

    // Queue exhausted without finding the end so no path exists.
    return result;
}

PathResult a_star(const Grid& grid) {
    return a_star(grid, grid.start(), grid.end());
}

} // namespace pathsim