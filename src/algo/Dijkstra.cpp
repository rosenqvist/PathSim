#include "Dijkstra.hpp"

#include "AlgoUtils.hpp"
#include "core/Types.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <vector>

namespace pathsim {
namespace {

// Priority queue node. Stores the grid position and the accumulated cost
// to reach it. The greater-than operator makes std::priority_queue act as
// a min-heap: the node with the lowest cost is always on top.
struct Node {
    Vec2i pos{};
    float cost{};

    bool operator>(const Node& other) const { return cost > other.cost; }
};

} // namespace

// Dijkstra's algorithm finds the lowest-cost path from start to end by always
// expanding the cheapest frontier node next. Unlike BFS which counts hops
// Dijkstra will account for cell weights so it avoids expensive terrain.
//
// The tradeoff compared to A* is that Dijkstra has no sense of direction.
// It expands outward uniformly in all directions while visiting many nodes that
// are away from the goal. A* fixes this with a heuristic, but Dijkstra is
// useful as a baseline to show what "optimal without guidance" looks like.
//
// Data structures:
//   frontier    — min-heap ordered by accumulated cost. Ensures we always
//                 process the cheapest reachable node next.
//   came_from   — flat vector indexed by (y * width + x). Stores the parent
//                 that led to the cheapest known path to each cell.
//   cost_so_far — flat vector indexed the same way. Stores the lowest cost
//                 found so far to reach each cell. Initialized to infinity
//                 so any real cost is automatically cheaper.
//
// Stale entries: when we find a cheaper path to a node we push a new entry
// to the priority queue but can't remove the old one (std::priority_queue
// doesn't support decrease-key). So when we pop a node, we check if its
// cost matches cost_so_far. If it's higher, someone found a better route
// while this entry was sitting in the queue we skip it.
//
// Time:  O((V + E) log V) due to the priority queue operations
// Space: O(V) for the flat vectors and the priority queue

PathResult dijkstra(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "Dijkstra";

    int w = grid.width();
    int h = grid.height();
    auto total = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    // Flat vectors matching Grid's internal layout (y * width + x).
    // cost_so_far starts at infinity so any real cost compares as cheaper,
    // which lets a single "new_cost < cost_so_far[idx]" check handle both
    // "never visited" and "found a better path" without separate logic.
    std::vector<Vec2i> came_from(total, algo_utils::kNoParent);
    std::vector<float> cost_so_far(total, std::numeric_limits<float>::infinity());

    // Seed the search with the start node at zero cost
    int start_idx = algo_utils::flat_index(start, w);
    came_from[start_idx] = start;
    cost_so_far[start_idx] = 0.0F;

    // Min-heap: std::greater makes the smallest cost float to the top
    std::priority_queue<Node, std::vector<Node>, std::greater<>> frontier;
    frontier.push({.pos = start, .cost = 0.0F});

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        int current_idx = algo_utils::flat_index(current.pos, w);

        // Stale entry check: if we already found a cheaper route to this
        // position while this entry was waiting in the queue, skip it.
        // This is cheaper than implementing a full decrease-key operation.
        if (current.cost > cost_so_far[current_idx]) {
            continue;
        }

        // Record as visited for playback (skip start/end markers)
        if (current.pos != start && current.pos != end) {
            result.steps.push_back({.position = current.pos, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        // Goal is reached begin reconstructing the path and return
        if (current.pos == end) {
            algo_utils::record_path(result, came_from, grid, start, end);
            return result;
        }

        // Expand neighbors. grid.neighbors() handles walls, bounds, and
        // one-way direction filtering so we only see valid moves here.
        for (const auto& neighbor : grid.neighbors(current.pos)) {
            // New cost = cost to reach current + edge weight to neighbor.
            // move_cost() returns the destination cell's weight (or weight * sqrt(2)
            // for diagonal moves).
            float new_cost = cost_so_far[current_idx] + grid.move_cost(current.pos, neighbor);
            int neighbor_idx = algo_utils::flat_index(neighbor, w);

            // Update if this is the first time reaching this cell (cost was
            // infinity) or if we found a cheaper path than before.
            if (new_cost < cost_so_far[neighbor_idx]) {
                cost_so_far[neighbor_idx] = new_cost;
                came_from[neighbor_idx] = current.pos;
                frontier.push({.pos = neighbor, .cost = new_cost});

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

PathResult dijkstra(const Grid& grid) {
    return dijkstra(grid, grid.start(), grid.end());
}

} // namespace pathsim