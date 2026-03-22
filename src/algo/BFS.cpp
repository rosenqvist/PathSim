#include "BFS.hpp"

#include "AlgoUtils.hpp"

#include <algorithm>
#include <queue>
#include <vector>

namespace pathsim {

// Breadth-first search explores the grid layer by layer, expanding all cells
// at distance N before any cell at distance N+1. This guarantees the fewest
// hops (edges) from start to end, but it completely ignores cell weights.
// On a weighted grid BFS often returns a more expensive path than Dijkstra
// or A* because it treats every edge as equal cost.
//
// Data structures:
//   frontier   — FIFO queue of positions to explore next. The queue ordering
//                is what gives BFS its layer-by-layer expansion pattern.
//   came_from  — flat vector indexed by (y * width + x). Each entry stores
//                the parent position that first discovered this cell. Also
//                doubles as the visited set: if came_from[idx] != kNoParent,
//                the cell has already been reached and we skip it.
//
// Time:  O(V + E) where V = grid cells, E = neighbor edges
// Space: O(V) for the came_from vector and the queue

PathResult bfs(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "BFS";

    int w = grid.width();
    int h = grid.height();
    auto total = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    // Flat came_from vector: same indexing as Grid uses internally (y * width + x).
    // Initialized to kNoParent (-1, -1) which acts as both "no parent" and "not visited".
    // This avoids the per-lookup hashing overhead of std::unordered_map.
    std::vector<Vec2i> came_from(total, algo_utils::kNoParent);

    // Mark start as visited by pointing it to itself. This is the base case
    // for the path reconstruction: we walk parents until we find a cell whose
    // parent is itself which is the start.
    came_from[algo_utils::flat_index(start, w)] = start;

    std::queue<Vec2i> frontier;
    frontier.push(start);

    while (!frontier.empty()) {
        Vec2i current = frontier.front();
        frontier.pop();

        // Record this cell as visited for playback animation. Skip start/end
        // so the renderer keeps their distinct green/red markers.
        if (current != start && current != end) {
            result.steps.push_back({.position = current, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        // Goal has been reached so reconstruct the path and return
        if (current == end) {
            algo_utils::record_path(result, came_from, grid, start, end);
            return result;
        }

        // Expand all valid neighbors. grid.neighbors() already filters out
        // walls, out-of-bounds and one-way direction violations.
        for (const auto& neighbor : grid.neighbors(current)) {
            int idx = algo_utils::flat_index(neighbor, w);

            // Only visit cells we haven't reached yet. Because BFS explores
            // in order of discovery, the first time we reach a cell is always
            // via a shortest-hop path so no need to update like we do for Dijkstra/A*.
            if (came_from[idx] == algo_utils::kNoParent) {
                came_from[idx] = current;
                frontier.push(neighbor);

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

PathResult bfs(const Grid& grid) {
    return bfs(grid, grid.start(), grid.end());
}

} // namespace pathsim