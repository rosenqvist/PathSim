#include "MultiPath.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace pathsim {

namespace {

std::vector<float> precompute_costs(const Grid& grid, const AlgoFunc& algo,
                                    const std::vector<Vec2i>& stops) {
    auto count = stops.size();
    std::vector<float> costs(count * count, -1.0F);

    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) {
                costs[(i * count) + j] = 0.0F;
                continue;
            }
            auto result = algo(grid, stops[i], stops[j]);
            if (!result.path.empty()) {
                costs[(i * count) + j] = result.path_cost;
            }
        }
    }
    return costs;
}

std::vector<int> brute_force_order(const std::vector<float>& costs, int stop_count,
                                   std::vector<int> middle) {
    int wp_count = static_cast<int>(middle.size());
    int end_idx = stop_count - 1;
    auto count = static_cast<std::size_t>(stop_count);

    float best_cost = std::numeric_limits<float>::max();
    std::vector<int> best_order = middle;

    // middle is already sorted from iota so next_permutation covers all orderings
    for (bool has_next = true; has_next;
         has_next = std::next_permutation(middle.begin(), middle.end())) {

        float total = 0.0F;
        bool valid = true;

        // start -> first waypoint
        float c = costs[static_cast<std::size_t>(middle[0])];
        if (c < 0.0F) {
            valid = false;
        } else {
            total += c;
        }

        // waypoint chain
        for (int i = 0; valid && i + 1 < wp_count; ++i) {
            c = costs[(static_cast<std::size_t>(middle[i]) * count) +
                      static_cast<std::size_t>(middle[i + 1])];
            if (c < 0.0F) {
                valid = false;
            } else {
                total += c;
            }
        }

        // last waypoint -> end
        if (valid) {
            c = costs[(static_cast<std::size_t>(middle[wp_count - 1]) * count) +
                      static_cast<std::size_t>(end_idx)];
            if (c < 0.0F) {
                valid = false;
            } else {
                total += c;
            }
        }

        if (valid && total < best_cost) {
            best_cost = total;
            best_order = middle;
        }
    }

    return best_order;
}

std::vector<int> nearest_neighbor_order(const std::vector<float>& costs, int stop_count,
                                        const std::vector<int>& middle) {
    int wp_count = static_cast<int>(middle.size());
    auto count = static_cast<std::size_t>(stop_count);

    std::vector<uint8_t> used(static_cast<std::size_t>(wp_count), 0);
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(wp_count));

    int current = 0; // start index
    for (int step = 0; step < wp_count; ++step) {
        float best = std::numeric_limits<float>::max();
        int best_idx = -1;
        for (int j = 0; j < wp_count; ++j) {
            if (used[static_cast<std::size_t>(j)] != 0U) {
                continue;
            }
            float c = costs[(static_cast<std::size_t>(current) * count) +
                            static_cast<std::size_t>(middle[j])];
            if (c >= 0.0F && c < best) {
                best = c;
                best_idx = j;
            }
        }
        if (best_idx < 0) {
            // Unreachable waypoint so append the remaining in original order
            for (int j = 0; j < wp_count; ++j) {
                if (used[static_cast<std::size_t>(j)] == 0U) {
                    order.push_back(middle[j]);
                }
            }
            break;
        }
        used[static_cast<std::size_t>(best_idx)] = 1;
        order.push_back(middle[best_idx]);
        current = middle[best_idx];
    }

    return order;
}

void append_segment(PathResult& combined, PathResult& segment,
                    std::vector<AlgoStep>& exploration_steps, std::vector<AlgoStep>& path_steps) {
    combined.algorithm_name = segment.algorithm_name;
    combined.nodes_visited += segment.nodes_visited;
    combined.path_cost += segment.path_cost;
    combined.max_frontier_size = std::max(segment.max_frontier_size, combined.max_frontier_size);

    for (const auto& step : segment.steps) {
        if (step.new_state == CellState::Path) {
            path_steps.push_back(step);
        } else {
            exploration_steps.push_back(step);
        }
    }

    if (combined.path.empty()) {
        for (const auto& pos : segment.path) {
            combined.path.push_back(pos);
        }
    } else {
        for (std::size_t j = 1; j < segment.path.size(); ++j) {
            combined.path.push_back(segment.path[j]);
        }
    }
}

} // namespace

PathResult find_path_with_waypoints(const Grid& grid, const AlgoFunc& algo) {
    const auto& waypoints = grid.waypoints();

    std::vector<Vec2i> stops;
    stops.push_back(grid.start());
    for (const auto& wp : waypoints) {
        stops.push_back(wp);
    }
    stops.push_back(grid.end());

    // Find optimal visit order when unordered with 2+ waypoints
    if (!grid.ordered_waypoints() && waypoints.size() >= 2) {
        auto costs = precompute_costs(grid, algo, stops);
        int stop_count = static_cast<int>(stops.size());
        int wp_count = stop_count - 2;

        std::vector<int> middle(static_cast<std::size_t>(wp_count));
        std::iota(middle.begin(), middle.end(), 1);

        auto best_order = (wp_count <= 8) ? brute_force_order(costs, stop_count, middle)
                                          : nearest_neighbor_order(costs, stop_count, middle);

        std::vector<Vec2i> reordered;
        reordered.push_back(grid.start());
        for (int idx : best_order) {
            reordered.push_back(stops[static_cast<std::size_t>(idx)]);
        }
        reordered.push_back(grid.end());
        stops = std::move(reordered);
    }

    PathResult combined;
    combined.algorithm_name = "None";

    std::vector<AlgoStep> exploration_steps;
    std::vector<AlgoStep> path_steps;

    for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
        PathResult segment = algo(grid, stops[i], stops[i + 1]);

        if (segment.path.empty()) {
            combined.algorithm_name = segment.algorithm_name;
            combined.steps = std::move(exploration_steps);
            combined.path.clear();
            combined.path_cost = 0.0F;
            return combined;
        }

        append_segment(combined, segment, exploration_steps, path_steps);
    }

    // All exploration first then path
    combined.steps = std::move(exploration_steps);
    for (const auto& step : path_steps) {
        combined.steps.push_back(step);
    }

    return combined;
}

} // namespace pathsim