#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "algo/MultiPath.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("No waypoints falls back to direct pathfind", "[multipath]") {
    pathsim::Grid grid(10, 10);

    auto direct = pathsim::bfs(grid, grid.start(), grid.end());
    auto multi = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });

    REQUIRE(multi.path.size() == direct.path.size());
    REQUIRE(multi.path_cost == direct.path_cost);
}

TEST_CASE("Single waypoint produces path through it", "[multipath]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};
    grid.add_waypoint(wp);

    auto result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());

    // Verify waypoint appears in the path
    bool found = false;
    for (const auto& pos : result.path) {
        if (pos == wp) {
            found = true;
            break;
        }
    }
    REQUIRE(found);
}

TEST_CASE("Multiple waypoints are visited in order", "[multipath]") {
    pathsim::Grid grid(20, 10);
    pathsim::Vec2i wp1{.x = 5, .y = 5};
    pathsim::Vec2i wp2{.x = 15, .y = 5};
    grid.add_waypoint(wp1);
    grid.add_waypoint(wp2);

    auto result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });

    REQUIRE_FALSE(result.path.empty());

    // Find indices of waypoints in path
    int idx1 = -1;
    int idx2 = -1;
    for (int i = 0; i < static_cast<int>(result.path.size()); ++i) {
        if (result.path[static_cast<std::size_t>(i)] == wp1 && idx1 == -1) {
            idx1 = i;
        }
        if (result.path[static_cast<std::size_t>(i)] == wp2 && idx2 == -1) {
            idx2 = i;
        }
    }

    REQUIRE(idx1 >= 0);
    REQUIRE(idx2 >= 0);
    REQUIRE(idx1 < idx2);
}

TEST_CASE("Unreachable waypoint returns empty path", "[multipath]") {
    pathsim::Grid grid(10, 10);

    // Wall off a cell completely and place waypoint there
    pathsim::Vec2i wp{.x = 5, .y = 5};
    grid.set_wall({.x = 4, .y = 5}, true);
    grid.set_wall({.x = 6, .y = 5}, true);
    grid.set_wall({.x = 5, .y = 4}, true);
    grid.set_wall({.x = 5, .y = 6}, true);
    grid.add_waypoint(wp);

    auto result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });

    REQUIRE(result.path.empty());
}

TEST_CASE("Waypoint path works with all three algorithms", "[multipath]") {
    pathsim::Grid grid(10, 10);
    grid.add_waypoint({.x = 5, .y = 5});

    auto bfs_result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });
    auto dij_result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::dijkstra(g, s, e);
        });
    auto astar_result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::a_star(g, s, e);
        });

    REQUIRE_FALSE(bfs_result.path.empty());
    REQUIRE_FALSE(dij_result.path.empty());
    REQUIRE_FALSE(astar_result.path.empty());
}

TEST_CASE("Path has no duplicate nodes at waypoint junctions", "[multipath]") {
    pathsim::Grid grid(10, 10);
    grid.add_waypoint({.x = 5, .y = 5});

    auto result = pathsim::find_path_with_waypoints(
        grid, [](const pathsim::Grid& g, pathsim::Vec2i s, pathsim::Vec2i e) {
            return pathsim::bfs(g, s, e);
        });

    REQUIRE_FALSE(result.path.empty());

    // Check no adjacent duplicates
    for (std::size_t i = 1; i < result.path.size(); ++i) {
        REQUIRE(result.path[i] != result.path[i - 1]);
    }
}