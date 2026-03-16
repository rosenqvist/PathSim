#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("A* finds a path on an open grid", "[astar]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::a_star(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
}

TEST_CASE("A* finds the shortest path on an open grid", "[astar]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::a_star(grid);

    REQUIRE(result.path.size() == 9);
    REQUIRE(result.path_cost == 8.0F);
}

TEST_CASE("A* returns empty path when completely walled off", "[astar]") {
    pathsim::Grid grid(5, 5);

    for (int col = 0; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 2}, true);
    }

    pathsim::PathResult result = pathsim::a_star(grid);

    REQUIRE(result.path.empty());
}

TEST_CASE("A* navigates around walls", "[astar]") {
    pathsim::Grid grid(5, 5);

    for (int col = 0; col < 4; ++col) {
        grid.set_wall({.x = col, .y = 1}, true);
    }
    for (int col = 1; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 3}, true);
    }

    pathsim::PathResult result = pathsim::a_star(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
    REQUIRE(result.path.size() > 9);
}

TEST_CASE("A* visits fewer nodes than BFS", "[astar]") {
    pathsim::Grid grid(10, 10);

    pathsim::PathResult bfs_result = pathsim::bfs(grid);
    pathsim::PathResult astar_result = pathsim::a_star(grid);

    // Same path length
    REQUIRE(bfs_result.path.size() == astar_result.path.size());

    // A* should explore fewer nodes due to heuristics
    REQUIRE(astar_result.nodes_visited < bfs_result.nodes_visited);
}