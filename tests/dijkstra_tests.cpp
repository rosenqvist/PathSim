#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Dijkstra finds a path on an open grid", "[dijkstra]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
}

TEST_CASE("Dijkstra finds the shortest path on an open grid", "[dijkstra]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE(result.path.size() == 9);
    REQUIRE(result.path_cost == 8.0F);
}

TEST_CASE("Dijkstra returns empty path when completely walled off", "[dijkstra]") {
    pathsim::Grid grid(5, 5);

    for (int col = 0; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 2}, true);
    }

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE(result.path.empty());
}

TEST_CASE("Dijkstra navigates around walls", "[dijkstra]") {
    pathsim::Grid grid(5, 5);

    for (int col = 0; col < 4; ++col) {
        grid.set_wall({.x = col, .y = 1}, true);
    }
    for (int col = 1; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 3}, true);
    }

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
    REQUIRE(result.path.size() > 9);
}

TEST_CASE("Dijkstra records visited nodes", "[dijkstra]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE(result.nodes_visited > 0);
    REQUIRE_FALSE(result.steps.empty());
}

TEST_CASE("Dijkstra finds cheapest path over fewest hops", "[dijkstra]") {
    pathsim::Grid grid(10, 3);

    grid.set_start({.x = 0, .y = 1});
    grid.set_end({.x = 9, .y = 1});

    // Middle row is expensive
    for (int col = 1; col < 9; ++col) {
        grid.set_weight({.x = col, .y = 1}, 9);
    }

    pathsim::PathResult bfs_result = pathsim::bfs(grid);
    pathsim::PathResult dijkstra_result = pathsim::dijkstra(grid);

    REQUIRE_FALSE(dijkstra_result.path.empty());
    // Dijkstra's path should cost less than BFS's path
    REQUIRE(dijkstra_result.path_cost < bfs_result.path_cost);
    // Dijkstra takes a longer route to avoid expensive cells
    REQUIRE(dijkstra_result.path.size() > bfs_result.path.size());
}

TEST_CASE("Dijkstra path cost reflects weights", "[dijkstra]") {
    pathsim::Grid grid(3, 1);

    grid.set_start({.x = 0, .y = 0});
    grid.set_end({.x = 2, .y = 0});
    grid.set_weight({.x = 1, .y = 0}, 5);

    pathsim::PathResult result = pathsim::dijkstra(grid);

    REQUIRE_FALSE(result.path.empty());
    // Path: (0,0) -> (1,0) -> (2,0), costs 5 + 1 = 6
    REQUIRE(result.path_cost == 6.0F);
}