#include "algo/BFS.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>


TEST_CASE("Grid initializes with correct dimensions", "[grid]") {
    pathsim::Grid grid(10, 8);

    REQUIRE(grid.width() == 10);
    REQUIRE(grid.height() == 8);
}

TEST_CASE("Grid start & end are placed on construction", "[grid]") {
    pathsim::Grid grid(10, 8);

    REQUIRE(grid.at(grid.start()) == pathsim::CellState::Start);
    REQUIRE(grid.at(grid.end()) == pathsim::CellState::End);
}

TEST_CASE("Setting a wall marks the cell", "[grid]") {
    pathsim::Grid grid(5, 5);
    pathsim::Vec2i pos{.x = 2, .y = 2};

    grid.set_wall(pos, true);

    REQUIRE(grid.is_wall(pos));
    REQUIRE(grid.at(pos) == pathsim::CellState::Wall);
}

TEST_CASE("Walls cannot overwrite start or end", "[grid]") {
    pathsim::Grid grid(5, 5);

    grid.set_wall(grid.start(), true);
    grid.set_wall(grid.end(), true);

    REQUIRE_FALSE(grid.is_wall(grid.start()));
    REQUIRE_FALSE(grid.is_wall(grid.end()));
}

TEST_CASE("Clear resets the entire grid", "[grid]") {
    pathsim::Grid grid(5, 5);

    grid.set_wall({.x = 1, .y = 1}, true);
    grid.set_wall({.x = 2, .y = 2}, true);
    grid.clear();

    REQUIRE_FALSE(grid.is_wall({.x = 1, .y = 1}));
    REQUIRE_FALSE(grid.is_wall({.x = 2, .y = 2}));
    REQUIRE(grid.at(grid.start()) == pathsim::CellState::Start);
    REQUIRE(grid.at(grid.end()) == pathsim::CellState::End);
}

TEST_CASE("Resize changes grid dimensions", "[grid]") {
    pathsim::Grid grid(10, 8);

    grid.resize(20, 15);

    REQUIRE(grid.width() == 20);
    REQUIRE(grid.height() == 15);
}

TEST_CASE("Resize places start and end correctly", "[grid]") {
    pathsim::Grid grid(10, 8);

    grid.resize(20, 15);

    REQUIRE(grid.start() == pathsim::Vec2i{.x = 0, .y = 0});
    REQUIRE(grid.end() == pathsim::Vec2i{.x = 19, .y = 14});
    REQUIRE(grid.at(grid.start()) == pathsim::CellState::Start);
    REQUIRE(grid.at(grid.end()) == pathsim::CellState::End);
}

TEST_CASE("Resize clears all walls and weights", "[grid]") {
    pathsim::Grid grid(10, 10);

    grid.set_wall({.x = 3, .y = 3}, true);
    grid.set_weight({.x = 5, .y = 5}, 7);
    grid.resize(10, 10);

    REQUIRE_FALSE(grid.is_wall({.x = 3, .y = 3}));
    REQUIRE(grid.weight({.x = 5, .y = 5}) == 1);
}

TEST_CASE("Resize clears waypoints", "[grid]") {
    pathsim::Grid grid(10, 10);

    grid.add_waypoint({.x = 5, .y = 5});
    grid.add_waypoint({.x = 7, .y = 7});
    grid.resize(10, 10);

    REQUIRE(grid.waypoints().empty());
}

TEST_CASE("Resize clears directions", "[grid]") {
    pathsim::Grid grid(10, 10);

    grid.set_direction({.x = 3, .y = 3}, pathsim::CellDirection::East);
    grid.resize(10, 10);

    REQUIRE(grid.direction({.x = 3, .y = 3}) == pathsim::CellDirection::None);
}

TEST_CASE("Resize to smaller grid is valid", "[grid]") {
    pathsim::Grid grid(40, 30);

    grid.resize(5, 5);

    REQUIRE(grid.width() == 5);
    REQUIRE(grid.height() == 5);
    REQUIRE(grid.at(grid.start()) == pathsim::CellState::Start);
    REQUIRE(grid.at(grid.end()) == pathsim::CellState::End);
}

TEST_CASE("Resize to larger grid is valid", "[grid]") {
    pathsim::Grid grid(5, 5);

    grid.resize(200, 200);

    REQUIRE(grid.width() == 200);
    REQUIRE(grid.height() == 200);
    REQUIRE(grid.at(grid.start()) == pathsim::CellState::Start);
    REQUIRE(grid.at(grid.end()) == pathsim::CellState::End);
}

TEST_CASE("Algorithms work after resize", "[grid]") {
    pathsim::Grid grid(40, 30);

    grid.resize(15, 10);

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
}