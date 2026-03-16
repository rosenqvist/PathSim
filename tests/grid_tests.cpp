#include "../src/core/Grid.hpp"

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