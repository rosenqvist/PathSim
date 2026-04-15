#include "core/Grid.hpp"
#include "ui/Persistence.hpp"
#include "ui/ViewSettings.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Round-trip preserves grid dimensions", "[persistence]") {
    pathsim::Grid grid(15, 10);
    pathsim::ViewSettings view;
    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings restored_view;
    REQUIRE(pathsim::persistence::deserialize(data, restored, restored_view));

    REQUIRE(restored.width() == 15);
    REQUIRE(restored.height() == 10);
}

TEST_CASE("Round-trip preserves start and end positions", "[persistence]") {
    pathsim::Grid grid(20, 20);
    grid.set_start({.x = 3, .y = 7});
    grid.set_end({.x = 15, .y = 12});
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.start() == pathsim::Vec2i{.x = 3, .y = 7});
    REQUIRE(restored.end() == pathsim::Vec2i{.x = 15, .y = 12});
}

TEST_CASE("Round-trip preserves walls and weights", "[persistence]") {
    pathsim::Grid grid(10, 10);
    grid.set_wall({.x = 3, .y = 3}, true);
    grid.set_wall({.x = 4, .y = 4}, true);
    grid.set_weight({.x = 5, .y = 5}, 7);
    grid.set_weight({.x = 6, .y = 6}, 3);
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.is_wall({.x = 3, .y = 3}));
    REQUIRE(restored.is_wall({.x = 4, .y = 4}));
    REQUIRE_FALSE(restored.is_wall({.x = 5, .y = 5}));
    REQUIRE(restored.weight({.x = 5, .y = 5}) == 7);
    REQUIRE(restored.weight({.x = 6, .y = 6}) == 3);
}

TEST_CASE("Round-trip preserves waypoints", "[persistence]") {
    pathsim::Grid grid(10, 10);
    grid.add_waypoint({.x = 2, .y = 3});
    grid.add_waypoint({.x = 7, .y = 8});
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.waypoints().size() == 2);
    REQUIRE(restored.waypoints()[0] == pathsim::Vec2i{.x = 2, .y = 3});
    REQUIRE(restored.waypoints()[1] == pathsim::Vec2i{.x = 7, .y = 8});
    REQUIRE(restored.at({.x = 2, .y = 3}) == pathsim::CellState::Waypoint);
}

TEST_CASE("Round-trip preserves impassable cells", "[persistence]") {
    pathsim::Grid grid(10, 10);
    grid.set_impassable({.x = 4, .y = 4}, true);
    grid.set_impassable({.x = 6, .y = 6}, true);
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.at({.x = 4, .y = 4}) == pathsim::CellState::Impassable);
    REQUIRE(restored.at({.x = 6, .y = 6}) == pathsim::CellState::Impassable);
}

TEST_CASE("Round-trip preserves directions", "[persistence]") {
    pathsim::Grid grid(10, 10);
    grid.set_direction({.x = 3, .y = 3}, pathsim::CellDirection::East);
    grid.set_direction({.x = 5, .y = 5}, pathsim::CellDirection::North);
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.direction({.x = 3, .y = 3}) == pathsim::CellDirection::East);
    REQUIRE(restored.direction({.x = 5, .y = 5}) == pathsim::CellDirection::North);
}

TEST_CASE("Round-trip preserves view settings", "[persistence]") {
    pathsim::Grid grid(10, 10);
    pathsim::ViewSettings view{
        .show_heatmap = true,
        .show_path_direction = true,
        .show_tooltips = false,
    };

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(rv.show_heatmap == true);
    REQUIRE(rv.show_path_direction == true);
    REQUIRE(rv.show_tooltips == false);
}

TEST_CASE("Round-trip preserves diagonal and ordered settings", "[persistence]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);
    grid.set_ordered_waypoints(false);
    pathsim::ViewSettings view;

    auto data = pathsim::persistence::serialize(grid, view);

    pathsim::Grid restored(5, 5);
    pathsim::ViewSettings rv;
    REQUIRE(pathsim::persistence::deserialize(data, restored, rv));

    REQUIRE(restored.allow_diagonals() == true);
    REQUIRE(restored.ordered_waypoints() == false);
}

TEST_CASE("Deserialize rejects empty data", "[persistence]") {
    pathsim::Grid grid(10, 10);
    pathsim::ViewSettings view;

    REQUIRE_FALSE(pathsim::persistence::deserialize("", grid, view));
}

TEST_CASE("Deserialize rejects wrong version", "[persistence]") {
    pathsim::Grid grid(10, 10);
    pathsim::ViewSettings view;

    REQUIRE_FALSE(pathsim::persistence::deserialize("99\n10 10\n", grid, view));
}
