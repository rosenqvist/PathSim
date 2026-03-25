#include "Persistence.hpp"

#include <fstream>
#include <sstream>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace pathsim::persistence {

namespace {

constexpr int kSaveVersion = 1;

void write_grid_cells(std::ostringstream& out, const Grid& grid) {
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            Vec2i pos{.x = x, .y = y};
            auto cell = grid.cell_data(pos);
            out << static_cast<int>(cell.wall) << ' ' << static_cast<int>(cell.weight) << ' '
                << static_cast<int>(cell.direction) << '\n';
        }
    }
}

bool read_grid_cells(std::istringstream& in, Grid& grid) {
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            int wall = 0;
            int weight = 1;
            int direction = 0;
            if (!(in >> wall >> weight >> direction)) {
                return false;
            }

            Vec2i pos{.x = x, .y = y};

            if (wall != 0) {
                grid.set_wall(pos, true);
            }
            if (weight > 1) {
                grid.set_weight(pos, weight);
            }
            if (direction != 0) {
                grid.set_direction(pos, static_cast<CellDirection>(direction));
            }
        }
    }
    return true;
}

void write_impassable_cells(std::ostringstream& out, const Grid& grid) {
    // Impassable cells are stored separately because set_wall and
    // set_impassable are different operations on the grid
    std::vector<Vec2i> impassable;
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            Vec2i pos{.x = x, .y = y};
            if (grid.at(pos) == CellState::Impassable) {
                impassable.push_back(pos);
            }
        }
    }

    out << static_cast<int>(impassable.size()) << '\n';
    for (const auto& pos : impassable) {
        out << pos.x << ' ' << pos.y << '\n';
    }
}

bool read_impassable_cells(std::istringstream& in, Grid& grid) {
    int count = 0;
    if (!(in >> count) || count < 0) {
        return false;
    }
    for (int i = 0; i < count; ++i) {
        int px = 0;
        int py = 0;
        if (!(in >> px >> py)) {
            return false;
        }
        Vec2i pos{.x = px, .y = py};
        if (grid.is_valid(pos)) {
            grid.set_impassable(pos, true);
        }
    }
    return true;
}

} // namespace

std::string serialize(const Grid& grid, const ViewSettings& view) {
    std::ostringstream out;

    // Header
    out << kSaveVersion << '\n';
    out << grid.width() << ' ' << grid.height() << '\n';

    // Start and end positions
    out << grid.start().x << ' ' << grid.start().y << ' ' << grid.end().x << ' ' << grid.end().y
        << '\n';

    // Settings
    out << static_cast<int>(grid.allow_diagonals()) << ' '
        << static_cast<int>(grid.ordered_waypoints()) << '\n';

    // View settings
    out << static_cast<int>(view.show_heatmap) << ' ' << static_cast<int>(view.show_path_direction)
        << ' ' << static_cast<int>(view.show_tooltips) << '\n';

    // Waypoints
    const auto& wps = grid.waypoints();
    out << static_cast<int>(wps.size()) << '\n';
    for (const auto& wp : wps) {
        out << wp.x << ' ' << wp.y << '\n';
    }

    // Cell data (wall, weight, direction per cell)
    write_grid_cells(out, grid);

    // Impassable cells (separate from walls)
    write_impassable_cells(out, grid);

    return out.str();
}

bool deserialize(const std::string& data, Grid& grid, ViewSettings& view) {
    if (data.empty()) {
        return false;
    }

    std::istringstream in(data);

    // Version check
    int version = 0;
    if (!(in >> version) || version != kSaveVersion) {
        return false;
    }

    // Grid dimensions
    int width = 0;
    int height = 0;
    if (!(in >> width >> height) || width < 5 || height < 5) {
        return false;
    }

    // Start and end
    int sx = 0;
    int sy = 0;
    int ex = 0;
    int ey = 0;
    if (!(in >> sx >> sy >> ex >> ey)) {
        return false;
    }

    // Settings
    int diag = 0;
    int ordered = 0;
    if (!(in >> diag >> ordered)) {
        return false;
    }

    // View settings
    int heatmap = 0;
    int path_dir = 0;
    int tooltips = 0;
    if (!(in >> heatmap >> path_dir >> tooltips)) {
        return false;
    }

    // Waypoints
    int wp_count = 0;
    if (!(in >> wp_count) || wp_count < 0) {
        return false;
    }

    std::vector<Vec2i> waypoints;
    for (int i = 0; i < wp_count; ++i) {
        int wx = 0;
        int wy = 0;
        if (!(in >> wx >> wy)) {
            return false;
        }
        waypoints.push_back({.x = wx, .y = wy});
    }

    // Apply everything to the grid
    grid.resize(width, height);
    grid.set_start({.x = sx, .y = sy});
    grid.set_end({.x = ex, .y = ey});
    grid.set_allow_diagonals(diag != 0);
    grid.set_ordered_waypoints(ordered != 0);

    if (!read_grid_cells(in, grid)) {
        return false;
    }

    if (!read_impassable_cells(in, grid)) {
        return false;
    }

    // Restore waypoints after cells so they don't get overwritten
    for (const auto& wp : waypoints) {
        grid.add_waypoint(wp);
    }

    // Apply view settings
    view.show_heatmap = heatmap != 0;
    view.show_path_direction = path_dir != 0;
    view.show_tooltips = tooltips != 0;

    return true;
}

void save_to_storage(const std::string& data) {
#ifdef __EMSCRIPTEN__
    std::string escaped;
    escaped.reserve(data.size() + 20);
    escaped += "localStorage.setItem('pathsim_save','";
    for (char c : data) {
        if (c == '\'' || c == '\\') {
            escaped += '\\';
        }
        if (c == '\n') {
            escaped += "\\n";
        } else {
            escaped += c;
        }
    }
    escaped += "');";
    emscripten_run_script(escaped.c_str());
#else
    std::ofstream file("pathsim_save.dat");
    if (file.is_open()) {
        file << data;
    }
#endif
}

std::string load_from_storage() {
#ifdef __EMSCRIPTEN__
    char* raw = emscripten_run_script_string("localStorage.getItem('pathsim_save') || ''");
    std::string result = (raw != nullptr) ? raw : "";
    return result;
#else
    std::ifstream file("pathsim_save.dat");
    if (!file.is_open()) {
        return "";
    }
    std::ostringstream buf;
    buf << file.rdbuf();
    return buf.str();
#endif
}

} // namespace pathsim::persistence