#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "algo/Playback.hpp"
#include "core/Grid.hpp"
#include "ui/GridRenderer.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdlib>
#include <string>

#ifdef __EMSCRIPTEN__
#include <GLFW/emscripten_glfw3.h>

#include <emscripten.h>
#include <emscripten/html5.h>

#endif

namespace menu_bar {
void draw(pathsim::Grid& grid, pathsim::GridRenderer& renderer, pathsim::Playback& playback);
} // namespace menu_bar

namespace stats_panel {
void draw(const pathsim::Playback& playback);
} // namespace stats_panel

struct AppState {
    GLFWwindow* window{};
    pathsim::Grid grid;
    pathsim::GridRenderer renderer;
    pathsim::Playback playback;

    AppState() : grid(40, 30) {}
};

void main_loop(void* arg) {
    auto* app = static_cast<AppState*>(arg);

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    app->playback.update(app->grid);
    app->renderer.draw(app->grid);

    bool can_edit = app->playback.state() == pathsim::PlaybackState::Idle ||
                    app->playback.state() == pathsim::PlaybackState::Finished;
    app->renderer.handle_input(app->grid, can_edit);

    menu_bar::draw(app->grid, app->renderer, app->playback);
    stats_panel::draw(app->playback);

    ImGui::Render();

    int w{};
    int h{};
    glfwGetFramebufferSize(app->window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(app->window);
}

int main() {
    if (glfwInit() == 0) {
        return EXIT_FAILURE;
    }

#ifdef __EMSCRIPTEN__
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    GLFWwindow* window = glfwCreateWindow(2560, 1440, "PathSim", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

#ifdef __EMSCRIPTEN__
    // Let contrib.glfw3 handle canvas resizing to fill the browser window
    emscripten::glfw3::MakeCanvasResizable(window, "window");
#endif

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    float dpi_scale = 1.0F;
#ifdef __EMSCRIPTEN__
    dpi_scale = static_cast<float>(emscripten_get_device_pixel_ratio());
#else
    float x_scale = 1.0F;
    float y_scale = 1.0F;
    glfwGetWindowContentScale(window, &x_scale, &y_scale);
    dpi_scale = x_scale;
#endif

    ImGuiIO& io = ImGui::GetIO();
    float font_size = 16.0F * dpi_scale;

#ifdef __EMSCRIPTEN__
    io.Fonts->AddFontFromFileTTF("/resources/fonts/Roboto-Medium.ttf", font_size);
    io.FontGlobalScale = 1.0F / dpi_scale;
#else
    io.Fonts->AddFontFromFileTTF("resources/fonts/Roboto-Medium.ttf", font_size);
    ImGui::GetStyle().ScaleAllSizes(dpi_scale);
#endif

    ImGui_ImplGlfw_InitForOpenGL(window, true);

#ifdef __EMSCRIPTEN__
    ImGui_ImplOpenGL3_Init("#version 300 es");
#else
    ImGui_ImplOpenGL3_Init("#version 330");
#endif

    AppState app;
    app.window = window;

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(main_loop, &app, 0, true);
#else
    while (glfwWindowShouldClose(window) == 0) {
        main_loop(&app);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
#endif

    return EXIT_SUCCESS;
}

namespace menu_bar {

void draw_algorithm_menu(pathsim::Playback& playback, pathsim::Grid& grid) {
    if (!ImGui::BeginMenu("Algorithm")) {
        return;
    }

    auto state = playback.state();

    bool can_run =
        state == pathsim::PlaybackState::Idle || state == pathsim::PlaybackState::Finished;

    if (ImGui::MenuItem("Run BFS", nullptr, false, can_run)) {
        playback.start(pathsim::bfs(grid), grid);
    }
    if (ImGui::MenuItem("Run Dijkstra", nullptr, false, can_run)) {
        playback.start(pathsim::dijkstra(grid), grid);
    }
    if (ImGui::MenuItem("Run A*", nullptr, false, can_run)) {
        playback.start(pathsim::a_star(grid), grid);
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Pause", nullptr, false, state == pathsim::PlaybackState::Playing)) {
        playback.pause();
    }
    if (ImGui::MenuItem("Resume", nullptr, false, state == pathsim::PlaybackState::Paused)) {
        playback.resume();
    }
    if (ImGui::MenuItem("Reset", nullptr, false, state != pathsim::PlaybackState::Idle)) {
        playback.reset(grid);
    }

    ImGui::Separator();

    int speed = playback.speed();
    if (ImGui::SliderInt("Speed", &speed, 1, 50)) {
        playback.set_speed(speed);
    }

    ImGui::EndMenu();
}

void draw(pathsim::Grid& grid, pathsim::GridRenderer& renderer, pathsim::Playback& playback) {
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }

    if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Clear Grid")) {
            playback.reset(grid);
            grid.clear();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Tools")) {
        auto tool = renderer.active_tool();
        if (ImGui::MenuItem("Wall", nullptr, tool == pathsim::EditTool::Wall)) {
            renderer.set_tool(pathsim::EditTool::Wall);
        }
        if (ImGui::MenuItem("Start", nullptr, tool == pathsim::EditTool::Start)) {
            renderer.set_tool(pathsim::EditTool::Start);
        }
        if (ImGui::MenuItem("End", nullptr, tool == pathsim::EditTool::End)) {
            renderer.set_tool(pathsim::EditTool::End);
        }
        if (ImGui::MenuItem("Erase", nullptr, tool == pathsim::EditTool::Erase)) {
            renderer.set_tool(pathsim::EditTool::Erase);
        }
        ImGui::Separator();

        if (ImGui::MenuItem("Weight Brush", nullptr, tool == pathsim::EditTool::Weight)) {
            renderer.set_tool(pathsim::EditTool::Weight);
        }

        if (tool == pathsim::EditTool::Weight) {
            int weight = renderer.weight_brush();
            if (ImGui::SliderInt("Weight", &weight, 1, 9)) {
                renderer.set_weight_brush(weight);
            }
        }
        ImGui::EndMenu();
    }

    draw_algorithm_menu(playback, grid);

    ImGui::EndMainMenuBar();
}
} // namespace menu_bar

namespace stats_panel {

void draw(const pathsim::Playback& playback) {
    if (playback.state() == pathsim::PlaybackState::Idle) {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(10.0F, 40.0F), ImGuiCond_FirstUseEver);

    ImGui::Begin("Stats", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const auto& result = playback.result();

    ImGui::Text("Algorithm: %s", result.algorithm_name);

    ImGui::Separator();

    ImGui::Text("Progress: %d / %d", playback.current_step(), playback.total_steps());
    ImGui::Text("Nodes explored: %d", result.nodes_visited);

    if (playback.state() == pathsim::PlaybackState::Finished) {
        ImGui::Separator();

        if (result.path.empty()) {
            ImGui::TextColored(ImVec4(1.0F, 0.3F, 0.3F, 1.0F), "No path found");
        } else {
            ImGui::Text("Path length: %d steps", static_cast<int>(result.path.size()) - 1);
            ImGui::Text("Path cost: %.0f", static_cast<double>(result.path_cost));

            if (std::string_view(result.algorithm_name) == "BFS") {
                ImGui::TextColored(ImVec4(1.0F, 0.8F, 0.3F, 1.0F),
                                   "BFS optimizes for fewest hops, not lowest cost");
            } else {
                ImGui::TextColored(ImVec4(0.3F, 0.8F, 0.4F, 1.0F),
                                   "%s optimizes for lowest total cost", result.algorithm_name);
            }
        }

        ImGui::Separator();

        if (ImGui::Button("Copy Stats")) {
            std::string stats;
            stats += std::string(result.algorithm_name) + "\n";
            stats += "Nodes explored: " + std::to_string(result.nodes_visited) + "\n";
            stats += "Path length: " + std::to_string(static_cast<int>(result.path.size()) - 1) +
                     " steps\n";
            stats += "Path cost: " + std::to_string(result.path_cost) + "\n";
            ImGui::SetClipboardText(stats.c_str());
        }
    }

    ImGui::End();
}

} // namespace stats_panel