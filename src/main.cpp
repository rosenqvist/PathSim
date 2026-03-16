#include "algo/BFS.hpp"
#include "algo/Playback.hpp"
#include "core/Grid.hpp"
#include "ui/GridRenderer.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdlib>

namespace menu_bar {
void draw(pathsim::Grid& grid, pathsim::GridRenderer& renderer, pathsim::Playback& playback);
} // namespace menu_bar

int main() {
    if (glfwInit() == 0) {
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1920, 1080, "PathSim", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    pathsim::Grid grid(30, 20);
    pathsim::GridRenderer renderer;
    pathsim::Playback playback;

    while (glfwWindowShouldClose(window) == 0) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        playback.update(grid);
        renderer.draw(grid);
        renderer.handle_input(grid);
        menu_bar::draw(grid, renderer, playback);

        ImGui::Render();

        int w{};
        int h{};
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}

namespace menu_bar {

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
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Algorithm")) {
        auto state = playback.state();

        if (ImGui::MenuItem("Run BFS", nullptr, false,
                            state == pathsim::PlaybackState::Idle ||
                                state == pathsim::PlaybackState::Finished)) {
            playback.start(pathsim::bfs(grid), grid);
        }

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

    ImGui::EndMainMenuBar();
}

} // namespace menu_bar