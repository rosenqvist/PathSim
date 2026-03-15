#include "core/Grid.hpp"
#include "ui/GridRenderer.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdlib>

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

    while (glfwWindowShouldClose(window) == 0) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderer.draw(grid);
        renderer.handle_input(grid);

        // toolbar (drawn after grid so it appears on top)
        ImGui::Begin("Tools", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        auto tool = renderer.active_tool();
        if (ImGui::RadioButton("Wall", tool == pathsim::EditTool::Wall)) {
            renderer.set_tool(pathsim::EditTool::Wall);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Start", tool == pathsim::EditTool::Start)) {
            renderer.set_tool(pathsim::EditTool::Start);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("End", tool == pathsim::EditTool::End)) {
            renderer.set_tool(pathsim::EditTool::End);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Erase", tool == pathsim::EditTool::Erase)) {
            renderer.set_tool(pathsim::EditTool::Erase);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            grid.clear();
        }

        ImGui::End();

        renderer.draw(grid);
        renderer.handle_input(grid);

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