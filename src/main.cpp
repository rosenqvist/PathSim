#include "algo/Playback.hpp"
#include "core/Grid.hpp"
#include "ui/GridRenderer.hpp"
#include "ui/KeyboardShortcuts.hpp"
#include "ui/MenuBar.hpp"
#include "ui/Persistence.hpp"
#include "ui/ViewSettings.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cstdlib>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/html5.h>

extern "C" int emscripten_glfw_make_canvas_resizable(GLFWwindow* window,
                                                     const char* canvas_resize_selector,
                                                     const char* handle_selector);
#endif

struct AppState {
    GLFWwindow* window{};
    pathsim::Grid grid;
    pathsim::GridRenderer renderer;
    pathsim::Playback playback;
    pathsim::AlgoHistory history;
    pathsim::ViewSettings view;

    float save_timer{0.0F};
    AppState() : grid(40, 30) {}
};

void main_loop(void* arg) {
    auto* app = static_cast<AppState*>(arg);

    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    app->playback.update(app->grid);

    if (app->playback.state() == pathsim::PlaybackState::Finished) {
        app->renderer.draw(app->grid, app->view, &app->playback.result());
    } else {
        app->renderer.draw(app->grid, app->view);
    }

    bool can_edit = app->playback.state() == pathsim::PlaybackState::Idle ||
                    app->playback.state() == pathsim::PlaybackState::Finished;
    app->renderer.handle_input(app->grid, can_edit);

    pathsim::menu_bar::draw(app->grid, app->renderer, app->playback, app->history, app->view);

    pathsim::keyboard_shortcuts::handle(app->grid, app->renderer, app->playback, app->view);

    // Auto-save every 2 seconds
    app->save_timer += ImGui::GetIO().DeltaTime;
    if (app->save_timer >= 2.0F) {
        app->save_timer = 0.0F;
        auto data = pathsim::persistence::serialize(app->grid, app->view);
        pathsim::persistence::save_to_storage(data);
    }

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
    emscripten_glfw_make_canvas_resizable(window, "window", nullptr);
#endif

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    float dpi_scale{1.0F};
#ifdef __EMSCRIPTEN__
    dpi_scale = static_cast<float>(emscripten_get_device_pixel_ratio());
#else
    float x_scale{1.0F};
    float y_scale{1.0F};
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

    // Restore previous session if available
    auto saved = pathsim::persistence::load_from_storage();
    pathsim::persistence::deserialize(saved, app.grid, app.view);

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