cmake_minimum_required(VERSION 3.25)
project(PathSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Core library (testable without UI, no platform dependencies) ---
add_library(pathsim_core STATIC
    src/core/Grid.cpp
    src/algo/BFS.cpp
    src/algo/Dijkstra.cpp
    src/algo/AStar.cpp
    src/algo/Playback.cpp
    src/algo/MazeGen.cpp
    src/algo/MultiPath.cpp
    src/algo/AlgoUtils.cpp
)
target_compile_options(pathsim_core PRIVATE
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror -Wno-unused-function>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Release>>:-O3>
)
target_include_directories(pathsim_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

if(EMSCRIPTEN)
    # --- Emscripten build: fetch ImGui, use Emscripten GLFW/OpenGL ---
    include(FetchContent)
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.6
    )
    FetchContent_MakeAvailable(imgui)

    add_library(imgui_em STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui_em PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_compile_options(imgui_em PRIVATE -Wno-everything)

    add_executable(PathSim
        src/main.cpp
        src/ui/Persistence.cpp
        src/ui/GridRenderer.cpp
        src/ui/MenuBar.cpp

        src/ui/KeyboardShortcuts.cpp
    )
    target_compile_options(PathSim PRIVATE -Wall -Wextra -Wpedantic -Werror --use-port=contrib.glfw3)
    target_link_libraries(PathSim PRIVATE pathsim_core imgui_em)

    set_target_properties(PathSim PROPERTIES
        SUFFIX ".html"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/web"
    )
    target_link_options(PathSim PRIVATE
        --use-port=contrib.glfw3
        -sWASM=1
        -sALLOW_MEMORY_GROWTH=1
        -sMIN_WEBGL_VERSION=2
        -sMAX_WEBGL_VERSION=2
        --shell-file ${CMAKE_SOURCE_DIR}/web/shell.html
        --embed-file ${CMAKE_SOURCE_DIR}/resources/fonts/Roboto-Medium.ttf@/resources/fonts/Roboto-Medium.ttf
    )

else()
    # --- Desktop build: use vcpkg packages ---
    find_package(imgui CONFIG REQUIRED)
    find_package(glfw3 CONFIG REQUIRED)
    find_package(OpenGL REQUIRED)

    add_executable(PathSim
        src/main.cpp
        src/ui/Persistence.cpp
        src/ui/GridRenderer.cpp
        src/ui/MenuBar.cpp
        src/ui/KeyboardShortcuts.cpp
    )
    target_compile_options(PathSim PRIVATE
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
        $<$<AND:$<NOT:$<CXX_COMPILER_ID:MSVC>>,$<CONFIG:Release>>:-O3>
    )
    target_link_libraries(PathSim
        PRIVATE
            pathsim_core
            imgui::imgui
            glfw
            OpenGL::GL
    )

    add_custom_command(TARGET PathSim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/resources
            $<TARGET_FILE_DIR:PathSim>/resources
    )

    # --- Tests (desktop only) ---
    find_package(Catch2 3 CONFIG REQUIRED)
    enable_testing()

    add_executable(pathsim_tests
        tests/grid_tests.cpp
        tests/bfs_tests.cpp
        tests/dijkstra_tests.cpp
        tests/astar_tests.cpp
        tests/playback_tests.cpp
        tests/mazegen_tests.cpp
        tests/waypoint_tests.cpp
        tests/multipath_tests.cpp
        tests/diagonal_tests.cpp
        tests/oneway_tests.cpp
        tests/persistence_tests.cpp
        src/ui/Persistence.cpp
    )
    target_compile_options(pathsim_tests PRIVATE
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
    target_link_libraries(pathsim_tests
        PRIVATE
            pathsim_core
            Catch2::Catch2WithMain
    )

    include(CTest)
    include(Catch)
    catch_discover_tests(pathsim_tests)
endif()