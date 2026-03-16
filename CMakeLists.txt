cmake_minimum_required(VERSION 3.25)
project(PathSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)

# --- Main application ---
add_executable(PathSim src/main.cpp src/ui/GridRenderer.cpp)
target_compile_options(PathSim PRIVATE
    -Wall -Wextra -Wpedantic -Werror
    $<$<CONFIG:Release>:-O3>
)

target_link_libraries(PathSim
    PRIVATE
        pathsim_core
        imgui::imgui
        glfw
        OpenGL::GL
)

# --- Core library (testable without UI) ---
add_library(pathsim_core STATIC
    src/core/Grid.cpp
    src/algo/BFS.cpp
    src/algo/Dijkstra.cpp
    src/algo/AStar.cpp
    src/algo/Playback.cpp
)
target_compile_options(pathsim_core PRIVATE
    -Wall -Wextra -Wpedantic -Werror -Wno-unused-function
    $<$<CONFIG:Release>:-O3>
)

target_include_directories(pathsim_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/src
)

# --- Tests ---
enable_testing()

add_executable(pathsim_tests
    tests/grid_tests.cpp
    tests/bfs_tests.cpp
    tests/dijkstra_tests.cpp
    tests/astar_tests.cpp
    tests/playback_tests.cpp
)
target_compile_options(pathsim_tests PRIVATE -Wall -Wextra)

target_link_libraries(pathsim_tests
    PRIVATE
        pathsim_core
        Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(pathsim_tests)