cmake_minimum_required(VERSION 3.25)
project(PathSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(PathSim src/main.cpp src/core/Grid.cpp)

target_link_libraries(PathSim
    PRIVATE
        imgui::imgui
        glfw
        OpenGL::GL
)