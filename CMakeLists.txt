cmake_minimum_required(VERSION 3.10)

project(aidrive)

add_subdirectory("3rdparty/ceres")

# eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("${EIGEN_INCLUDE_DIR}")

# gtest
add_subdirectory("3rdparty/gtest")
enable_testing()

# glfw
add_subdirectory("3rdparty/glfw/")
find_package(OpenGL REQUIRED)

# boost
find_package(Boost REQUIRED)

# glew
find_package(GLEW REQUIRED) # apt-get install libglfw-dev

# imgui
add_subdirectory("3rdparty/imgui/")

# build an imgui example
add_executable(example_glfw_opengl3
    3rdparty/imgui/imgui/examples/example_glfw_opengl3/main.cpp)

target_include_directories(example_glfw_opengl3 PUBLIC
    ${IMGUI_HEADERS}
    "3rdparty/imgui/imgui/examples/")

target_link_libraries(example_glfw_opengl3 PUBLIC
    ImGui
    glfw
    GLEW::GLEW
    OpenGL::GL
)

# main
add_executable(main src/main.cpp)
target_link_libraries(main 
    glfw
    OpenGL::GL
)
