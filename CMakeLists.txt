cmake_minimum_required(VERSION 3.10)

project(aidrive)

add_subdirectory("3rdparty/ceres")

# eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# gtest
add_subdirectory("3rdparty/gtest")
enable_testing()

# glfw
add_subdirectory("3rdparty/glfw/")
find_package(OpenGL REQUIRED)

# main
add_executable(main src/main.cpp)