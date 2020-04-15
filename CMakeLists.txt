cmake_minimum_required(VERSION 3.10)

project(aidrive)

set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -fPIC --std=c++17")

# eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("${EIGEN3_INCLUDE_DIR}")

# gtest
add_subdirectory("3rdparty/gtest")
enable_testing()

# ceres
add_subdirectory("3rdparty/ceres")

# glfw
add_subdirectory("3rdparty/glfw/")
find_package(OpenGL REQUIRED)

# boost
find_package(Boost REQUIRED)

# glew
find_package(GLEW REQUIRED) # apt-get install libglfw-dev

# imgui
add_subdirectory("3rdparty/imgui/")

# library
add_subdirectory("src/")

# tests
add_subdirectory("tests/")

# app
add_subdirectory("apps/")

# clang-format
# TODO need a better way to format recursively in subdirectories and define exclude directories
FIND_PROGRAM(CLANG_FORMAT "clang-format")
IF(CLANG_FORMAT)
    ADD_CUSTOM_TARGET(
        format
        COMMAND
        ${CLANG_FORMAT}
        -i
        -style=file
        ${PROJECT_SOURCE_DIR}/apps/*pp
        ${PROJECT_SOURCE_DIR}/src/model/*pp
        ${PROJECT_SOURCE_DIR}/src/render/*pp
        ${PROJECT_SOURCE_DIR}/tests/*pp
        COMMENT "Auto formatting of all source files"
    )
ENDIF()
