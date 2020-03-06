cmake_minimum_required(VERSION 3.10)

set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CMAKE_BUILD_TYPE Release)
add_subdirectory("ceres-solver")