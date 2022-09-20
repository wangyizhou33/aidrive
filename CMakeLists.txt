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
# set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/3rdparty/ceres/ceres-solver/build/install/lib/cmake/)
find_package(Ceres REQUIRED)
# add_subdirectory("3rdparty/ceres")
# TODO take advantage of EXPORT_BUILD_DIR option

# glfw
add_subdirectory("3rdparty/glfw/")
find_package(OpenGL REQUIRED)

# boost
find_package(Boost REQUIRED)

# glew
find_package(GLEW REQUIRED) # apt-get install libglfw-dev

# imgui
add_subdirectory("3rdparty/imgui/")

# protobuf
# TODO include protobuf as 3rdparty
# right now sudo apt install libprotobuf-dev protobuf-compiler
# Find required protobuf package
set(protobuf_DIR ${CMAKE_SOURCE_DIR}/cmake)
find_package(protobuf CONFIG REQUIRED)

if(protobuf_VERBOSE)
  message(STATUS "Using Protocol Buffers ${Protobuf_VERSION}")
endif()

# cereal serialization
add_subdirectory("3rdparty/cereal")

# gcov 
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(CodeCoverage)
APPEND_COVERAGE_COMPILER_FLAGS()

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
        ${PROJECT_SOURCE_DIR}/include/aidrive/*pp
        ${PROJECT_SOURCE_DIR}/src/model/*pp
        ${PROJECT_SOURCE_DIR}/src/render/*pp
        ${PROJECT_SOURCE_DIR}/src/control/*pp
        ${PROJECT_SOURCE_DIR}/src/planner/*pp
        ${PROJECT_SOURCE_DIR}/tests/*pp
        COMMENT "Auto formatting of all source files"
    )
ENDIF()

# doxygen
find_package(Doxygen REQUIRED)
IF(DOXYGEN_FOUND)

    set(DOXYGEN_EXCLUDE_PATTERNS
        */.git/*
        */3rdparty/*)

    doxygen_add_docs(
        doc
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/Doxyfile
        COMMENT "Generate man pages"
    )
ENDIF()

if(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    setup_target_for_coverage_lcov(
            NAME AidriveCoverage_loc
            EXECUTABLE Test
            DEPENDENCIES Test modules
    )

    setup_target_for_coverage_gcovr_xml(
            NAME AidriveCoverage_xml
            EXECUTABLE Test
            DEPENDENCIES Test modules
    )

    setup_target_for_coverage_gcovr_html(
            NAME AidriveCoverage_html
            EXECUTABLE Test
            DEPENDENCIES Test modules
    )
endif()