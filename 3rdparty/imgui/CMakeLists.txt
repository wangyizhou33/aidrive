cmake_minimum_required (VERSION 3.10)

# dependency
# apt-get install libglew-dev

project(imgui)

set(inst_src_dir include/imgui_src)

set(root ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(phdrs ${root}/imgui.h)
set(hdrs ${root}/stb_rect_pack.h ${root}/stb_textedit.h ${root}/stb_truetype.h)
set(srcs ${root}/imgui.cpp)
set(inst_srcs ${inst_src_dir}/imgui.cpp)

set(IMGUI_SOURCES
    "${root}/imgui.cpp"
    "${root}/imgui_demo.cpp"
    "${root}/imgui_draw.cpp"
    "${root}/imgui_widgets.cpp"
    "${root}/examples/imgui_impl_glfw.cpp"
    "${root}/examples/imgui_impl_opengl3.cpp"
)

set(IMGUI_HEADERS
    "${root}/imconfig.h"
    "${root}/imgui.h"
    "${root}/imgui_internal.h"
    "${root}/imstb_rectpack.h"
    "${root}/imstb_textedit.h"
    "${root}/imstb_truetype.h"
    "${root}/examples/imgui_impl_glfw.h"
    "${root}/examples/imgui_impl_opengl3.h"
)

add_library(
    ImGui STATIC
    ${IMGUI_SOURCES}
    ${IMGUI_HEADERS}
)

target_include_directories(ImGui PUBLIC "${root}")

# build an imgui example
add_executable(example_glfw_opengl3
    ${root}/examples/example_glfw_opengl3/main.cpp)

target_include_directories(example_glfw_opengl3 PUBLIC
    ${IMGUI_HEADERS}
    "${root}/examples/")

target_link_libraries(example_glfw_opengl3 PUBLIC
    ImGui
    glfw
    GLEW::GLEW
    OpenGL::GL
)