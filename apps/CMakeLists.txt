# main

add_executable(main 
    main.cpp
    imgui_impl_glfw.cpp
    imgui_impl_opengl3.cpp
    )

target_include_directories(main PUBLIC
    "${CMAKE_SOURCE_DIR}/include/"
)

target_link_libraries(main 
    glfw
    ImGui
    ImPlot
    glew
    OpenGL::GL
    modules
    HybridAStar
)