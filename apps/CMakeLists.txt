# main
add_executable(main 
    main.cpp
    imgui_impl_glfw.cpp
    imgui_impl_opengl3.cpp
    )

target_link_libraries(main 
    glfw
    ImGui
    GLEW::GLEW
    OpenGL::GL
    modules
)