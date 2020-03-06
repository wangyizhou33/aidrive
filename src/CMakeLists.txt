# library
set(HEADERS
    model/Model.hpp
)

set(SOURCES
    model/Model.cpp
)

add_library(modules SHARED
    ${HEADERS}
    ${SOURCES}
)
target_include_directories(modules PUBLIC
    "${CMAKE_SOURCE_DIR}/include/"
    "${BOOST_INCLUDE_DIR}"
)

# main
add_executable(main main.cpp)
target_link_libraries(main 
    glfw
    OpenGL::GL
)
