# library
set(HEADERS
    model/Model.hpp

    render/Render.hpp
)

set(SOURCES
    model/Model.cpp

    render/Render.cpp
)

add_library(modules SHARED
    ${HEADERS}
    ${SOURCES}
)
target_include_directories(modules PUBLIC
    "${CMAKE_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/model/"
    "${CMAKE_CURRENT_SOURCE_DIR}/render/"
)

target_link_libraries(modules
    ImGui)
