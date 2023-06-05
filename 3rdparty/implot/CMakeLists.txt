set(root ${CMAKE_CURRENT_SOURCE_DIR}/implot)

set(IMPLOT_SOURCES
    "${root}/implot.cpp"
)

set(IMPLOT_HEADERS
    "${root}/implot.h"
)

add_library(
    ImPlot STATIC
    ${IMPLOT_SOURCES}
    ${IMPLOT_HEADERS}
)

target_include_directories(ImPlot PUBLIC
    "${root}/"
)
target_link_libraries(ImPlot 
    ImGui
)