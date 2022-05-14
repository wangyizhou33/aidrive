set(HEADERS
    HybridAStar.hpp
    ReedsShepp.hpp
    Coord2d.hpp
    CollisionGrid.hpp
    DStarLite.hpp
)

set(SOURCES
    HybridAStar.cpp
    ReedsShepp.cpp
    CollisionGrid.cpp
    DStarLite.cpp
)

add_library(HybridAStar SHARED
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(HybridAStar PUBLIC
    "${CMAKE_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/"
)