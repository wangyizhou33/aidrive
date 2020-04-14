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
