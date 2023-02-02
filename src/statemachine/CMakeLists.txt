set(HEADERS
    lib.hpp
)

set(SOURCES
    lib.cpp
)

add_library(StateMachine SHARED
    ${HEADERS}
    ${SOURCES}
)

# target_include_directories(StateMachine PUBLIC
#     "${CMAKE_SOURCE_DIR}/include/"
#     "${CMAKE_CURRENT_SOURCE_DIR}/"
# )