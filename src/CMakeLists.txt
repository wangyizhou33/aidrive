# library

# protobuf definition
set(PROTOS 
    addressbook.proto
)
set(PROTO_SRCS "")
foreach(proto_file ${PROTOS})
  get_filename_component(proto_file_abs ${proto_file} ABSOLUTE)
  get_filename_component(basename ${proto_file} NAME_WE)
  set(generated_files ${basename}.pb.cc ${basename}.pb.h)
  list(APPEND PROTO_SRCS ${generated_files})

  add_custom_command(
    OUTPUT ${generated_files}
    COMMAND protoc
    ARGS --cpp_out ${CMAKE_CURRENT_BINARY_DIR} -I ${CMAKE_CURRENT_SOURCE_DIR} ${proto_file_abs}
    COMMENT "Generating ${generated_files} from ${proto_file}"
    VERBATIM
  )
endforeach()

set(HEADERS
    model/Model.hpp

    render/Render.hpp

    control/Control.hpp

    planner/SpeedOpt.hpp

    estimator/histogram.hpp
    estimator/holo_histogram.hpp
)

set(SOURCES
    ${PROTO_SRCS}

    model/Model.cpp

    render/Render.cpp

    control/Control.cpp

    planner/SpeedOpt.cpp

    estimator/histogram.cpp
    estimator/holo_histogram.cpp
)

include_directories(${CERES_INCLUDE_DIRS})

add_library(modules SHARED
    ${HEADERS}
    ${SOURCES}
)

target_include_directories(modules PUBLIC
    "${CMAKE_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/"
    "${CMAKE_CURRENT_BINARY_DIR}/"
)

target_link_libraries(modules
    ImGui
    ${PROTOBUF_LIBRARIES}
    ${CERES_LIBRARIES}
)

add_subdirectory(newton)
add_subdirectory(unstructure)