include(GoogleTest)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${CEREAL_INCLUDE_DIR})

add_executable(Test
    TestModel.cpp
    TestCeres.cpp
    TestGmock.cpp
    TestCereal.cpp
    main.cpp
)

target_link_libraries(Test
    gtest
    gmock
    modules
    ${CERES_LIBRARIES}
)

gtest_add_tests(Test "" AUTO)