include(GoogleTest)

include_directories(${CERES_INCLUDE_DIRS})

add_executable(Test
    TestModel.cpp
    TestCeres.cpp
    main.cpp
)

target_link_libraries(Test
    gtest
    modules
    ${CERES_LIBRARIES}
)

gtest_add_tests(Test "" AUTO)