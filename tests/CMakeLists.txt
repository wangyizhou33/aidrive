include(GoogleTest)

add_executable(TestModel
    TestModel.cpp
    main.cpp
)

target_link_libraries(TestModel
    gtest
    modules
)

gtest_add_tests(TestModel "" AUTO)