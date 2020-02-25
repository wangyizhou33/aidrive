cmake_minimum_required(VERSION 3.10)

if(TARGET gtest) # To avoid diamond dependencies; may not be necessary depending on you project.
    return()
endif()

add_subdirectory("googletest")