cmake_minimum_required(VERSION 3.10)

if(TARGET gtest) # To avoid diamond dependencies; may not be necessary depending on you project.
    message("gtest target existing !")
    return()
endif()

set(INSTALL_GTEST ON)
add_subdirectory("googletest")