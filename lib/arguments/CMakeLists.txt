add_library(
    arguments
    arguments.hpp
    ../../src/arguments.cpp
)

target_include_directories(arguments PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
