add_library(
    optimization
    optimization.hpp
    ../../src/optimization.cpp
)

target_include_directories(optimization PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
