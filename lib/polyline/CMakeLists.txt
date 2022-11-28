add_library(
    polyline
    polyline.hpp
    ../../src/polyline.cpp
)

target_include_directories(polyline PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
