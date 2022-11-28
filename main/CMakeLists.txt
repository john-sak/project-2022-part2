add_executable(to_polygon main.cpp)

target_link_libraries(to_polygon PRIVATE CGAL::CGAL)
target_link_libraries(to_polygon PRIVATE arguments)
target_link_libraries(to_polygon PRIVATE polyline)
