add_executable(optimal_polygon main.cpp)

target_link_libraries(optimal_polygon PRIVATE CGAL::CGAL)
target_link_libraries(optimal_polygon PRIVATE arguments)
target_link_libraries(optimal_polygon PRIVATE polyline)
target_link_libraries(optimal_polygon PRIVATE optimization)
