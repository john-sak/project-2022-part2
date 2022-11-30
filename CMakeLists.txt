cmake_minimum_required(VERSION 3.12)
project(Project2022 VERSION 1.0.0)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif()

add_subdirectory(lib/arguments)
add_subdirectory(lib/polyline)
add_subdirectory(lib/optimization)
add_subdirectory(main)
