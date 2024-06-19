cmake_minimum_required(VERSION 3.28)
project(CDT)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(CGAL CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 20)

add_executable(CDT main.cpp)

if(CGAL_Qt5_FOUND)
    target_link_libraries(CDT PUBLIC CGAL::CGAL_Basic_viewer)
else()
    message(STATUS "NOTICE: Several examples require Qt5 and will not be compiled.")
endif()

target_link_libraries(CDT PRIVATE CGAL::CGAL)