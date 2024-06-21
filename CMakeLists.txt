cmake_minimum_required(VERSION 3.28)
project(ConstraintDelaunayTriangulation)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(CGAL CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 20)

# GEOMETRY
file(GLOB GEOMETRY "${PROJECT_SOURCE_DIR}/include/geometry/*.h" "${PROJECT_SOURCE_DIR}/src/geometry/*.cpp")
#NUMERIC
file(GLOB NUMERIC "${PROJECT_SOURCE_DIR}/include/numeric/*.h" "${PROJECT_SOURCE_DIR}/src/numeric/*.cpp")
#DELAUNAY
file(GLOB DELAUNAY "${PROJECT_SOURCE_DIR}/include/dt/*.h" "${PROJECT_SOURCE_DIR}/src/dt/*.cpp")
#UTILS
file(GLOB UTILS "${PROJECT_SOURCE_DIR}/include/utils/*.h" "${PROJECT_SOURCE_DIR}/src/utils/*.cpp")
add_executable(${PROJECT_NAME} main.cpp
                ${GEOMETRY}
                ${NUMERIC}
                ${DELAUNAY}
                ${UTILS}
                )

add_subdirectory(third_party/CDT CDT)
target_link_libraries(${PROJECT_NAME} PRIVATE CDT::CDT)

target_include_directories(${PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/include")
if(CGAL_Qt5_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC CGAL::CGAL_Basic_viewer)
else()
    message(STATUS "NOTICE: Several examples require Qt5 and will not be compiled.")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE CGAL::CGAL)