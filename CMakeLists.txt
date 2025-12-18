cmake_minimum_required(VERSION 3.11)
project(unilidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(pybind11 REQUIRED)

message(STATUS "--------------------------------------------")
message(STATUS "Building project..")
message(STATUS "Building platform: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Building type: ${CMAKE_BUILD_TYPE}")

# build lidar
pybind11_add_module(lidar cpplib/lidar.cpp)

target_compile_options(lidar PRIVATE
    $ENV{CXXFLAGS}
    $<$<CONFIG:Debug>:-O0 -Wall -g2 -ggdb>
    $<$<CONFIG:Release>:-O3 -Wall -DNDEBUG>
)

target_include_directories(lidar PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_directories(lidar PRIVATE ${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
target_link_libraries(lidar PRIVATE libunilidar_sdk2.a)

# build djset
pybind11_add_module(djset cpplib/djset.cpp)

target_compile_options(djset PRIVATE
    $ENV{CXXFLAGS}
    $<$<CONFIG:Debug>:-O0 -Wall -g2 -ggdb>
    $<$<CONFIG:Release>:-O3 -Wall -DNDEBUG>
)