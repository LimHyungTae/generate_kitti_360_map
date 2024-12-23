cmake_minimum_required(VERSION 3.10)
project(generate_kitti_360_map)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    include
    src
)

add_executable(main src/main.cc src/ply_io.cc src/tinyply.cpp)
target_include_directories(main
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(main
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
