cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMapGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

add_executable(occupancy_grid_map_generator
    src/main.cpp
    src/file_utils.cpp
    src/config_loader.cpp
    src/map_parameters.cpp
    src/map_io.cpp # Added map_io.cpp
)

target_link_libraries(occupancy_grid_map_generator
    PRIVATE
        PCL::common
        PCL::io
        yaml-cpp
)
