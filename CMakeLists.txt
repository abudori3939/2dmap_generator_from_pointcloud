cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMapGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# PCLを探す
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}) # PCLのヘッダファイルを見つけるために必要

# yaml-cppを探す
find_package(yaml-cpp REQUIRED)

add_executable(occupancy_grid_map_generator
    src/main.cpp
    src/file_utils.cpp
    src/config_loader.cpp
    src/map_parameters.cpp
    src/map_io.cpp
    src/point_cloud_processor.cpp # Added point_cloud_processor.cpp
)

target_link_libraries(occupancy_grid_map_generator
    PRIVATE
        ${PCL_LIBRARIES} # PCLライブラリ群をリンク
        yaml-cpp         # yaml-cppライブラリをリンク
)
