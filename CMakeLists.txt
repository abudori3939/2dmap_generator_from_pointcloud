cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMapGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# PCLを探す
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}) # PCLのヘッダファイルを見つけるために必要

# yaml-cppを探す
find_package(yaml-cpp REQUIRED)

# OpenCVを探す
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(occupancy_grid_map_generator
    src/main.cpp
    src/file_utils.cpp
    src/config_loader.cpp
    src/map_parameters.cpp
    src/map_io.cpp
    src/point_cloud_processor.cpp
    src/pipeline.cpp
    src/map_processor.cpp
)

target_link_libraries(occupancy_grid_map_generator
    PRIVATE
        ${PCL_LIBRARIES} # PCLライブラリ群をリンク
        yaml-cpp         # yaml-cppライブラリをリンク
        ${OpenCV_LIBS}   # OpenCVライブラリをリンク
)

# Google Test Setup
enable_testing()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test executable
add_executable(pcd_to_gridmap_tests
    tests/test_map_processor.cpp
    src/map_processor.cpp
    # src/config_loader.cpp # Not strictly needed if tests pass kernel sizes explicitly
)
target_include_directories(pcd_to_gridmap_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pcd_to_gridmap_tests PRIVATE
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(pcd_to_gridmap_tests)
