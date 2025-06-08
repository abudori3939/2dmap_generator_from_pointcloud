cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMapGenerator)

# Update CMAKE_CXX_STANDARD from 14 to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True) # Ensure C++17 is used, error if not available

add_executable(occupancy_grid_map_generator src/main.cpp)

# It's good practice to explicitly state the C++ standard for the target as well,
# though set(CMAKE_CXX_STANDARD 17) often suffices.
# target_compile_features(occupancy_grid_map_generator PRIVATE cxx_std_17) # Alternative
