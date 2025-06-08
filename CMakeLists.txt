cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMapGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# PCLを探す (コンポーネント指定なし、または基本的なもののみ)
find_package(PCL REQUIRED)
# find_package(PCL REQUIRED COMPONENTS io) # ioコンポーネントだけを指定するのも試す価値あり

# yaml-cppを探す
find_package(yaml-cpp REQUIRED)

# PCLのインクルードディレクトリとライブラリ定義を追加 (find_package(PCL) がこれらを適切に設定することを期待)
include_directories(${PCL_INCLUDE_DIRS})
# add_definitions(${PCL_DEFINITIONS}) # 通常は不要

add_executable(occupancy_grid_map_generator
    src/main.cpp
    src/file_utils.cpp
    src/config_loader.cpp
    src/map_parameters.cpp
    src/map_io.cpp
)

target_link_libraries(occupancy_grid_map_generator
    PRIVATE
        ${PCL_LIBRARIES} # PCL::common や PCL::io の代わりに ${PCL_LIBRARIES} を使用
        yaml-cpp
)