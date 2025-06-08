# 3D Point Cloud to 2D Occupancy Grid Map Generator

This project generates a 2D occupancy grid map from 3D point cloud data.

## Installation

1.  Clone the repository.
2.  Run the installation script to install dependencies (PCL, Eigen):
    ```bash
    ./install.bash
    ```

## Building

1.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```
2.  Run CMake and build the project:
    ```bash
    cmake ..
    make
    ```
The executable `occupancy_grid_map_generator` will be created in the `build` directory.
