#include <iostream>
// #include <string> // Included via iostream or pipeline.h
// #include <vector> // Included via pipeline.h
// <filesystem> and <algorithm> moved to pipeline.cpp

// PCL includes (most moved to pipeline.cpp)
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>

// Custom module includes (most are now included via pipeline.h or used within pipeline.cpp)
// #include "file_utils.h"
// #include "config_loader.h"
// #include "map_parameters.h"
// #include "map_io.h"
#include "point_cloud_processor.h" // Still needed for proc::PointCloudProcessor instantiation
#include "pipeline.h" // Add this include

// PCL Visualization includes moved to pipeline.cpp
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/common/io.h>
// #include <pcl/common/transforms.h>
// #include <pcl/search/kdtree.h>


// getFileExtension function removed from here as it's moved to pipeline.cpp

int main(int argc, char* argv[]) {
    pipeline::PipelineData data; // Instantiate PipelineData

    if (!pipeline::parseArguments(argc, argv, data.pointcloud_file_path, data.config_file_path)) {
        return 1; // Or some error code
    }
    std::cout << "点群ファイルパス: " << data.pointcloud_file_path << std::endl;
    std::cout << "設定ファイルパス: " << data.config_file_path << std::endl;

    // 2. 設定読み込み (Configuration Loading)
    if (!pipeline::loadConfiguration(data.config_file_path, data.app_config)) {
        return 1; // Or some error code
    }

    // 3. 点群読み込み (Point Cloud Loading)
    if (!pipeline::loadPointCloud(data.pointcloud_file_path, data.raw_cloud)) {
        return 1; // Or some error code
    }

    // 3.5 点群処理 (Point Cloud Processing)
    proc::PointCloudProcessor processor; // Instantiate processor
    if (!pipeline::processPointCloud(data, processor)) {
        std::cerr << "エラー: 点群処理中に致命的なエラーが発生しました。プログラムを終了します。" << std::endl;
        return 1; // Or some error code
    }
    // The large block of point cloud processing code has been moved to pipeline::processPointCloud

    // Call generateOccupancyGrid
    if (!pipeline::generateOccupancyGrid(data)) {
        std::cerr << "エラー: 占有格子地図の生成に失敗しました。プログラムを終了します。" << std::endl;
        return 1; // Or some error code
    }

    // map_params and occupancy_grid are now populated within data by generateOccupancyGrid

    // Call saveMap
    if (!pipeline::saveMap(data)) { // Assumes default output directory "output"
        std::cerr << "エラー: 地図ファイルの保存に失敗しました。プログラムを終了します。" << std::endl;
        return 1; // Or some error code
    }

    std::cout << "プログラムは正常に終了しました。" << std::endl;
    return 0;
}
