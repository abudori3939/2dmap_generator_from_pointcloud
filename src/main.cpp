#include <iostream>
#include <string>
#include <vector>
#include <filesystem> // For path operations like getting extension (C++17)
#include <algorithm>  // For std::transform to use with ::tolower

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h> // Added for PLY support
#include <pcl/point_types.h>

// Custom module includes
#include "file_utils.h"
#include "config_loader.h"
#include "map_parameters.h"
#include "map_io.h"

// Helper function to get file extension (lowercase)
std::string getFileExtension(const std::string& filepath) {
    try {
        std::filesystem::path p(filepath);
        std::string ext = p.extension().string();
        // Convert extension to lowercase
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return ext;
    } catch (const std::filesystem::filesystem_error& e) {
        // This catch block might not be strictly necessary for basic path operations
        // if the path string itself is valid, but good for robustness.
        std::cerr << "ファイルパスエラー (getFileExtension): " << e.what() << std::endl;
        return ""; // Return empty string on error
    }
}


int main(int argc, char* argv[]) {
    // 1. 引数解析 (Argument Parsing)
    if (argc != 3) {
        std::cerr << "使用方法: " << argv[0] << " <PCD/PLYファイルパス> <設定ファイルパス>" << std::endl;
        std::cerr << "例: " << argv[0] << " data/input.pcd config/config.yaml" << std::endl;
        std::cerr << "   " << argv[0] << " data/input.ply config/config.yaml" << std::endl;
        return 1;
    }
    std::string pointcloud_file_path = argv[1]; // Renamed for clarity
    std::string config_file_path = argv[2];
    std::cout << "点群ファイルパス: " << pointcloud_file_path << std::endl; // Updated label
    std::cout << "設定ファイルパス: " << config_file_path << std::endl;

    // 2. 設定読み込み (Configuration Loading)
    map_config::Config app_config;
    std::cout << "設定ファイルを読み込み中: " << config_file_path << std::endl;
    if (!map_config::loadConfig(config_file_path, app_config)) {
        std::cerr << "エラー: 設定ファイルの読み込みに失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "読み込まれた設定値:" << std::endl;
    std::cout << "  地図解像度: " << app_config.map_resolution << " [m/pixel]" << std::endl;
    std::cout << "  ロボットの高さ: " << app_config.robot_height << " [m]" << std::endl;
    std::cout << "  法線推定半径: " << app_config.normal_estimation_radius << " [m]" << std::endl;
    std::cout << "  地面法線Z閾値: " << app_config.ground_normal_z_threshold << std::endl;
    std::cout << "  ブロックサイズ: " << app_config.block_size << " [m]" << std::endl;


    // 3. 点群読み込み (Point Cloud Loading)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "点群ファイルを読み込み中: " << pointcloud_file_path << std::endl; // Updated label

    std::string extension = getFileExtension(pointcloud_file_path);

    int load_status = -1; // PCL loaders typically return 0 on success, -1 on failure

    if (extension == ".pcd") {
        std::cout << "PCDファイルとして読み込みます。" << std::endl;
        load_status = pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_file_path, *cloud);
    } else if (extension == ".ply") {
        std::cout << "PLYファイルとして読み込みます。" << std::endl;
        load_status = pcl::io::loadPLYFile<pcl::PointXYZ>(pointcloud_file_path, *cloud);
    } else {
        std::cerr << "エラー: 未サポートのファイル拡張子です: '" << extension << "'" << std::endl;
        std::cerr << "PCD (.pcd) または PLY (.ply) ファイルを指定してください。" << std::endl;
        return -1;
    }

    if (load_status == -1) {
        std::cerr << "エラー: 点群ファイルの読み込みに失敗しました: " << pointcloud_file_path << std::endl;
        // PCL_ERROR might be redundant if PCL loaders log verbosely, but can be kept for emphasis.
        // PCL_ERROR("Couldn't read file %s \n", pointcloud_file_path.c_str());
        return -1;
    }

    // 点群が実際にデータを保持しているか確認 (特にPLYローダーが0点を返す場合があるため)
    if (cloud->points.empty()) {
         std::cerr << "エラー: 点群ファイルは読み込まれましたが、データが空です: " << pointcloud_file_path << std::endl;
         return -1;
    }

    std::cout << "点群ファイルから " << cloud->width * cloud->height << " 点のデータを読み込みました。" << std::endl;


    // 4. 地図パラメータ計算 (Map Parameter Calculation)
    map_params_util::MapParameters map_params;
    // The check for cloud->points.empty() is still in calculateMapParameters for robustness,
    // though it's also checked above after loading.
    if (!map_params_util::calculateMapParameters(cloud, app_config.map_resolution, map_params)) {
        std::cerr << "エラー: 地図パラメータの計算に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "計算された地図パラメータ:" << std::endl;
    std::cout << "  原点X: " << map_params.origin_x << " [m], Y: " << map_params.origin_y << " [m]" << std::endl;
    std::cout << "  幅: " << map_params.width_pixels << " [ピクセル], 高さ: " << map_params.height_pixels << " [ピクセル]" << std::endl;
    std::cout << "  解像度: " << map_params.resolution << " [m/pixel]" << std::endl;

    // 5. 占有格子地図の初期化 (Occupancy Grid Initialization)
    std::vector<int8_t> occupancy_grid;
    try {
        occupancy_grid.resize(
            static_cast<size_t>(map_params.width_pixels) * map_params.height_pixels,
            map_io_util::GRID_VALUE_UNKNOWN
        );
    } catch (const std::bad_alloc& e) {
        std::cerr << "エラー: 地図データ構造のためのメモリ確保に失敗 ("
                  << static_cast<size_t>(map_params.width_pixels) * map_params.height_pixels * sizeof(int8_t)
                  << " bytes requested): " << e.what() << std::endl;
        return 1;
    }
    std::cout << "占有格子地図を初期化。サイズ: " << map_params.width_pixels << "x" << map_params.height_pixels
              << ", 初期値: " << static_cast<int>(map_io_util::GRID_VALUE_UNKNOWN) << std::endl;

    // 6. 地図出力 (Map Output)
    std::string output_dir = "output";
    utils::ensureDirectoryExists(output_dir);

    std::string pgm_file_basename = "map.pgm";
    std::string pgm_file_path = output_dir + "/" + pgm_file_basename;
    std::string yaml_file_path = output_dir + "/map_metadata.yaml";

    if (!map_io_util::saveMapAsPGM(occupancy_grid, map_params, pgm_file_path)) {
        std::cerr << "エラー: PGM地図の保存に失敗しました。" << std::endl;
        return 1;
    }
    if (!map_io_util::saveMapMetadataYAML(map_params, pgm_file_basename, yaml_file_path)) {
        std::cerr << "エラー: 地図メタデータYAMLの保存に失敗しました。" << std::endl;
        return 1;
    }

    std::cout << "地図が " << output_dir << " に正常に出力されました。" << std::endl;
    std::cout << "プログラムは正常に終了しました。" << std::endl;
    return 0;
}
