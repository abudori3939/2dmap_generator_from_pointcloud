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
#include "point_cloud_processor.h" // 新しいヘッダ

// PCL Visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h> // For copyPointCloud
// #include <pcl/search/kdtree.h> // Already included via point_cloud_processor.h or other PCL headers

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

    // 3.5 点群処理 (Point Cloud Processing) - 法線推定など
    std::cout << "\n--- 点群処理開始 ---" << std::endl;
    proc::PointCloudProcessor processor;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    if (!processor.estimateNormals(cloud, static_cast<float>(app_config.normal_estimation_radius), normals)) {
        std::cerr << "エラー: 法線推定に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    // 現時点では法線は使用されていませんが、推定は行われました。
    // 将来のステップで地面除去などに使用します。
    std::cout << "法線推定が正常に完了しました。法線数: " << normals->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>());
    if (!processor.extractGroundCandidates(cloud, normals, static_cast<float>(app_config.ground_normal_z_threshold), ground_candidates)) {
        std::cerr << "エラー: 地面候補点の抽出に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "地面候補点の抽出成功。候補点数: " << ground_candidates->size() << std::endl;
    if (ground_candidates->empty()) {
        std::cout << "注意: 地面候補点が見つかりませんでした。パラメータまたは入力データを確認してください。" << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr main_ground_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    // クラスタリングのパラメータを設定 (これらの値は設定ファイルからも読み込めるようにすると良い)
    // app_config から読み込むか、適切なデフォルト値を設定
    float cluster_tolerance = 2.0f * static_cast<float>(app_config.map_resolution); // 例: 解像度の2倍
    int min_cluster_size = 50;  // 例: 最小50点
    int max_cluster_size = 25000; // 例: 最大25000点

    if (!processor.extractMainGroundCluster(ground_candidates, cluster_tolerance, min_cluster_size, max_cluster_size, main_ground_cluster)) {
        std::cerr << "エラー: 主地面クラスタの特定に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "主地面クラスタの特定成功。クラスタ点数: " << main_ground_cluster->size() << std::endl;
    if (main_ground_cluster->empty()) {
        if (ground_candidates->empty()) {
            std::cout << "情報: 地面候補点がなかったため、主地面クラスタもありません。" << std::endl;
        } else {
            std::cout << "注意: 地面候補点はありましたが、条件に合う主地面クラスタは見つかりませんでした。" << std::endl;
        }
    } else {
        std::cerr << "エラー: 主地面クラスタの特定に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());

    // 平面フィッティングのパラメータ (設定ファイルから読み込めるようにすると良い)
    float plane_distance_threshold = 0.02f; // 例: 2cm (app_config.map_resolution * 0.4 なども検討可)

    if (processor.fitGlobalGroundPlane(main_ground_cluster, plane_distance_threshold, plane_coefficients, plane_inliers)) {
        std::cout << "グローバル地面平面のフィッティング成功。" << std::endl;
        // 平面係数とインライア数は fitGlobalGroundPlane 内で詳細に出力される
        if (plane_coefficients->values.empty()) { // 追加の堅牢性チェック
             std::cout << "注意: 平面フィッティングは成功と報告されましたが、係数が空です。" << std::endl;
        }
    } else {
        std::cout << "情報: グローバル地面平面のフィッティングに失敗、または平面が見つかりませんでした。" << std::endl;
        // 地面平面が見つからない場合、この後の処理で問題が起きる可能性がある。
        // 例えば、ロボットの高さに基づいて点をフィルタリングする場合など。
        // ここでは、警告を出しつつ処理を続行する。必要に応じて return 1; で終了させる。
    }
    // plane_coefficients と plane_inliers は次のステップ (占有グリッド生成) で使用されます。

    // 3.6 地面候補点の可視化 (Visualization of Ground Candidates)
    std::cout << "\n--- 地面候補点の可視化開始 ---" << std::endl;
    if (!cloud->points.empty()) { // Ensure original cloud is not empty for visualization
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *colored_cloud); // 元の点群をコピー

        uint8_t r_other = 0, g_other = 255, b_other = 0; // 緑色 (非地面候補)
        uint8_t r_ground = 255, g_ground = 0, b_ground = 0; // 赤色 (地面候補)

        // まず全点をデフォルト色（緑）に設定
        for (auto& point : colored_cloud->points) {
            point.r = r_other;
            point.g = g_other;
            point.b = b_other;
        }

        if (ground_candidates && !ground_candidates->points.empty()) {
            // 地面候補点が存在する場合、それらを赤色にマークする
            // KdTreeを使って元の点群内の地面候補点に対応する点を見つける
            // (注意: ground_candidates は cloud のサブセットなので、インデックスは直接対応しない)
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_rgb(new pcl::search::KdTree<pcl::PointXYZRGB>());
            tree_rgb->setInputCloud(colored_cloud);

            for (const auto& ground_point : ground_candidates->points) {
                pcl::PointXYZRGB search_point_rgb;
                search_point_rgb.x = ground_point.x;
                search_point_rgb.y = ground_point.y;
                search_point_rgb.z = ground_point.z;
                // RGB値は何でもよいが、XYZが重要

                std::vector<int> point_idx_nkns(1);
                std::vector<float> point_squared_distance(1);

                if (tree_rgb->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                    if (point_squared_distance[0] < 0.00001f) { // 小さな許容誤差
                        colored_cloud->points[point_idx_nkns[0]].r = r_ground;
                        colored_cloud->points[point_idx_nkns[0]].g = g_ground;
                        colored_cloud->points[point_idx_nkns[0]].b = b_ground;
                    }
                }
            }
            std::cout << "地面候補点の色付け処理完了。" << std::endl;
        } else {
            std::cout << "地面候補点がないため、色付け処理はスキップされました（全点が緑色）。" << std::endl;
        }

        // PCLViewerの初期化と表示
        try {
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer - Ground Candidates"));
            viewer->setBackgroundColor(0.1, 0.1, 0.1); // 背景を濃い灰色に

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "ground_visualization_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ground_visualization_cloud");
            // viewer->addCoordinateSystem(1.0); // 座標軸の表示
            viewer->initCameraParameters(); // カメラパラメータを初期化

            std::cout << "PCLViewerを表示します。ウィンドウを閉じて続行してください..." << std::endl;
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
            }
            viewer->close();
            std::cout << "PCLViewerを閉じました。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "PCLViewerの初期化または表示中にエラーが発生しました: " << e.what() << std::endl;
            std::cerr << "ヘッドレス環境ではPCLViewerを実行できない可能性があります。処理は続行します。" << std::endl;
        }
    } else {
        std::cout << "元の点群が空のため、可視化をスキップします。" << std::endl;
    }
    std::cout << "--- 点群処理終了 ---\n" << std::endl;

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
