#include <iostream>
#include <string>
#include <vector>
#include <filesystem> // For path operations like getting extension (C++17)
#include <algorithm>  // For std::transform to use with ::tolower

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Custom module includes
#include "file_utils.h"
#include "config_loader.h"
#include "map_parameters.h"
#include "map_io.h"
#include "point_cloud_processor.h"

// PCL Visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h> // For copyPointCloud
#include <pcl/search/kdtree.h> // For KdTree used in visualization coloring (restored)

// Helper function to get file extension (lowercase)
std::string getFileExtension(const std::string& filepath) {
    try {
        std::filesystem::path p(filepath);
        std::string ext = p.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return ext;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "ファイルパスエラー (getFileExtension): " << e.what() << std::endl;
        return "";
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
    std::string pointcloud_file_path = argv[1];
    std::string config_file_path = argv[2];
    std::cout << "点群ファイルパス: " << pointcloud_file_path << std::endl;
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
    std::cout << "  最小クラスタ点数 (min_cluster_size): " << app_config.min_cluster_size << std::endl;
    std::cout << "  最大クラスタ点数 (max_cluster_size): " << app_config.max_cluster_size << std::endl;

    // 3. 点群読み込み (Point Cloud Loading)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "点群ファイルを読み込み中: " << pointcloud_file_path << std::endl;
    std::string extension = getFileExtension(pointcloud_file_path);
    int load_status = -1;

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
        return -1;
    }
    if (cloud->points.empty()) {
         std::cerr << "エラー: 点群ファイルは読み込まれましたが、データが空です: " << pointcloud_file_path << std::endl;
         return -1;
    }
    std::cout << "点群ファイルから " << cloud->width * cloud->height << " 点のデータを読み込みました。" << std::endl;

    // 3.5 点群処理 (Point Cloud Processing)
    std::cout << "\n--- 点群処理開始 ---" << std::endl;
    proc::PointCloudProcessor processor;

    //   3.5.1 法線推定 (Normal Estimation)
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (!processor.estimateNormals(cloud, static_cast<float>(app_config.normal_estimation_radius), normals)) {
        std::cerr << "エラー: 法線推定に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "法線推定が正常に完了しました。法線数: " << normals->size() << std::endl;

    //   3.5.2 地面候補点の抽出 (Ground Candidate Extraction)
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>());
    if (!processor.extractGroundCandidates(cloud, normals, static_cast<float>(app_config.ground_normal_z_threshold), ground_candidates)) {
        std::cerr << "エラー: 地面候補点の抽出に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "地面候補点の抽出成功。候補点数: " << ground_candidates->size() << std::endl;

    //   3.5.3 地面候補点の可視化 (Visualization of Ground Candidates) - 元に戻したブロック
    std::cout << "\n--- 地面候補点の可視化開始 ---" << std::endl;
    if (!cloud->points.empty()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *colored_cloud);

        uint8_t r_other = 0, g_other = 255, b_other = 0;
        uint8_t r_ground = 255, g_ground = 0, b_ground = 0;

        for (auto& point : colored_cloud->points) {
            point.r = r_other; point.g = g_other; point.b = b_other;
        }

        if (ground_candidates && !ground_candidates->points.empty()) {
            std::cout << "地面候補点の色付け処理（KdTree）を開始します..." << std::endl;
            // KdTreeを使って元の点群内の地面候補点に対応する点を見つける
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_rgb(new pcl::search::KdTree<pcl::PointXYZRGB>());
            tree_rgb->setInputCloud(colored_cloud); // tree_rgb に colored_cloud を設定

            int colored_count = 0;
            for (const auto& ground_point : ground_candidates->points) {
                pcl::PointXYZRGB search_point_rgb; // KdTree検索用のPointXYZRGB点を作成
                search_point_rgb.x = ground_point.x;
                search_point_rgb.y = ground_point.y;
                search_point_rgb.z = ground_point.z;

                std::vector<int> point_idx_nkns(1); // 見つかった点のインデックスを格納するベクター
                std::vector<float> point_squared_distance(1); // 見つかった点との距離の二乗を格納するベクター

                if (tree_rgb->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                    if (point_squared_distance[0] < 0.00001f) { // 小さな許容誤差 (1e-5f)
                        colored_cloud->points[point_idx_nkns[0]].r = r_ground;
                        colored_cloud->points[point_idx_nkns[0]].g = g_ground;
                        colored_cloud->points[point_idx_nkns[0]].b = b_ground;
                        colored_count++;
                    }
                }
            }
            std::cout << "地面候補点の色付け処理（KdTree）完了。 " << colored_count << " 点が赤色にマークされました。" << std::endl;
            if (static_cast<size_t>(colored_count) != ground_candidates->points.size()) {
                std::cout << "警告: 全ての地面候補点 (" << ground_candidates->points.size()
                          << "点) がKdTreeで色付けされたわけではありません。許容誤差またはロジックを確認してください。" << std::endl;
            }
        } else {
            std::cout << "地面候補点がないため、色付け処理はスキップされました（全点が緑色）。" << std::endl;
        }

        try {
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer - Ground Candidates"));
            viewer->setBackgroundColor(0.1, 0.1, 0.1);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "ground_visualization_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ground_visualization_cloud");
            viewer->initCameraParameters();
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
    // Visualization ends here, before the check for empty ground_candidates for further processing

    if (ground_candidates->empty()) {
        std::cout << "注意: 地面候補点が見つかりませんでした。パラメータまたは入力データを確認してください。" << std::endl;
    }

    //   3.5.4 主地面クラスタの特定 (Main Ground Cluster Extraction)
    pcl::PointCloud<pcl::PointXYZ>::Ptr main_ground_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    // クラスタリングパラメータをapp_configから使用
    float cluster_tolerance = 2.0f * static_cast<float>(app_config.map_resolution); // 許容距離は解像度に応じて設定
    // min_cluster_size と max_cluster_size は app_config から直接使用

    if (!processor.extractMainGroundCluster(ground_candidates, cluster_tolerance, app_config.min_cluster_size, app_config.max_cluster_size, main_ground_cluster)) {
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
    }

    // ---- ここから新しい視覚化コード (主地面クラスタの視覚化) ----
    std::cout << "\n--- 主地面クラスタの視覚化開始 ---" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_viz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    uint8_t r_main_cluster = 255, g_main_cluster = 0, b_main_cluster = 0; // 赤色 (主地面クラスタ)
    uint8_t r_other_candidates = 0, g_other_candidates = 255, b_other_candidates = 0; // 緑色 (その他の地面候補)

    if (ground_candidates && !ground_candidates->points.empty()) {
        // まず、全ての地面候補点を緑色で cluster_viz_cloud にコピー
        // (この時点では main_ground_cluster の点は ground_candidates にも含まれているので、一旦全て緑になる)
        for (const auto& candidate_point : ground_candidates->points) {
            pcl::PointXYZRGB point_rgb;
            point_rgb.x = candidate_point.x;
            point_rgb.y = candidate_point.y;
            point_rgb.z = candidate_point.z;
            point_rgb.r = r_other_candidates;
            point_rgb.g = g_other_candidates;
            point_rgb.b = b_other_candidates;
            cluster_viz_cloud->points.push_back(point_rgb);
        }
        cluster_viz_cloud->width = cluster_viz_cloud->points.size();
        cluster_viz_cloud->height = 1;
        cluster_viz_cloud->is_dense = true;


        // 次に、主地面クラスタの点を赤色にマークする (KdTreeを使用)
        if (main_ground_cluster && !main_ground_cluster->points.empty()) {
            // cluster_viz_cloud には ground_candidates がコピーされているので、
            // main_ground_cluster の点と一致するものを cluster_viz_cloud の中で見つけて赤くする
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_cluster_viz(new pcl::search::KdTree<pcl::PointXYZRGB>());
            tree_cluster_viz->setInputCloud(cluster_viz_cloud);

            int re_colored_count = 0;
            for (const auto& main_cluster_point : main_ground_cluster->points) {
                pcl::PointXYZRGB search_point_rgb;
                search_point_rgb.x = main_cluster_point.x;
                search_point_rgb.y = main_cluster_point.y;
                search_point_rgb.z = main_cluster_point.z;

                std::vector<int> point_idx_nkns(1);
                std::vector<float> point_squared_distance(1);

                if (tree_cluster_viz->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                    if (point_squared_distance[0] < 0.00001f) { // 小さな許容誤差
                        cluster_viz_cloud->points[point_idx_nkns[0]].r = r_main_cluster;
                        cluster_viz_cloud->points[point_idx_nkns[0]].g = g_main_cluster;
                        cluster_viz_cloud->points[point_idx_nkns[0]].b = b_main_cluster;
                        re_colored_count++;
                    }
                }
            }
            std::cout << "主地面クラスタの点 " << re_colored_count << " 点を赤色にマークしました。" << std::endl;
        } else {
            std::cout << "主地面クラスタが空のため、赤色マーク処理はスキップ（全地面候補点が緑色）。" << std::endl;
        }
    } else {
        std::cout << "表示対象の地面候補点がありません。視覚化をスキップします。" << std::endl;
    }

    if (cluster_viz_cloud && !cluster_viz_cloud->points.empty()) {
        try {
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Ground Cluster Viewer"));
            viewer->setBackgroundColor(0.1, 0.1, 0.1);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cluster_viz_cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cluster_viz_cloud, rgb, "ground_clusters_viz");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ground_clusters_viz");
            viewer->initCameraParameters();
            // viewer->addCoordinateSystem(1.0);

            std::cout << "PCLViewer（地面クラスタ）を表示します。ウィンドウを閉じて続行してください..." << std::endl;
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
            }
            viewer->close();
            std::cout << "PCLViewer（地面クラスタ）を閉じました。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "PCLViewer（地面クラスタ）の初期化または実行中に例外が発生しました: " << e.what() << std::endl;
            std::cerr << "ヘッドレス環境など、表示がサポートされていない可能性があります。" << std::endl;
        }
    } else {
        std::cout << "表示する点群データがないため、PCLViewer（地面クラスタ）の起動をスキップしました。" << std::endl;
    }
    // ---- ここまで新しい視覚化コード ----

    //   3.5.5 グローバル地面平面のフィッティング (Global Ground Plane Fitting)
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
    float plane_distance_threshold = 0.02f;

    if (processor.fitGlobalGroundPlane(main_ground_cluster, plane_distance_threshold, plane_coefficients, plane_inliers)) {
        std::cout << "グローバル地面平面のフィッティング成功。" << std::endl;
        if (plane_coefficients->values.empty()) {
             std::cout << "注意: 平面フィッティングは成功と報告されましたが、係数が空です。" << std::endl;
        } else {
            // Visualize the main ground cluster and the fitted plane
            std::cout << "\n--- 地面平面と主クラスタの可視化開始 ---" << std::endl;
            processor.visualizePlane(main_ground_cluster, plane_coefficients);
            std::cout << "--- 地面平面と主クラスタの可視化終了 ---" << std::endl;

            // Rotate the main ground cluster to be horizontal
            if (main_ground_cluster && !main_ground_cluster->points.empty() && plane_coefficients && !plane_coefficients->values.empty()) {
                std::cout << "\n--- 主クラスタの水平回転処理開始 ---" << std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::Matrix4f rotation_matrix; // To store the transformation matrix
                if (processor.rotateCloudToHorizontal(main_ground_cluster, plane_coefficients, rotated_cloud, rotation_matrix)) {
                    std::cout << "主クラスタの水平回転成功。回転後の点数: " << rotated_cloud->size() << std::endl;

                    // Translate the rotated cloud to Z=0
                    if (rotated_cloud && !rotated_cloud->points.empty()) {
                        std::cout << "\n--- 水平化済みクラスタのZ=0への平行移動処理開始 ---" << std::endl;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr translated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                        Eigen::Matrix4f translation_matrix;
                        if (processor.translateCloudToZZero(rotated_cloud, translated_cloud, translation_matrix)) {
                            std::cout << "水平化済みクラスタのZ=0への平行移動成功。点数: " << translated_cloud->size() << std::endl;
                            processor.visualizeCloud(translated_cloud, "Translated Cloud (Z=0)");
                        } else {
                            std::cout << "警告: 水平化済みクラスタのZ=0への平行移動に失敗しました。" << std::endl;
                        }
                        std::cout << "--- 水平化済みクラスタのZ=0への平行移動処理終了 ---" << std::endl;
                    } else {
                        std::cout << "警告: 回転後クラスタが空のため、Z=0への平行移動をスキップします。" << std::endl;
                    }
                } else {
                    std::cout << "警告: 主クラスタの水平回転に失敗しました。" << std::endl;
                }
                std::cout << "--- 主クラスタの水平回転処理終了 ---" << std::endl;
            }
        }
    } else {
        std::cout << "情報: グローバル地面平面のフィッティングに失敗、または平面が見つかりませんでした。" << std::endl;
    }
    std::cout << "--- 点群処理終了 ---\n" << std::endl;

    // 4. 地図パラメータ計算 (Map Parameter Calculation)
    map_params_util::MapParameters map_params;
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
