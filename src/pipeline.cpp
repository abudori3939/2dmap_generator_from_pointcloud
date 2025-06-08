#include "pipeline.h"

#include <iostream>
#include <string>
#include <filesystem>       // For path operations
#include <algorithm>        // For std::transform
#include <cctype>           // For ::tolower
#include "config_loader.h"  // For map_config::loadConfig
#include <pcl/io/pcd_io.h>  // For PCL I/O
#include <pcl/io/ply_io.h>  // For PCL I/O
#include <pcl/point_types.h>// For PCL point types
#include <pcl/common/io.h> // For copyPointCloud
#include <pcl/common/transforms.h> // For pcl::transformPointCloud
#include <pcl/search/kdtree.h> // For KdTree
#include <pcl/visualization/pcl_visualizer.h> // For PCLVisualizer
#include "map_io.h"       // For GRID_VALUE_UNKNOWN, saveMapAsPGM, saveMapMetadataYAML
#include "file_utils.h"   // For ensureDirectoryExists
#include "map_parameters.h" // For map_params_util::calculateMapParameters

// Anonymous namespace for helper functions like getFileExtension
namespace {

std::string getFileExtension(const std::string& filepath) {
    try {
        std::filesystem::path p(filepath);
        std::string ext = p.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
                       [](unsigned char c){ return std::tolower(c); });
        return ext;
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "File path error (getFileExtension): " << e.what() << std::endl;
        return "";
    }
}

} // anonymous namespace

namespace pipeline {

bool parseArguments(int argc, char* argv[], std::string& pointcloud_file_path, std::string& config_file_path) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pointcloud_file_path> <config_file_path>" << std::endl;
        return false;
    }
    pointcloud_file_path = argv[1];
    config_file_path = argv[2];
    return true;
}

bool saveMap(const PipelineData& data, const std::string& output_dir) {
    utils::ensureDirectoryExists(output_dir);
    std::string pgm_file_basename = "map.pgm";
    std::string pgm_file_path = output_dir + "/" + pgm_file_basename;
    std::string yaml_file_path = output_dir + "/map_metadata.yaml";

    if (!map_io_util::saveMapAsPGM(data.occupancy_grid, data.map_params, pgm_file_path)) {
        std::cerr << "エラー: PGM地図の保存に失敗しました。" << std::endl;
        return false;
    }
    if (!map_io_util::saveMapMetadataYAML(data.map_params, pgm_file_basename, yaml_file_path)) {
        std::cerr << "エラー: 地図メタデータYAMLの保存に失敗しました。" << std::endl;
        return false;
    }

    std::cout << "地図が " << output_dir << " に正常に出力されました。" << std::endl;

   if (data.app_config.preview_map_on_exit) {
       std::cout << "\n情報: 地図のプレビュー表示が設定で有効になっています。" << std::endl;
       std::cout << "      しかし、このバージョンでは直接的なGUIプレビュー機能は実装されていません。" << std::endl;
       std::cout << "      保存された地図ファイルを確認してください: " << pgm_file_path << std::endl;
   }
    return true;
}

bool loadConfiguration(const std::string& config_file_path, map_config::Config& app_config) {
    std::cout << "設定ファイルを読み込み中: " << config_file_path << std::endl;
    if (!map_config::loadConfig(config_file_path, app_config)) {
        std::cerr << "エラー: 設定ファイルの読み込みに失敗しました。プログラムを終了します。" << std::endl;
        return false;
    }
    std::cout << "読み込まれた設定値:" << std::endl;
    std::cout << "  地図解像度: " << app_config.map_resolution << " [m/pixel]" << std::endl;
    std::cout << "  ロボットの高さ: " << app_config.robot_height << " [m]" << std::endl;
    std::cout << "  法線推定半径: " << app_config.normal_estimation_radius << " [m]" << std::endl;
    std::cout << "  地面法線Z閾値: " << app_config.ground_normal_z_threshold << std::endl;
    std::cout << "  ブロックサイズ: " << app_config.block_size << " [m]" << std::endl;
    std::cout << "  最小クラスタ点数 (min_cluster_size): " << app_config.min_cluster_size << std::endl;
    std::cout << "  最大クラスタ点数 (max_cluster_size): " << app_config.max_cluster_size << std::endl;
    std::string yaml_file_path = output_dir + "/map_metadata.yaml";

    if (!map_io_util::saveMapAsPGM(data.occupancy_grid, data.map_params, pgm_file_path)) { // Ensure "map_io.h" is included
        std::cerr << "エラー: PGM地図の保存に失敗しました。" << std::endl;
        return false;
    }
    if (!map_io_util::saveMapMetadataYAML(data.map_params, pgm_file_basename, yaml_file_path)) {
        std::cerr << "エラー: 地図メタデータYAMLの保存に失敗しました。" << std::endl;
        return false;
    }

    std::cout << "地図が " << output_dir << " に正常に出力されました。" << std::endl;

   if (data.app_config.preview_map_on_exit) {
       std::cout << "\n情報: 地図のプレビュー表示が設定で有効になっています。" << std::endl;
       std::cout << "      しかし、このバージョンでは直接的なGUIプレビュー機能は実装されていません。" << std::endl;
       std::cout << "      保存された地図ファイルを確認してください: " << pgm_file_path << std::endl;
   }
    return true;
}

bool loadConfiguration(const std::string& config_file_path, map_config::Config& app_config) {
    std::cout << "設定ファイルを読み込み中: " << config_file_path << std::endl;
    if (!map_config::loadConfig(config_file_path, app_config)) {
        std::cerr << "エラー: 設定ファイルの読み込みに失敗しました。プログラムを終了します。" << std::endl;
        return false;
    }
    std::cout << "読み込まれた設定値:" << std::endl;
    std::cout << "  地図解像度: " << app_config.map_resolution << " [m/pixel]" << std::endl;
    std::cout << "  ロボットの高さ: " << app_config.robot_height << " [m]" << std::endl;
    std::cout << "  法線推定半径: " << app_config.normal_estimation_radius << " [m]" << std::endl;
    std::cout << "  地面法線Z閾値: " << app_config.ground_normal_z_threshold << std::endl;
    std::cout << "  ブロックサイズ: " << app_config.block_size << " [m]" << std::endl;
    std::cout << "  最小クラスタ点数 (min_cluster_size): " << app_config.min_cluster_size << std::endl;
    std::cout << "  最大クラスタ点数 (max_cluster_size): " << app_config.max_cluster_size << std::endl;
    return true;
}

bool loadPointCloud(const std::string& pointcloud_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "点群ファイルを読み込み中: " << pointcloud_file_path << std::endl;
    std::string extension = getFileExtension(pointcloud_file_path); // Uses helper from anonymous namespace
    int load_status = -1;

    if (cloud == nullptr) {
        std::cerr << "エラー: 出力先の点群ポインタがnullです。" << std::endl;
        return false;
    }
    cloud->points.clear(); // Clear any existing points

    if (extension == ".pcd") {
        std::cout << "PCDファイルとして読み込みます。" << std::endl;
        load_status = pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_file_path, *cloud);
    } else if (extension == ".ply") {
        std::cout << "PLYファイルとして読み込みます。" << std::endl;
        load_status = pcl::io::loadPLYFile<pcl::PointXYZ>(pointcloud_file_path, *cloud);
    } else {
        std::cerr << "エラー: 未サポートのファイル拡張子です: '" << extension << "'" << std::endl;
        std::cerr << "PCD (.pcd) または PLY (.ply) ファイルを指定してください。" << std::endl;
        return false;
    }

    if (load_status == -1) {
        std::cerr << "エラー: 点群ファイルの読み込みに失敗しました: " << pointcloud_file_path << std::endl;
        return false;
    }
    if (cloud->points.empty()) {
         std::cerr << "エラー: 点群ファイルは読み込まれましたが、データが空です: " << pointcloud_file_path << std::endl;
         return false;
    }
    std::cout << "点群ファイルから " << cloud->width * cloud->height << " 点のデータを読み込みました。" << std::endl;
    return true;
}

bool processPointCloud(PipelineData& data, proc::PointCloudProcessor& processor) {
    std::cout << "\n--- 点群処理開始 ---" << std::endl;

    //   3.5.1 法線推定 (Normal Estimation)
    if (!processor.estimateNormals(data.raw_cloud, static_cast<float>(data.app_config.normal_estimation_radius), data.normals)) {
        std::cerr << "エラー: 法線推定に失敗しました。プログラムを終了します。" << std::endl;
        return false; // Fatal error
    }
    std::cout << "法線推定が正常に完了しました。法線数: " << data.normals->size() << std::endl;

    //   3.5.2 地面候補点の抽出 (Ground Candidate Extraction)
    if (!processor.extractGroundCandidates(data.raw_cloud, data.normals, static_cast<float>(data.app_config.ground_normal_z_threshold), data.ground_candidates, data.ground_candidate_indices)) {
        std::cerr << "エラー: 地面候補点の抽出に失敗しました。プログラムを終了します。" << std::endl;
        return false; // Fatal error
    }
    std::cout << "地面候補点の抽出成功。候補点数: " << data.ground_candidates->size() << ", インデックス数: " << data.ground_candidate_indices->indices.size() << std::endl;

    // Extract non-horizontal points
    if (processor.extractNonHorizontalPoints(data.raw_cloud, data.ground_candidate_indices, data.non_horizontal_cloud)) {
        std::cout << "非水平要素の抽出成功。非水平点群の点数: " << data.non_horizontal_cloud->size() << std::endl;
    } else {
        std::cerr << "警告: 非水平要素の抽出に失敗しました。" << std::endl;
        // Not necessarily fatal, could be an empty non-horizontal set
    }

    //   3.5.3 地面候補点の可視化 (Visualization of Ground Candidates)
    std::cout << "\n--- 地面候補点の可視化開始 ---" << std::endl;
    if (!data.raw_cloud->points.empty()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*data.raw_cloud, *colored_cloud);

        uint8_t r_other = 0, g_other = 255, b_other = 0;
        uint8_t r_ground = 255, g_ground = 0, b_ground = 0;

        for (auto& point : colored_cloud->points) {
            point.r = r_other; point.g = g_other; point.b = b_other;
        }

        if (data.ground_candidates && !data.ground_candidates->points.empty()) {
            std::cout << "地面候補点の色付け処理（KdTree）を開始します..." << std::endl;
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_rgb(new pcl::search::KdTree<pcl::PointXYZRGB>());
            tree_rgb->setInputCloud(colored_cloud);

            int colored_count = 0;
            for (const auto& ground_point : data.ground_candidates->points) {
                pcl::PointXYZRGB search_point_rgb;
                search_point_rgb.x = ground_point.x;
                search_point_rgb.y = ground_point.y;
                search_point_rgb.z = ground_point.z;

                std::vector<int> point_idx_nkns(1);
                std::vector<float> point_squared_distance(1);

                if (tree_rgb->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                    if (point_squared_distance[0] < 0.00001f) {
                        colored_cloud->points[point_idx_nkns[0]].r = r_ground;
                        colored_cloud->points[point_idx_nkns[0]].g = g_ground;
                        colored_cloud->points[point_idx_nkns[0]].b = b_ground;
                        colored_count++;
                    }
                }
            }
            std::cout << "地面候補点の色付け処理（KdTree）完了。 " << colored_count << " 点が赤色にマークされました。" << std::endl;
            if (static_cast<size_t>(colored_count) != data.ground_candidates->points.size()) {
                std::cout << "警告: 全ての地面候補点 (" << data.ground_candidates->points.size()
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

    if (data.ground_candidates->empty()) {
        std::cout << "注意: 地面候補点が見つかりませんでした。パラメータまたは入力データを確認してください。" << std::endl;
        // This might not be fatal, could still proceed to generate an empty map or map with only non-ground
    }

    //   3.5.4 主地面クラスタの特定 (Main Ground Cluster Extraction)
    float cluster_tolerance = 2.0f * static_cast<float>(data.app_config.map_resolution);
    if (!processor.extractMainGroundCluster(data.ground_candidates, cluster_tolerance, data.app_config.min_cluster_size, data.app_config.max_cluster_size, data.main_ground_cluster)) {
        std::cerr << "エラー: 主地面クラスタの特定に失敗しました。プログラムを終了します。" << std::endl;
        return false; // Often fatal if no main ground can be identified for transformation
    }
    std::cout << "主地面クラスタの特定成功。クラスタ点数: " << data.main_ground_cluster->size() << std::endl;
    if (data.main_ground_cluster->empty()) {
        if (data.ground_candidates->empty()) {
            std::cout << "情報: 地面候補点がなかったため、主地面クラスタもありません。" << std::endl;
        } else {
            std::cout << "注意: 地面候補点はありましたが、条件に合う主地面クラスタは見つかりませんでした。" << std::endl;
        }
        // Depending on requirements, this could be fatal or not.
        // If subsequent steps require a main_ground_cluster, this should return false.
        // For now, let's assume it might be non-fatal and an empty map is acceptable.
    }

    // ---- 主地面クラスタの視覚化 ----
    std::cout << "\n--- 主地面クラスタの視覚化開始 ---" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_viz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    uint8_t r_main_cluster = 255, g_main_cluster = 0, b_main_cluster = 0;
    uint8_t r_other_candidates = 0, g_other_candidates = 255, b_other_candidates = 0;

    if (data.ground_candidates && !data.ground_candidates->points.empty()) {
        for (const auto& candidate_point : data.ground_candidates->points) {
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

        if (data.main_ground_cluster && !data.main_ground_cluster->points.empty()) {
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_cluster_viz(new pcl::search::KdTree<pcl::PointXYZRGB>());
            tree_cluster_viz->setInputCloud(cluster_viz_cloud);
            int re_colored_count = 0;
            for (const auto& main_cluster_point : data.main_ground_cluster->points) {
                pcl::PointXYZRGB search_point_rgb;
                search_point_rgb.x = main_cluster_point.x;
                search_point_rgb.y = main_cluster_point.y;
                search_point_rgb.z = main_cluster_point.z;
                std::vector<int> point_idx_nkns(1);
                std::vector<float> point_squared_distance(1);
                if (tree_cluster_viz->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                    if (point_squared_distance[0] < 0.00001f) {
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

    //   3.5.5 グローバル地面平面のフィッティング (Global Ground Plane Fitting)
    float plane_distance_threshold = 0.02f;
    if (processor.fitGlobalGroundPlane(data.main_ground_cluster, plane_distance_threshold, data.plane_coefficients, data.plane_inliers)) {
        std::cout << "グローバル地面平面のフィッティング成功。" << std::endl;
        if (data.plane_coefficients->values.empty()) {
             std::cout << "注意: 平面フィッティングは成功と報告されましたが、係数が空です。" << std::endl;
        } else {
            std::cout << "\n--- 地面平面と主クラスタの可視化開始 ---" << std::endl;
            processor.visualizePlane(data.main_ground_cluster, data.plane_coefficients);
            std::cout << "--- 地面平面と主クラスタの可視化終了 ---" << std::endl;

            if (data.main_ground_cluster && !data.main_ground_cluster->points.empty() && data.plane_coefficients && !data.plane_coefficients->values.empty()) {
                std::cout << "\n--- 主クラスタの水平回転処理開始 ---" << std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_main_ground_cluster(new pcl::PointCloud<pcl::PointXYZ>());
                if (processor.rotateCloudToHorizontal(data.main_ground_cluster, data.plane_coefficients, rotated_main_ground_cluster, data.rotation_matrix)) {
                    std::cout << "主クラスタの水平回転成功。回転後の点数: " << rotated_main_ground_cluster->size() << std::endl;

                    if (rotated_main_ground_cluster && !rotated_main_ground_cluster->points.empty()) {
                        std::cout << "\n--- 水平化済みクラスタのZ=0への平行移動処理開始 ---" << std::endl;
                        if (processor.translateCloudToZZero(rotated_main_ground_cluster, data.ground_transformed_cloud, data.translation_matrix)) {
                            std::cout << "水平化済みクラスタのZ=0への平行移動成功。点数: " << data.ground_transformed_cloud->size() << std::endl;

                            if (data.non_horizontal_cloud && !data.non_horizontal_cloud->points.empty()) {
                                std::cout << "\n--- 非水平要素の変換処理開始 ---" << std::endl;
                                pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                                pcl::transformPointCloud(*data.non_horizontal_cloud, *non_horizontal_rotated_cloud, data.rotation_matrix);
                                pcl::transformPointCloud(*non_horizontal_rotated_cloud, *data.non_horizontal_transformed_cloud, data.translation_matrix);
                                std::cout << "非水平要素の変換成功。変換後の点数: " << data.non_horizontal_transformed_cloud->size() << std::endl;
                                std::cout << "--- 非水平要素の変換処理終了 ---" << std::endl;

                                processor.visualizeCombinedClouds(data.ground_transformed_cloud, data.non_horizontal_transformed_cloud, "Combined Transformed Clouds");

                                if (data.ground_candidates && !data.ground_candidates->points.empty() &&
                                    data.ground_transformed_cloud && !data.ground_transformed_cloud->points.empty()) {
                                    std::cout << "\n--- 全ての水平候補点の変換処理開始 ---" << std::endl;
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_rotated_ground_candidates(new pcl::PointCloud<pcl::PointXYZ>());
                                    std::cout << "全ての水平候補点 (" << data.ground_candidates->size() << "点) を回転中..." << std::endl;
                                    pcl::transformPointCloud(*data.ground_candidates, *temp_rotated_ground_candidates, data.rotation_matrix);
                                    std::cout << "回転された全ての水平候補点 (" << temp_rotated_ground_candidates->size() << "点) を平行移動中..." << std::endl;
                                    pcl::transformPointCloud(*temp_rotated_ground_candidates, *data.all_ground_candidates_transformed_cloud, data.translation_matrix);
                                    std::cout << "全ての水平候補点の変換成功。変換後の点数: " << data.all_ground_candidates_transformed_cloud->size() << std::endl;
                                    std::cout << "--- 全ての水平候補点の変換処理終了 ---" << std::endl;
                                } else {
                                    std::cout << "\n--- 全ての水平候補点の変換処理 (スキップ) ---" << std::endl;
                                    if (!data.ground_candidates || data.ground_candidates->points.empty()) {
                                        std::cout << "理由: 元の水平候補点群が空です。" << std::endl;
                                    } else if (!data.ground_transformed_cloud || data.ground_transformed_cloud->points.empty()) {
                                        std::cout << "理由: 主地面クラスタの変換が成功しなかったため、有効な変換行列がありません。" << std::endl;
                                    }
                                    data.all_ground_candidates_transformed_cloud->points.clear();
                                    data.all_ground_candidates_transformed_cloud->width = 0;
                                    data.all_ground_candidates_transformed_cloud->height = 1;
                                    data.all_ground_candidates_transformed_cloud->is_dense = true;
                                    std::cout << "all_ground_candidates_transformed_cloud は空として初期化されました。" << std::endl;
                                }

                                if (data.non_horizontal_transformed_cloud && !data.non_horizontal_transformed_cloud->points.empty()) {
                                    std::cout << "\n--- 非水平要素のダウンサンプリング開始 ---" << std::endl;
                                    float leaf_size = static_cast<float>(data.app_config.map_resolution);
                                    if (!processor.downsampleCloud(data.non_horizontal_transformed_cloud, leaf_size, leaf_size, leaf_size, data.non_horizontal_downsampled_cloud)) {
                                        std::cerr << "警告: 非水平要素のダウンサンプリングに失敗しました。以降の処理ではダウンサンプリングされていない非水平要素を使用します。" << std::endl;
                                    } else {
                                        std::cout << "非水平要素のダウンサンプリング成功。処理前点数: " << data.non_horizontal_transformed_cloud->size()
                                                  << ", 処理後点数: " << data.non_horizontal_downsampled_cloud->size() << std::endl;
                                    }
                                    std::cout << "--- 非水平要素のダウンサンプリング終了 ---" << std::endl;
                                } else {
                                    std::cout << "情報: 変換後の非水平要素の点群が空のため、ダウンサンプリングをスキップします。" << std::endl;
                                    if (data.non_horizontal_downsampled_cloud->points.empty() && data.non_horizontal_downsampled_cloud->header.frame_id.empty()) {
                                         data.non_horizontal_downsampled_cloud->width = 0;
                                         data.non_horizontal_downsampled_cloud->height = 1;
                                         data.non_horizontal_downsampled_cloud->is_dense = true;
                                    }
                                }

                                if (data.non_horizontal_downsampled_cloud && !data.non_horizontal_downsampled_cloud->points.empty()) {
                                    std::cout << "\n--- 非水平要素の高さフィルタリング開始 ---" << std::endl;
                                    float min_z_filter = -1.0f;
                                    float max_z_filter = static_cast<float>(data.app_config.robot_height);
                                    if (!processor.filterCloudByHeight(data.non_horizontal_downsampled_cloud, min_z_filter, max_z_filter, data.non_horizontal_filtered_cloud)) {
                                        std::cerr << "警告: 非水平要素の高さフィルタリングに失敗しました。" << std::endl;
                                    } else {
                                        std::cout << "非水平要素の高さフィルタリング成功。処理前点数: " << data.non_horizontal_downsampled_cloud->size()
                                                  << ", 処理後点数: " << data.non_horizontal_filtered_cloud->size() << std::endl;
                                    }
                                    std::cout << "--- 非水平要素の高さフィルタリング終了 ---" << std::endl;
                                } else {
                                    std::cout << "情報: ダウンサンプリング後の非水平要素の点群が空のため、高さフィルタリングをスキップします。" << std::endl;
                                    if (data.non_horizontal_filtered_cloud->points.empty() && data.non_horizontal_filtered_cloud->header.frame_id.empty()) {
                                        data.non_horizontal_filtered_cloud->width = 0;
                                        data.non_horizontal_filtered_cloud->height = 1;
                                        data.non_horizontal_filtered_cloud->is_dense = true;
                                    }
                                }

                            } else { // Non-horizontal cloud was empty initially
                                std::cout << "情報: 非水平要素の点群が空のため、変換をスキップします。" << std::endl;
                                processor.visualizeCombinedClouds(data.ground_transformed_cloud, data.non_horizontal_transformed_cloud, "Combined Transformed Clouds (Ground Only)");
                                // Ensure dependent clouds are also empty and valid
                                data.all_ground_candidates_transformed_cloud->points.clear(); // Should be empty if non_horizontal_cloud is empty and ground_candidates were transformed based on main_ground
                                data.all_ground_candidates_transformed_cloud->width = 0; data.all_ground_candidates_transformed_cloud->height = 1; data.all_ground_candidates_transformed_cloud->is_dense = true;

                                data.non_horizontal_downsampled_cloud->points.clear();
                                data.non_horizontal_downsampled_cloud->width = 0; data.non_horizontal_downsampled_cloud->height = 1; data.non_horizontal_downsampled_cloud->is_dense = true;
                                std::cout << "情報: 非水平要素が空だったため、対応するダウンサンプリング済み点群も空として初期化されました。" << std::endl;
                                data.non_horizontal_filtered_cloud->points.clear();
                                data.non_horizontal_filtered_cloud->width = 0; data.non_horizontal_filtered_cloud->height = 1; data.non_horizontal_filtered_cloud->is_dense = true;
                                std::cout << "情報: 非水平要素が空だったため、対応する高さフィルタリング済み点群も空として初期化されました。" << std::endl;
                            }
                        } else {
                            std::cout << "警告: 水平化済みクラスタのZ=0への平行移動に失敗しました。" << std::endl;
                            // This could be considered fatal if subsequent map generation relies on transformed_cloud
                        }
                        std::cout << "--- 水平化済みクラスタのZ=0への平行移動処理終了 ---" << std::endl;
                    } else {
                        std::cout << "警告: 回転後クラスタが空のため、Z=0への平行移動をスキップします。" << std::endl;
                    }
                } else {
                    std::cout << "警告: 主クラスタの水平回転に失敗しました。" << std::endl;
                }
                std::cout << "--- 主クラスタの水平回転処理終了 ---" << std::endl;
            } else if (!data.main_ground_cluster || data.main_ground_cluster->points.empty()) {
                 std::cout << "情報: 主地面クラスタが空のため、変換処理はスキップされます。" << std::endl;
            } else if (!data.plane_coefficients || data.plane_coefficients->values.empty()) {
                 std::cout << "情報: 平面係数が利用できないため、変換処理はスキップされます。" << std::endl;
            }
        }
    } else {
        std::cout << "情報: グローバル地面平面のフィッティングに失敗、または平面が見つかりませんでした。" << std::endl;
        // This could be fatal if ground plane is essential for subsequent steps
        // For now, allow to proceed, potentially resulting in an empty or untransformed map
    }
    std::cout << "--- 点群処理終了 ---\n" << std::endl;
    return true; // If we reached here, assume processing was "successful" in terms of not hitting a return false
}

bool generateOccupancyGrid(PipelineData& data) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_for_map_bounds(new pcl::PointCloud<pcl::PointXYZ>());
    std::cout << "\n--- 地図範囲定義のための結合点群作成開始 ---" << std::endl;

    if (data.all_ground_candidates_transformed_cloud && !data.all_ground_candidates_transformed_cloud->points.empty()) {
        *combined_for_map_bounds += *data.all_ground_candidates_transformed_cloud;
        std::cout << "  - 追加された変換済みの全地面候補点群 (all_ground_candidates_transformed_cloud) の点数: " << data.all_ground_candidates_transformed_cloud->size() << std::endl;
    } else {
        std::cout << "  - 変換済みの全地面候補点群 (all_ground_candidates_transformed_cloud) は空か無効です。" << std::endl;
    }

    if (data.non_horizontal_filtered_cloud && !data.non_horizontal_filtered_cloud->points.empty()) {
        *combined_for_map_bounds += *data.non_horizontal_filtered_cloud;
        std::cout << "  - 追加された障害物点群 (non_horizontal_filtered_cloud) の点数: " << data.non_horizontal_filtered_cloud->size() << std::endl;
    } else {
        std::cout << "  - 障害物点群 (non_horizontal_filtered_cloud) は空か無効です。" << std::endl;
    }

    if (combined_for_map_bounds->points.empty()) {
        std::cout << "  - 結合後の点群 (combined_for_map_bounds) は空です。" << std::endl;
        combined_for_map_bounds->width = 0;
        combined_for_map_bounds->height = 1;
        combined_for_map_bounds->is_dense = true;
    } else {
        if (data.ground_transformed_cloud && !data.ground_transformed_cloud->points.empty() && !data.ground_transformed_cloud->header.frame_id.empty()) {
            combined_for_map_bounds->header = data.ground_transformed_cloud->header;
        } else if (data.non_horizontal_filtered_cloud && !data.non_horizontal_filtered_cloud->points.empty() && !data.non_horizontal_filtered_cloud->header.frame_id.empty()) {
            combined_for_map_bounds->header = data.non_horizontal_filtered_cloud->header;
        } else if (data.raw_cloud && !data.raw_cloud->header.frame_id.empty()) {
            combined_for_map_bounds->header.frame_id = data.raw_cloud->header.frame_id;
        }
        combined_for_map_bounds->width = combined_for_map_bounds->points.size();
        combined_for_map_bounds->height = 1;
        combined_for_map_bounds->is_dense = true;
        std::cout << "  - 結合後の総点数 (combined_for_map_bounds): " << combined_for_map_bounds->size() << std::endl;
    }
    std::cout << "--- 地図範囲定義のための結合点群作成終了 ---" << std::endl;

    std::cout << "\n--- 地図パラメータ計算開始 (結合点群基準) ---" << std::endl;
    if (!map_params_util::calculateMapParameters(combined_for_map_bounds, data.app_config.map_resolution, data.map_params)) {
        std::cerr << "エラー: 地図パラメータの計算に失敗しました。入力となる結合点群が空であるか、解像度が不適切である可能性があります。プログラムを終了します。" << std::endl;
        return false;
    }
    std::cout << "計算された地図パラメータ (結合点群基準):" << std::endl;
    std::cout << "  原点X: " << data.map_params.origin_x << " [m], Y: " << data.map_params.origin_y << " [m]" << std::endl;
    std::cout << "  幅: " << data.map_params.width_pixels << " [ピクセル], 高さ: " << data.map_params.height_pixels << " [ピクセル]" << std::endl;
    std::cout << "  解像度: " << data.map_params.resolution << " [m/pixel]" << std::endl;
    std::cout << "--- 地図パラメータ計算終了 ---" << std::endl;

    std::cout << "\n--- 占有格子地図の初期化開始 ---" << std::endl;
    try {
        size_t grid_size = 0;
        if (data.map_params.width_pixels > 0 && data.map_params.height_pixels > 0) {
            grid_size = static_cast<size_t>(data.map_params.width_pixels) * static_cast<size_t>(data.map_params.height_pixels);
        } else {
            std::cout << "情報: 地図の幅または高さが0以下です (" << data.map_params.width_pixels << "x" << data.map_params.height_pixels
                      << ")。空の占有格子地図を作成します。" << std::endl;
        }
        data.occupancy_grid.assign(grid_size, map_io_util::GRID_VALUE_UNKNOWN);
    } catch (const std::bad_alloc& e) {
        std::cerr << "エラー: 地図データ構造のためのメモリ確保に失敗 ("
                  << (static_cast<size_t>(data.map_params.width_pixels) * static_cast<size_t>(data.map_params.height_pixels) * sizeof(int8_t))
                  << " bytes requested): " << e.what() << std::endl;
        return false;
    } catch (const std::length_error& e) {
        std::cerr << "エラー: 地図データ構造のサイズが大きすぎます ("
                  << (static_cast<size_t>(data.map_params.width_pixels) * static_cast<size_t>(data.map_params.height_pixels))
                  << " elements): " << e.what() << std::endl;
        return false;
    }
    std::cout << "占有格子地図を初期化。リクエストサイズ: " << data.map_params.width_pixels << "x" << data.map_params.height_pixels
              << ", 実際のグリッド要素数: " << data.occupancy_grid.size()
              << ", 初期値: " << static_cast<int>(map_io_util::GRID_VALUE_UNKNOWN) << std::endl;
    std::cout << "--- 占有格子地図の初期化終了 ---" << std::endl;

    if (data.all_ground_candidates_transformed_cloud && !data.all_ground_candidates_transformed_cloud->points.empty()) {
        if (data.map_params.width_pixels > 0 && data.map_params.height_pixels > 0 && !data.occupancy_grid.empty()) {
            std::cout << "\n--- 占有格子地図への空き領域（全水平候補）プロット開始 ---" << std::endl;
            std::cout << "処理対象の点群 (変換済み全水平候補): " << data.all_ground_candidates_transformed_cloud->size() << " 点" << std::endl;
            int free_cells_marked = 0;
            int points_outside_map_free = 0;

            for (const auto& point : data.all_ground_candidates_transformed_cloud->points) {
                int map_x = static_cast<int>(std::floor((point.x - data.map_params.origin_x) / data.map_params.resolution));
                int map_y_ros = static_cast<int>(std::floor((point.y - data.map_params.origin_y) / data.map_params.resolution));
                int map_y_pgm = data.map_params.height_pixels - 1 - map_y_ros;

                if (map_x >= 0 && map_x < data.map_params.width_pixels &&
                    map_y_pgm >= 0 && map_y_pgm < data.map_params.height_pixels) {
                    size_t index = static_cast<size_t>(map_y_pgm) * data.map_params.width_pixels + map_x;
                    if (index < data.occupancy_grid.size()) {
                        data.occupancy_grid[index] = map_io_util::GRID_VALUE_FREE;
                        free_cells_marked++;
                    } else {
                        points_outside_map_free++;
                    }
                } else {
                    points_outside_map_free++;
                }
            }
            std::cout << "空き領域プロット完了。 " << free_cells_marked << " 個のセルが空き領域としてマークされました。" << std::endl;
            if (points_outside_map_free > 0) {
                std::cout << points_outside_map_free << " 個の点は地図範囲外か、インデックス計算エラーでした。" << std::endl;
            }
            std::cout << "--- 占有格子地図への空き領域プロット終了 ---" << std::endl;
        } else {
            std::cout << "情報: 地図の次元が無効 (" << data.map_params.width_pixels << "x" << data.map_params.height_pixels
                      << ") または占有格子が空のため、空き領域プロットをスキップします。" << std::endl;
        }
    } else {
        std::cout << "\n情報: プロット対象の変換済み全水平候補点群が空のため、空き領域プロットをスキップします。" << std::endl;
    }

    if (data.non_horizontal_filtered_cloud && !data.non_horizontal_filtered_cloud->points.empty()) {
        if (data.map_params.width_pixels > 0 && data.map_params.height_pixels > 0 && !data.occupancy_grid.empty()) {
            std::cout << "\n--- 占有格子地図への障害物プロット開始 ---" << std::endl;
            std::cout << "処理対象の点群 (フィルタリング済み非水平要素): " << data.non_horizontal_filtered_cloud->size() << " 点" << std::endl;
            int obstacles_marked = 0;
            int points_outside_map = 0;

            for (const auto& point : data.non_horizontal_filtered_cloud->points) {
                int map_x = static_cast<int>(std::floor((point.x - data.map_params.origin_x) / data.map_params.resolution));
                int map_y_ros = static_cast<int>(std::floor((point.y - data.map_params.origin_y) / data.map_params.resolution));
                int map_y_pgm = data.map_params.height_pixels - 1 - map_y_ros;

                if (map_x >= 0 && map_x < data.map_params.width_pixels &&
                    map_y_pgm >= 0 && map_y_pgm < data.map_params.height_pixels) {
                    size_t index = static_cast<size_t>(map_y_pgm) * data.map_params.width_pixels + map_x;
                    if (index < data.occupancy_grid.size()) {
                        data.occupancy_grid[index] = map_io_util::GRID_VALUE_OCCUPIED;
                        obstacles_marked++;
                    } else {
                        points_outside_map++;
                    }
                } else {
                    points_outside_map++;
                }
            }
            std::cout << "障害物プロット完了。 " << obstacles_marked << " 個のセルが障害物としてマークされました。" << std::endl;
            if (points_outside_map > 0) {
                std::cout << points_outside_map << " 個の点は地図範囲外か、インデックス計算エラーでした。" << std::endl;
            }
            std::cout << "--- 占有格子地図への障害物プロット終了 ---" << std::endl;
        } else {
            std::cout << "情報: 地図の次元が無効 (" << data.map_params.width_pixels << "x" << data.map_params.height_pixels
                      << ") または占有格子が空のため、障害物プロットをスキップします。" << std::endl;
        }
    } else {
        std::cout << "\n情報: プロット対象の障害物点群が空のため、占有格子地図への障害物プロットをスキップします。" << std::endl;
    }
    return true;
}

} // namespace pipeline
