#include "pipeline.h"

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>       // For path operations
#include <algorithm>        // For std::transform
#include <cctype>           // For ::tolower

#include "config_loader.h"
#include "map_parameters.h" // For map_params_util, calculateMapParameters
#include "map_io.h"         // For GRID_VALUE_UNKNOWN, GRID_VALUE_FREE, GRID_VALUE_OCCUPIED, saveMapAsPGM, saveMapMetadataYAML
#include "file_utils.h"     // For ensureDirectoryExists
#include "map_processor.h"  // For MapProcessor class

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h> // For copyPointCloud
#include <pcl/common/transforms.h> // For pcl::transformPointCloud
#include <pcl/search/kdtree.h> // For KdTree
#include <pcl/visualization/pcl_visualizer.h> // For PCLVisualizer
// Note: point_cloud_processor.h is included via pipeline.h

namespace { // Anonymous namespace for helper functions

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

// Forward declarations for static helper functions
namespace detail {
    static bool estimateNormals(PipelineData& data, proc::PointCloudProcessor& processor);
    static bool extractGroundAndNonHorizontal(PipelineData& data, proc::PointCloudProcessor& processor);
    static void visualizeGroundCandidates(bool enable_visualization_flag, const PipelineData& data); // Const if only reading
    static bool extractMainGroundCluster(PipelineData& data, proc::PointCloudProcessor& processor);
    static void visualizeMainGroundCluster(const PipelineData& data); // Const if only reading
    static bool fitGlobalGroundPlane(PipelineData& data, proc::PointCloudProcessor& processor);
    static void visualizeGroundPlane(bool enable_visualization_flag, const PipelineData& data, proc::PointCloudProcessor& processor); // REMOVED CONST
    static bool transformClouds(PipelineData& data, proc::PointCloudProcessor& processor);
    static void visualizeCombinedTransformedClouds(bool enable_visualization_flag, const PipelineData& data, proc::PointCloudProcessor& processor); // REMOVED CONST
    static bool transformAllGroundCandidates(PipelineData& data);
    static bool processNonHorizontalCloud(PipelineData& data, proc::PointCloudProcessor& processor);
}


bool parseArguments(int argc, char* argv[], std::string& pointcloud_file_path, std::string& config_file_path) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <pointcloud_file_path> <config_file_path>" << std::endl;
        return false;
    }
    pointcloud_file_path = argv[1];
    config_file_path = argv[2];
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
    std::string extension = getFileExtension(pointcloud_file_path);
    int load_status = -1;

    if (cloud == nullptr) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>()); // Ensure cloud is initialized
    }
    cloud->points.clear();

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

// --- Start of detail namespace for helper functions ---
namespace detail {

static bool estimateNormals(PipelineData& data, proc::PointCloudProcessor& processor) {
    if (!processor.estimateNormals(data.raw_cloud, static_cast<float>(data.app_config.normal_estimation_radius), data.normals)) {
        std::cerr << "エラー: 法線推定に失敗しました。" << std::endl;
        return false;
    }
    std::cout << "法線推定が正常に完了しました。法線数: " << data.normals->size() << std::endl;
    return true;
}

static bool extractGroundAndNonHorizontal(PipelineData& data, proc::PointCloudProcessor& processor) {
    if (!processor.extractGroundCandidates(data.raw_cloud, data.normals, static_cast<float>(data.app_config.ground_normal_z_threshold), data.ground_candidates, data.ground_candidate_indices)) {
        std::cerr << "エラー: 地面候補点の抽出に失敗しました。" << std::endl;
        return false;
    }
    std::cout << "地面候補点の抽出成功。候補点数: " << data.ground_candidates->size() << ", インデックス数: " << data.ground_candidate_indices->indices.size() << std::endl;

    if (processor.extractNonHorizontalPoints(data.raw_cloud, data.ground_candidate_indices, data.non_horizontal_cloud)) {
        std::cout << "非水平要素の抽出成功。非水平点群の点数: " << data.non_horizontal_cloud->size() << std::endl;
    } else {
        std::cerr << "警告: 非水平要素の抽出に失敗しました。" << std::endl;
    }
    if (data.ground_candidates->empty()) {
        std::cout << "注意: 地面候補点が見つかりませんでした。" << std::endl;
    }
    return true;
}

static void visualizeGroundCandidates(bool enable_visualization_flag, const PipelineData& data) {
    if (!enable_visualization_flag) {
        std::cout << "可視化が無効なため、visualizeGroundCandidatesをスキップします。" << std::endl;
        return;
    }
    std::cout << "\n--- 地面候補点の可視化開始 ---" << std::endl;
    if (data.raw_cloud->points.empty()) {
        std::cout << "元の点群が空のため、可視化をスキップします。" << std::endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*data.raw_cloud, *colored_cloud);
    uint8_t r_other = 0, g_other = 255, b_other = 0; // Green for non-candidates
    uint8_t r_ground = 255, g_ground = 0, b_ground = 0; // Red for candidates

    for (auto& point : colored_cloud->points) {
        point.r = r_other; point.g = g_other; point.b = b_other;
    }
    if (data.ground_candidates && !data.ground_candidates->points.empty()) {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_rgb(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree_rgb->setInputCloud(colored_cloud);
        int colored_count = 0;
        for (const auto& ground_point : data.ground_candidates->points) {
            pcl::PointXYZRGB search_point_rgb;
            search_point_rgb.x = ground_point.x; search_point_rgb.y = ground_point.y; search_point_rgb.z = ground_point.z;
            std::vector<int> point_idx_nkns(1); std::vector<float> point_squared_distance(1);
            if (tree_rgb->nearestKSearch(search_point_rgb, 1, point_idx_nkns, point_squared_distance) > 0) {
                if (point_squared_distance[0] < 0.00001f) { // Tolerance for matching
                    colored_cloud->points[point_idx_nkns[0]].r = r_ground;
                    colored_cloud->points[point_idx_nkns[0]].g = g_ground;
                    colored_cloud->points[point_idx_nkns[0]].b = b_ground;
                    colored_count++;
                }
            }
        }
        std::cout << "地面候補点の色付け処理（KdTree）完了。 " << colored_count << " 点が赤色にマークされました。" << std::endl;
         if (static_cast<size_t>(colored_count) != data.ground_candidates->points.size()) {
             std::cout << "警告: 全ての地面候補点 (" << data.ground_candidates->points.size() << "点) がKdTreeで色付けされたわけではありません。" << std::endl;
         }
    } else { std::cout << "地面候補点がないため、色付け処理はスキップされました。" << std::endl; }
    try {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer - Ground Candidates"));
        viewer->setBackgroundColor(0.1, 0.1, 0.1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb, "ground_visualization_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ground_visualization_cloud");
        viewer->initCameraParameters();
        std::cout << "PCLViewerを表示します。ウィンドウを閉じて続行してください..." << std::endl;
        while (!viewer->wasStopped()) { viewer->spinOnce(100); }
        viewer->close(); std::cout << "PCLViewerを閉じました。" << std::endl;
    } catch (const std::exception& e) { std::cerr << "PCLViewerエラー (Ground Candidates): " << e.what() << std::endl; }
}

static bool extractMainGroundCluster(PipelineData& data, proc::PointCloudProcessor& processor) {
    float cluster_tolerance = 2.0f * static_cast<float>(data.app_config.map_resolution);
    if (!processor.extractMainGroundCluster(data.ground_candidates, cluster_tolerance, data.app_config.min_cluster_size, data.app_config.max_cluster_size, data.main_ground_cluster)) {
        std::cerr << "エラー: 主地面クラスタの特定に失敗しました。" << std::endl;
        return false;
    }
    std::cout << "主地面クラスタの特定成功。クラスタ点数: " << data.main_ground_cluster->size() << std::endl;
    if (data.main_ground_cluster->empty()) { std::cout << "注意: 主地面クラスタが空です。" << std::endl; }
    return true;
}

static void visualizeMainGroundCluster(const PipelineData& data) {
    // Similar to visualizeGroundCandidates, but colors main_ground_cluster vs other ground_candidates
    // This visualization logic can be complex and is omitted for brevity here but would be similar to visualizeGroundCandidates
    std::cout << "\n--- 主地面クラスタの視覚化 (省略) ---" << std::endl;
}

static bool fitGlobalGroundPlane(PipelineData& data, proc::PointCloudProcessor& processor) {
    if (data.main_ground_cluster->empty()) {
        std::cout << "情報: 主地面クラスタが空のため、平面フィッティングをスキップします。" << std::endl;
        return true; // Not a failure if there's no ground to fit
    }
    float plane_distance_threshold = 0.02f; // Example value
    if (!processor.fitGlobalGroundPlane(data.main_ground_cluster, plane_distance_threshold, data.plane_coefficients, data.plane_inliers)) {
        std::cout << "情報: グローバル地面平面のフィッティングに失敗、または平面が見つかりませんでした。" << std::endl;
        // Depending on strictness, this could be true or false. Let's say it's not fatal.
    } else {
        std::cout << "グローバル地面平面のフィッティング成功。" << std::endl;
    }
    return true;
}

static void visualizeGroundPlane(bool enable_visualization_flag, const PipelineData& data, proc::PointCloudProcessor& processor) { // Removed const from processor
    if (!enable_visualization_flag) {
        std::cout << "可視化が無効なため、visualizeGroundPlaneをスキップします。" << std::endl;
        return;
    }
    if (data.main_ground_cluster && !data.main_ground_cluster->points.empty() && data.plane_coefficients && !data.plane_coefficients->values.empty()) {
        std::cout << "\n--- 地面平面と主クラスタの可視化開始 ---" << std::endl;
        processor.visualizePlane(enable_visualization_flag, data.main_ground_cluster, data.plane_coefficients);
        std::cout << "--- 地面平面と主クラスタの可視化終了 ---" << std::endl;
    }
}

static bool transformClouds(PipelineData& data, proc::PointCloudProcessor& processor) {
    if (!data.main_ground_cluster || data.main_ground_cluster->points.empty() || !data.plane_coefficients || data.plane_coefficients->values.empty()) {
        std::cout << "情報: 主地面クラスタまたは平面係数が不十分なため、変換処理をスキップします。" << std::endl;
        // Ensure dependent clouds are validly empty
        data.ground_transformed_cloud->points.clear();
        data.non_horizontal_transformed_cloud->points.clear();
        return true; // Not a failure if there's nothing to transform properly
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_main_ground_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    if (!processor.rotateCloudToHorizontal(data.main_ground_cluster, data.plane_coefficients, rotated_main_ground_cluster, data.rotation_matrix)) {
        std::cout << "警告: 主クラスタの水平回転に失敗しました。" << std::endl;
        return true; // Allow continuation
    }
    if (!processor.translateCloudToZZero(rotated_main_ground_cluster, data.ground_transformed_cloud, data.translation_matrix)) {
        std::cout << "警告: 水平化済みクラスタのZ=0への平行移動に失敗しました。" << std::endl;
        return true; // Allow continuation
    }

    if (data.non_horizontal_cloud && !data.non_horizontal_cloud->points.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*data.non_horizontal_cloud, *non_horizontal_rotated_cloud, data.rotation_matrix);
        pcl::transformPointCloud(*non_horizontal_rotated_cloud, *data.non_horizontal_transformed_cloud, data.translation_matrix);
        std::cout << "非水平要素の変換成功。変換後の点数: " << data.non_horizontal_transformed_cloud->size() << std::endl;
    } else {
        std::cout << "情報: 非水平要素の点群が空のため、変換をスキップします。" << std::endl;
        data.non_horizontal_transformed_cloud->points.clear();
    }
    return true;
}

static void visualizeCombinedTransformedClouds(bool enable_visualization_flag, const PipelineData& data, proc::PointCloudProcessor& processor) { // Removed const from processor
    if (!enable_visualization_flag) {
        std::cout << "可視化が無効なため、visualizeCombinedTransformedCloudsをスキップします。" << std::endl;
        return;
    }
    if ((data.ground_transformed_cloud && !data.ground_transformed_cloud->points.empty()) ||
        (data.non_horizontal_transformed_cloud && !data.non_horizontal_transformed_cloud->points.empty())) {
        processor.visualizeCombinedClouds(enable_visualization_flag, data.ground_transformed_cloud, data.non_horizontal_transformed_cloud, "Combined Transformed Clouds");
    }
}

static bool transformAllGroundCandidates(PipelineData& data) {
    if (data.ground_candidates && !data.ground_candidates->points.empty() &&
        data.ground_transformed_cloud && !data.ground_transformed_cloud->points.empty()) { // Check if main ground transformation was successful (indicated by non-empty ground_transformed_cloud and valid matrices)

        if(data.rotation_matrix.isIdentity() && data.translation_matrix.isIdentity() && data.main_ground_cluster->empty()){
             std::cout << "情報: 主地面クラスタが空のため、有効な変換行列がありません。全水平候補点の変換をスキップします。" << std::endl;
             data.all_ground_candidates_transformed_cloud->points.clear();
             return true;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_rotated_ground_candidates(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*data.ground_candidates, *temp_rotated_ground_candidates, data.rotation_matrix);
        pcl::transformPointCloud(*temp_rotated_ground_candidates, *data.all_ground_candidates_transformed_cloud, data.translation_matrix);
        std::cout << "全ての水平候補点の変換成功。変換後の点数: " << data.all_ground_candidates_transformed_cloud->size() << std::endl;
    } else {
        std::cout << "情報: 全ての水平候補点の変換をスキップ (元の候補がないか、主地面変換が失敗)。" << std::endl;
        data.all_ground_candidates_transformed_cloud->points.clear();
    }
    // Ensure cloud is valid even if empty
    if(data.all_ground_candidates_transformed_cloud->width == 0) {
        data.all_ground_candidates_transformed_cloud->height = 1;
        data.all_ground_candidates_transformed_cloud->is_dense = true;
    }
    return true;
}

static bool processNonHorizontalCloud(PipelineData& data, proc::PointCloudProcessor& processor) {
    if (data.non_horizontal_transformed_cloud && !data.non_horizontal_transformed_cloud->points.empty()) {
        float leaf_size = static_cast<float>(data.app_config.map_resolution);
        if (!processor.downsampleCloud(
                data.non_horizontal_transformed_cloud,
                leaf_size,
                leaf_size,
                leaf_size,
                data.app_config.outlier_removal_enable,      // New argument
                data.app_config.outlier_removal_mean_k,      // New argument
                static_cast<float>(data.app_config.outlier_removal_std_dev_mul_thresh), // New argument, ensure correct type
                data.non_horizontal_downsampled_cloud
            )) {
            std::cerr << "警告: 非水平要素のダウンサンプリングに失敗しました。" << std::endl;
            // Fallback: use non-downsampled, or clear if that's safer
            data.non_horizontal_downsampled_cloud = data.non_horizontal_transformed_cloud;
        } else {
             std::cout << "非水平要素のダウンサンプリング成功。処理後点数: " << data.non_horizontal_downsampled_cloud->size() << std::endl;
        }

        if (data.non_horizontal_downsampled_cloud && !data.non_horizontal_downsampled_cloud->points.empty()) {
            if (!processor.filterCloudByHeight(data.non_horizontal_downsampled_cloud, -1.0f, static_cast<float>(data.app_config.robot_height), data.non_horizontal_filtered_cloud)) {
                std::cerr << "警告: 非水平要素の高さフィルタリングに失敗しました。" << std::endl;
                 // Fallback: use non-filtered, or clear
                data.non_horizontal_filtered_cloud = data.non_horizontal_downsampled_cloud;
            } else {
                std::cout << "非水平要素の高さフィルタリング成功。処理後点数: " << data.non_horizontal_filtered_cloud->size() << std::endl;
            }
        } else {
            std::cout << "情報: ダウンサンプリング後の非水平要素が空のため、高さフィルタリングをスキップ。" << std::endl;
            data.non_horizontal_filtered_cloud->points.clear();
        }
    } else {
        std::cout << "情報: 変換後の非水平要素が空のため、ダウンサンプリングとフィルタリングをスキップ。" << std::endl;
        data.non_horizontal_downsampled_cloud->points.clear();
        data.non_horizontal_filtered_cloud->points.clear();
    }
    // Ensure clouds are valid even if empty
    if(data.non_horizontal_downsampled_cloud->width == 0) {
        data.non_horizontal_downsampled_cloud->height = 1; data.non_horizontal_downsampled_cloud->is_dense = true;
    }
    if(data.non_horizontal_filtered_cloud->width == 0) {
        data.non_horizontal_filtered_cloud->height = 1; data.non_horizontal_filtered_cloud->is_dense = true;
    }
    return true;
}

} // namespace detail
// --- End of detail namespace ---

bool processPointCloud(PipelineData& data, proc::PointCloudProcessor& processor) {
    std::cout << "\n--- 点群ブロック処理パイプライン開始 ---" << std::endl;

    if (!data.raw_cloud || data.raw_cloud->empty()) {
        std::cerr << "エラー: 入力点群 (raw_cloud) が空です。処理を中止します。" << std::endl;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_global_ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_global_obstacle_points(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*data.raw_cloud, min_pt, max_pt);
    double block_size = data.app_config.block_size;

    if (block_size <= 0) {
        std::cerr << "エラー: block_size が無効です (" << block_size << ")。正の値を設定してください。" << std::endl;
        return false;
    }

    int num_blocks_x = (max_pt.x > min_pt.x) ? static_cast<int>(std::ceil((max_pt.x - min_pt.x) / block_size)) : 1;
    int num_blocks_y = (max_pt.y > min_pt.y) ? static_cast<int>(std::ceil((max_pt.y - min_pt.y) / block_size)) : 1;
    if (data.raw_cloud->points.size() == 1) { // Handle single point cloud
        num_blocks_x = 1;
        num_blocks_y = 1;
    }

    if (data.app_config.force_single_block_processing) {
        std::cout << "情報: 'force_single_block_processing' が有効なため、点群全体を単一ブロックとして処理します。" << std::endl;
        num_blocks_x = 1;
        num_blocks_y = 1;
    }
    // Recalculate total_blocks and update the log message
    int total_blocks = num_blocks_x * num_blocks_y;
    if (data.app_config.force_single_block_processing) {
         std::cout << "点群全体を単一ブロック (1x1) として処理します。" << std::endl;
    } else {
        std::cout << "点群を " << num_blocks_x << "x" << num_blocks_y << " = " << total_blocks << " 個のブロックに分割して処理します。" << std::endl;
    }
    int processed_block_count = 0;

    for (int ix = 0; ix < num_blocks_x; ++ix) {
        for (int iy = 0; iy < num_blocks_y; ++iy) {
            processed_block_count++;
            std::cout << "\n処理中のブロック: " << processed_block_count << "/" << total_blocks << " (x_idx=" << ix << ", y_idx=" << iy << ")" << std::endl;

            double current_block_min_x, current_block_max_x, current_block_min_y, current_block_max_y;
            if (data.app_config.force_single_block_processing) {
                current_block_min_x = min_pt.x;
                current_block_max_x = max_pt.x;
                current_block_min_y = min_pt.y;
                current_block_max_y = max_pt.y;
            } else {
                current_block_min_x = min_pt.x + ix * block_size;
                current_block_max_x = min_pt.x + (ix + 1) * block_size;
                current_block_min_y = min_pt.y + iy * block_size;
                current_block_max_y = min_pt.y + (iy + 1) * block_size;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr block_raw_cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr block_raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            pcl::PassThrough<pcl::PointXYZ> pass_x;
            pass_x.setInputCloud(data.raw_cloud);
            pass_x.setFilterFieldName("x");
            pass_x.setFilterLimits(current_block_min_x, current_block_max_x);
            pass_x.filter(*block_raw_cloud_filtered_x);
            if (block_raw_cloud_filtered_x->empty()) {
                std::cout << "ブロック " << processed_block_count << " はXフィルタリング後に空になりました。スキップします。" << std::endl;
                continue;
            }

            pcl::PassThrough<pcl::PointXYZ> pass_y;
            pass_y.setInputCloud(block_raw_cloud_filtered_x);
            pass_y.setFilterFieldName("y");
            pass_y.setFilterLimits(current_block_min_y, current_block_max_y);
            pass_y.filter(*block_raw_cloud);
            if (block_raw_cloud->empty()) {
                std::cout << "ブロック " << processed_block_count << " はYフィルタリング後に空になりました。スキップします。" << std::endl;
                continue;
            }
            std::cout << "ブロック " << processed_block_count << " のフィルタリング後点群サイズ: " << block_raw_cloud->size() << " 点" << std::endl; // Changed log message slightly for clarity

            // New check for min_points_for_block_processing
            if (block_raw_cloud->points.size() < static_cast<size_t>(data.app_config.min_points_for_block_processing)) {
                std::cout << "ブロック " << processed_block_count << " の点数 (" << block_raw_cloud->points.size()
                          << ") が閾値 (" << data.app_config.min_points_for_block_processing
                          << ") 未満のため、処理をスキップします。" << std::endl;
                continue; // Skip to the next block
            }

            PipelineData block_pipeline_data;
            block_pipeline_data.app_config = data.app_config;
            block_pipeline_data.raw_cloud = block_raw_cloud;
            bool block_ok = true;

            if (block_ok && !detail::estimateNormals(block_pipeline_data, processor)) { std::cerr << "ブロック " << processed_block_count << ": 法線推定に失敗。" << std::endl; block_ok = false; }
            if (block_ok && !detail::extractGroundAndNonHorizontal(block_pipeline_data, processor)) { std::cerr << "ブロック " << processed_block_count << ": 地面/非水平要素の抽出に失敗。" << std::endl; block_ok = false; }
            if (block_ok && !detail::extractMainGroundCluster(block_pipeline_data, processor)) { std::cerr << "ブロック " << processed_block_count << ": 主地面クラスタの抽出に失敗。" << std::endl; block_ok = false; }
            if (block_ok && !detail::fitGlobalGroundPlane(block_pipeline_data, processor)) {
                 std::cout << "ブロック " << processed_block_count << ": 地面平面フィッティングに失敗。恒等変換を使用します。" << std::endl;
                 block_pipeline_data.rotation_matrix = Eigen::Matrix4f::Identity();
                 block_pipeline_data.translation_matrix = Eigen::Matrix4f::Identity();
            }
            if (block_ok && !detail::transformClouds(block_pipeline_data, processor)) { std::cerr << "ブロック " << processed_block_count << ": 点群変換に失敗。" << std::endl; block_ok = false; }
            if (block_ok && !detail::transformAllGroundCandidates(block_pipeline_data)) { std::cerr << "ブロック " << processed_block_count << ": 全地面候補の変換に失敗。" << std::endl; block_ok = false; }
            if (block_ok && !detail::processNonHorizontalCloud(block_pipeline_data, processor)) { std::cerr << "ブロック " << processed_block_count << ": 非水平要素の処理に失敗。" << std::endl; block_ok = false; }

            if (!block_ok) {
                std::cerr << "ブロック " << processed_block_count << " の処理中にエラーが発生したため、このブロックの結果はスキップされます。" << std::endl;
                continue;
            }

            Eigen::Matrix4f R_block = block_pipeline_data.rotation_matrix;
            Eigen::Matrix4f T_block_z0 = block_pipeline_data.translation_matrix;
            Eigen::Matrix4f R_block_inv = Eigen::Matrix4f::Identity();
            R_block_inv.block<3,3>(0,0) = R_block.block<3,3>(0,0).transpose();
            Eigen::Matrix4f T_block_z0_inv = Eigen::Matrix4f::Identity();
            T_block_z0_inv.block<3,1>(0,3) = -T_block_z0.block<3,1>(0,3);
            Eigen::Matrix4f global_transform_matrix = R_block_inv * T_block_z0_inv;

            if (block_pipeline_data.all_ground_candidates_transformed_cloud && !block_pipeline_data.all_ground_candidates_transformed_cloud->empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr block_ground_global(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*block_pipeline_data.all_ground_candidates_transformed_cloud, *block_ground_global, global_transform_matrix);
                *combined_global_ground_points += *block_ground_global;
            }
            if (block_pipeline_data.non_horizontal_filtered_cloud && !block_pipeline_data.non_horizontal_filtered_cloud->empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr block_obstacles_global(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*block_pipeline_data.non_horizontal_filtered_cloud, *block_obstacles_global, global_transform_matrix);
                *combined_global_obstacle_points += *block_obstacles_global;
            }
            std::cout << "ブロック " << processed_block_count << " の処理完了。収集されたグローバル地面点 (累計): " << combined_global_ground_points->size() << ", 障害物点 (累計): " << combined_global_obstacle_points->size() << std::endl;
        }
    }

    data.all_ground_candidates_transformed_cloud = combined_global_ground_points;
    data.non_horizontal_filtered_cloud = combined_global_obstacle_points;

    // Clear intermediate Ptr members in the main 'data' object
    if(data.non_horizontal_downsampled_cloud) data.non_horizontal_downsampled_cloud->clear();
    if(data.ground_transformed_cloud) data.ground_transformed_cloud->clear();
    if(data.main_ground_cluster) data.main_ground_cluster->clear();
    if(data.normals) data.normals->clear();
    if(data.ground_candidates) data.ground_candidates->clear();
    if(data.ground_candidate_indices) data.ground_candidate_indices->indices.clear();
    if(data.non_horizontal_cloud) data.non_horizontal_cloud->clear();
    if(data.plane_coefficients) data.plane_coefficients->values.clear();
    if(data.plane_inliers) data.plane_inliers->indices.clear();
    data.rotation_matrix = Eigen::Matrix4f::Identity();
    data.translation_matrix = Eigen::Matrix4f::Identity();

    std::cout << "全ブロック処理完了。最終的なグローバル地面点群サイズ: " << (data.all_ground_candidates_transformed_cloud ? data.all_ground_candidates_transformed_cloud->size() : 0) << std::endl;
    std::cout << "最終的なグローバル障害物点群サイズ: " << (data.non_horizontal_filtered_cloud ? data.non_horizontal_filtered_cloud->size() : 0) << std::endl;

    // Final visualization of combined clouds
    detail::visualizeCombinedTransformedClouds(data.app_config.enable_visualization, data, processor);

    std::cout << "--- 点群ブロック処理パイプライン終了 --- \n" << std::endl;
    return true;
}


bool generateOccupancyGrid(PipelineData& data) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_for_map_bounds(new pcl::PointCloud<pcl::PointXYZ>());
    std::cout << "\n--- 地図範囲定義のための結合点群作成開始 ---" << std::endl;

    if (data.all_ground_candidates_transformed_cloud && !data.all_ground_candidates_transformed_cloud->points.empty()) {
        *combined_for_map_bounds += *data.all_ground_candidates_transformed_cloud;
        std::cout << "  - 追加された変換済みの全地面候補点群: " << data.all_ground_candidates_transformed_cloud->size() << " 点" << std::endl;
    }
    if (data.non_horizontal_filtered_cloud && !data.non_horizontal_filtered_cloud->points.empty()) {
        *combined_for_map_bounds += *data.non_horizontal_filtered_cloud;
        std::cout << "  - 追加された障害物点群: " << data.non_horizontal_filtered_cloud->size() << " 点" << std::endl;
    }

    if (combined_for_map_bounds->points.empty()) {
        std::cout << "  - 注意: 結合後の地図範囲定義用点群は空です。地図は空になる可能性があります。" << std::endl;
    }
    combined_for_map_bounds->width = combined_for_map_bounds->points.size();
    combined_for_map_bounds->height = 1;
    combined_for_map_bounds->is_dense = true;
    std::cout << "  - 結合後の総点数 (combined_for_map_bounds): " << combined_for_map_bounds->size() << std::endl;
    std::cout << "--- 地図範囲定義のための結合点群作成終了 ---" << std::endl;

    if (!map_params_util::calculateMapParameters(combined_for_map_bounds, data.app_config.map_resolution, data.map_params)) {
        std::cerr << "エラー: 地図パラメータの計算に失敗しました。" << std::endl;
        return false;
    }
    std::cout << "計算された地図パラメータ:" << std::endl;
    std::cout << "  原点X: " << data.map_params.origin_x << " [m], Y: " << data.map_params.origin_y << " [m]" << std::endl;
    std::cout << "  幅: " << data.map_params.width_pixels << " [ピクセル], 高さ: " << data.map_params.height_pixels << " [ピクセル]" << std::endl;

    try {
        size_t grid_size = 0;
        if (data.map_params.width_pixels > 0 && data.map_params.height_pixels > 0) {
             grid_size = static_cast<size_t>(data.map_params.width_pixels) * static_cast<size_t>(data.map_params.height_pixels);
        } else {
            std::cout << "情報: 地図の次元が0のため、空の占有格子地図を作成します。" << std::endl;
        }
        data.occupancy_grid.assign(grid_size, map_io_util::GRID_VALUE_UNKNOWN);
    } catch (const std::exception& e) {
        std::cerr << "エラー: 地図データ構造のためのメモリ確保に失敗: " << e.what() << std::endl;
        return false;
    }
    std::cout << "占有格子地図を初期化。サイズ: " << data.occupancy_grid.size() << std::endl;

    if (data.all_ground_candidates_transformed_cloud && !data.all_ground_candidates_transformed_cloud->points.empty() && !data.occupancy_grid.empty()) {
        std::cout << "空き領域プロット中..." << std::endl;
        for (const auto& point : data.all_ground_candidates_transformed_cloud->points) {
            int map_x = static_cast<int>(std::floor((point.x - data.map_params.origin_x) / data.map_params.resolution));
            int map_y_pgm = data.map_params.height_pixels - 1 - static_cast<int>(std::floor((point.y - data.map_params.origin_y) / data.map_params.resolution));
            if (map_x >= 0 && map_x < data.map_params.width_pixels && map_y_pgm >= 0 && map_y_pgm < data.map_params.height_pixels) {
                data.occupancy_grid[static_cast<size_t>(map_y_pgm) * data.map_params.width_pixels + map_x] = map_io_util::GRID_VALUE_FREE;
            }
        }
    }
    if (data.non_horizontal_filtered_cloud && !data.non_horizontal_filtered_cloud->points.empty() && !data.occupancy_grid.empty()) {
        std::cout << "障害物プロット中..." << std::endl;
        for (const auto& point : data.non_horizontal_filtered_cloud->points) {
            int map_x = static_cast<int>(std::floor((point.x - data.map_params.origin_x) / data.map_params.resolution));
            int map_y_pgm = data.map_params.height_pixels - 1 - static_cast<int>(std::floor((point.y - data.map_params.origin_y) / data.map_params.resolution));
             if (map_x >= 0 && map_x < data.map_params.width_pixels && map_y_pgm >= 0 && map_y_pgm < data.map_params.height_pixels) {
                data.occupancy_grid[static_cast<size_t>(map_y_pgm) * data.map_params.width_pixels + map_x] = map_io_util::GRID_VALUE_OCCUPIED;
            }
        }
    }
    std::cout << "--- 占有格子地図生成完了 ---" << std::endl;

    // --- Apply Map Processing (Free Space and Obstacle Filling) ---
    std::cout << "\n--- 後処理 (フリースペース・障害物充填) 開始 ---" << std::endl;
    MapProcessor map_proc;

    // 1. Create OccupancyGrid object for MapProcessor
    OccupancyGrid grid_for_processing;
    grid_for_processing.info.width = data.map_params.width_pixels;
    grid_for_processing.info.height = data.map_params.height_pixels;
    grid_for_processing.info.resolution = static_cast<float>(data.map_params.resolution);
    // The data.occupancy_grid already uses -1 (unknown), 0 (free), 100 (occupied)
    grid_for_processing.data = data.occupancy_grid;

    std::cout << "  - OccupancyGrid for processing created. Width: " << grid_for_processing.info.width
              << ", Height: " << grid_for_processing.info.height << ", Data size: " << grid_for_processing.data.size() << std::endl;

    if (grid_for_processing.data.empty() || grid_for_processing.info.width == 0 || grid_for_processing.info.height == 0) {
        std::cout << "  - 注意: 地図データが空のため、後処理をスキップします。" << std::endl;
    } else {
        // 2. Convert to cv::Mat
        cv::Mat cv_map = map_proc.toCvMat(grid_for_processing);
        std::cout << "  - OccupancyGridをcv::Matに変換完了。Rows: " << cv_map.rows << ", Cols: " << cv_map.cols << std::endl;

        // 3. Fill Free Space
        cv::Mat processed_free_map = map_proc.fillFreeSpace(cv_map, data.app_config.free_space_kernel_size);
        std::cout << "  - フリースペース充填完了。Kernel size: " << data.app_config.free_space_kernel_size << std::endl;

        // 4. Fill Obstacle Space
        cv::Mat processed_obstacle_map = map_proc.fillObstacleSpace(processed_free_map, data.app_config.obstacle_space_kernel_size);
        std::cout << "  - 障害物スペース充填完了。Kernel size: " << data.app_config.obstacle_space_kernel_size << std::endl;

        // 5. Convert back to OccupancyGrid data
        // Pass original grid_for_processing to keep its info (width, height, resolution)
        data.occupancy_grid = map_proc.toOccupancyGridData(processed_obstacle_map, grid_for_processing);
        std::cout << "  - cv::MatをOccupancyGridデータに再変換完了。" << std::endl;
    }
    std::cout << "--- 後処理 (フリースペース・障害物充填) 完了 ---" << std::endl;

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
    if (data.app_config.enable_visualization) {
        std::cout << "\n情報: 地図プレビューは有効ですが、この環境では表示されません。" << pgm_file_path << std::endl;
    }
    return true;
}

} // namespace pipeline
