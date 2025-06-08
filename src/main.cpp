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
#include <pcl/common/transforms.h> // For pcl::transformPointCloud
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
    pcl::PointIndices::Ptr ground_candidate_indices(new pcl::PointIndices); // Declare indices
    if (!processor.extractGroundCandidates(cloud, normals, static_cast<float>(app_config.ground_normal_z_threshold), ground_candidates, ground_candidate_indices)) {
        std::cerr << "エラー: 地面候補点の抽出に失敗しました。プログラムを終了します。" << std::endl;
        return 1;
    }
    std::cout << "地面候補点の抽出成功。候補点数: " << ground_candidates->size() << ", インデックス数: " << ground_candidate_indices->indices.size() << std::endl;

    // Extract non-horizontal points
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // Declare earlier for wider scope
    if (processor.extractNonHorizontalPoints(cloud, ground_candidate_indices, non_horizontal_cloud)) {
        std::cout << "非水平要素の抽出成功。非水平点群の点数: " << non_horizontal_cloud->size() << std::endl;
        // Optionally, visualize non_horizontal_cloud here if needed for debugging
        // processor.visualizeCloud(non_horizontal_cloud, "Non-Horizontal Points");
    } else {
        std::cerr << "警告: 非水平要素の抽出に失敗しました。" << std::endl;
    }

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
                            // processor.visualizeCloud(translated_cloud, "Translated Cloud (Z=0)"); // Commented out

                            // Now, transform the non-horizontal points using the same matrices
                            if (non_horizontal_cloud && !non_horizontal_cloud->points.empty()) {
                                std::cout << "\n--- 非水平要素の変換処理開始 ---" << std::endl;
                                pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                                // Apply the rotation
                                pcl::transformPointCloud(*non_horizontal_cloud, *non_horizontal_rotated_cloud, rotation_matrix);

                                // Apply the translation (non_horizontal_transformed_cloud was declared earlier)
                                pcl::transformPointCloud(*non_horizontal_rotated_cloud, *non_horizontal_transformed_cloud, translation_matrix);

                                std::cout << "非水平要素の変換成功。変換後の点数: " << non_horizontal_transformed_cloud->size() << std::endl;
                                // Visualization of non_horizontal_transformed_cloud will be in the next step
                                std::cout << "--- 非水平要素の変換処理終了 ---" << std::endl;

                                // Call the new combined visualization
                                processor.visualizeCombinedClouds(translated_cloud, non_horizontal_transformed_cloud, "Combined Transformed Clouds");

                                // Declare downsampled cloud pointer before the conditional block
                                pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                                // Declare filtered cloud pointer before the conditional block as well
                                pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

                                if (non_horizontal_transformed_cloud && !non_horizontal_transformed_cloud->points.empty()) {
                                    std::cout << "\n--- 非水平要素のダウンサンプリング開始 ---" << std::endl;
                                    float leaf_size = static_cast<float>(app_config.map_resolution);
                                    if (!processor.downsampleCloud(non_horizontal_transformed_cloud, leaf_size, leaf_size, leaf_size, non_horizontal_downsampled_cloud)) {
                                        std::cerr << "警告: 非水平要素のダウンサンプリングに失敗しました。以降の処理ではダウンサンプリングされていない非水平要素を使用します。" << std::endl;
                                        // Fallback: Copy original to downsampled if downsampling fails, to ensure subsequent steps have some input.
                                        // However, the current downsampleCloud implementation clears the output on failure,
                                        // so non_horizontal_downsampled_cloud will be empty if it returns false.
                                        // This is acceptable if subsequent steps can handle an empty cloud.
                                    } else {
                                        std::cout << "非水平要素のダウンサンプリング成功。処理前点数: " << non_horizontal_transformed_cloud->size()
                                                  << ", 処理後点数: " << non_horizontal_downsampled_cloud->size() << std::endl;
                                    }
                                    std::cout << "--- 非水平要素のダウンサンプリング終了 ---" << std::endl;
                                } else {
                                    std::cout << "情報: 変換後の非水平要素の点群が空のため、ダウンサンプリングをスキップします。" << std::endl;
                                    // Ensure non_horizontal_downsampled_cloud is empty and valid if it was not processed
                                    if (non_horizontal_downsampled_cloud->points.empty() && non_horizontal_downsampled_cloud->header.frame_id.empty()) {
                                         non_horizontal_downsampled_cloud->width = 0;
                                         non_horizontal_downsampled_cloud->height = 1;
                                         non_horizontal_downsampled_cloud->is_dense = true;
                                    }
                                }
                                // Subsequent steps should use non_horizontal_downsampled_cloud

                                // Filter the downsampled cloud by height
                                // pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // Moved up
                                if (non_horizontal_downsampled_cloud && !non_horizontal_downsampled_cloud->points.empty()) {
                                    std::cout << "\n--- 非水平要素の高さフィルタリング開始 ---" << std::endl;
                                    float min_z_filter = -1.0f;
                                    float max_z_filter = static_cast<float>(app_config.robot_height);
                                    if (!processor.filterCloudByHeight(non_horizontal_downsampled_cloud, min_z_filter, max_z_filter, non_horizontal_filtered_cloud)) {
                                        std::cerr << "警告: 非水平要素の高さフィルタリングに失敗しました。" << std::endl;
                                        // non_horizontal_filtered_cloud will be empty if filtering fails
                                    } else {
                                        std::cout << "非水平要素の高さフィルタリング成功。処理前点数: " << non_horizontal_downsampled_cloud->size()
                                                  << ", 処理後点数: " << non_horizontal_filtered_cloud->size() << std::endl;
                                    }
                                    std::cout << "--- 非水平要素の高さフィルタリング終了 ---" << std::endl;
                                } else {
                                    std::cout << "情報: ダウンサンプリング後の非水平要素の点群が空のため、高さフィルタリングをスキップします。" << std::endl;
                                    if (non_horizontal_filtered_cloud->points.empty() && non_horizontal_filtered_cloud->header.frame_id.empty()) {
                                        non_horizontal_filtered_cloud->width = 0;
                                        non_horizontal_filtered_cloud->height = 1;
                                        non_horizontal_filtered_cloud->is_dense = true;
                                    }
                                }
                                // Subsequent steps (like occupancy grid population) should use non_horizontal_filtered_cloud.

                            } else {
                                std::cout << "情報: 非水平要素の点群が空のため、変換をスキップします。" << std::endl;
                                // Still visualize the translated ground if non-horizontal is empty
                                processor.visualizeCombinedClouds(translated_cloud, non_horizontal_transformed_cloud, "Combined Transformed Clouds (Ground Only)");
                                // Ensure the pre-declared non_horizontal_downsampled_cloud is properly initialized as empty
                                non_horizontal_downsampled_cloud->width = 0;
                                non_horizontal_downsampled_cloud->height = 1;
                                non_horizontal_downsampled_cloud->is_dense = true;
                                std::cout << "情報: 非水平要素が空だったため、対応するダウンサンプリング済み点群も空として初期化されました。" << std::endl;
                                // Also ensure the pre-declared filtered cloud is empty and valid in this path
                                // pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>()); // Removed, it's declared outside
                                non_horizontal_filtered_cloud->width = 0;
                                non_horizontal_filtered_cloud->height = 1;
                                non_horizontal_filtered_cloud->is_dense = true;
                                std::cout << "情報: 非水平要素が空だったため、対応する高さフィルタリング済み点群も空として初期化されました。" << std::endl;


                            }
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

    // MOVED DECLARATIONS for map_params and occupancy_grid
   map_params_util::MapParameters map_params;
   std::vector<int8_t> occupancy_grid;

   // Create a combined point cloud for calculating map bounds in the stabilized coordinate system
   pcl::PointCloud<pcl::PointXYZ>::Ptr combined_for_map_bounds(new pcl::PointCloud<pcl::PointXYZ>());
   std::cout << "\n--- 地図範囲定義のための結合点群作成開始 ---" << std::endl;

   if (translated_cloud && !translated_cloud->points.empty()) {
       *combined_for_map_bounds += *translated_cloud;
       std::cout << "  - 追加された地面点群 (translated_cloud) の点数: " << translated_cloud->size() << std::endl;
   } else {
       std::cout << "  - 地面点群 (translated_cloud) は空か無効です。" << std::endl;
   }

   // Note: non_horizontal_filtered_cloud is used here, which is declared within the
   // 'if (non_horizontal_cloud && !non_horizontal_cloud->points.empty())' block.
   // This means if that block was skipped, non_horizontal_filtered_cloud might not be in scope or initialized.
   // The prompt's logic for initializing non_horizontal_filtered_cloud in the 'else' path for non_horizontal_cloud
   // handles this, ensuring it's always declared.
   if (non_horizontal_filtered_cloud && !non_horizontal_filtered_cloud->points.empty()) {
       *combined_for_map_bounds += *non_horizontal_filtered_cloud;
       std::cout << "  - 追加された障害物点群 (non_horizontal_filtered_cloud) の点数: " << non_horizontal_filtered_cloud->size() << std::endl;
   } else {
       std::cout << "  - 障害物点群 (non_horizontal_filtered_cloud) は空か無効です。" << std::endl;
   }

   if (combined_for_map_bounds->points.empty()) {
        std::cout << "  - 結合後の点群 (combined_for_map_bounds) は空です。" << std::endl;
        combined_for_map_bounds->width = 0;
        combined_for_map_bounds->height = 1;
        combined_for_map_bounds->is_dense = true; // Validly empty
   } else {
        // Set header, width, height, and is_dense for the combined cloud
        // Attempt to copy header from one of the source clouds if they have a frame_id
        if (translated_cloud && !translated_cloud->points.empty() && !translated_cloud->header.frame_id.empty()) {
            combined_for_map_bounds->header = translated_cloud->header;
        } else if (non_horizontal_filtered_cloud && !non_horizontal_filtered_cloud->points.empty() && !non_horizontal_filtered_cloud->header.frame_id.empty()) {
            combined_for_map_bounds->header = non_horizontal_filtered_cloud->header;
        } else if (cloud && !cloud->header.frame_id.empty()) {
            // Fallback to original cloud's frame_id if others are not available or empty.
            // This is just for frame_id, coordinates are in the transformed system.
            combined_for_map_bounds->header.frame_id = cloud->header.frame_id;
        }
        combined_for_map_bounds->width = combined_for_map_bounds->points.size();
        combined_for_map_bounds->height = 1;
        combined_for_map_bounds->is_dense = true; // Assuming points are finite after prior processing
       std::cout << "  - 結合後の総点数 (combined_for_map_bounds): " << combined_for_map_bounds->size() << std::endl;
   }
   std::cout << "--- 地図範囲定義のための結合点群作成終了 ---" << std::endl;

   // MAP PARAMETER CALCULATION (MOVED HERE and MODIFIED to use combined_for_map_bounds)
   std::cout << "\n--- 地図パラメータ計算開始 (結合点群基準) ---" << std::endl;
   if (!map_params_util::calculateMapParameters(combined_for_map_bounds, app_config.map_resolution, map_params)) {
       std::cerr << "エラー: 地図パラメータの計算に失敗しました。入力となる結合点群が空であるか、解像度が不適切である可能性があります。プログラムを終了します。" << std::endl;
       return 1; // Exits if combined_for_map_bounds is empty or other error in calculation
   }
   std::cout << "計算された地図パラメータ (結合点群基準):" << std::endl;
   std::cout << "  原点X: " << map_params.origin_x << " [m], Y: " << map_params.origin_y << " [m]" << std::endl;
   std::cout << "  幅: " << map_params.width_pixels << " [ピクセル], 高さ: " << map_params.height_pixels << " [ピクセル]" << std::endl;
   std::cout << "  解像度: " << map_params.resolution << " [m/pixel]" << std::endl;
   std::cout << "--- 地図パラメータ計算終了 ---" << std::endl;

   // OCCUPANCY GRID INITIALIZATION (MOVED HERE)
   std::cout << "\n--- 占有格子地図の初期化開始 ---" << std::endl;
   try {
       size_t grid_size = 0;
       if (map_params.width_pixels > 0 && map_params.height_pixels > 0) {
            grid_size = static_cast<size_t>(map_params.width_pixels) * static_cast<size_t>(map_params.height_pixels);
       } else {
            std::cout << "情報: 地図の幅または高さが0以下です (" << map_params.width_pixels << "x" << map_params.height_pixels
                      << ")。空の占有格子地図を作成します。" << std::endl;
       }
       occupancy_grid.assign(grid_size, map_io_util::GRID_VALUE_UNKNOWN);
   } catch (const std::bad_alloc& e) {
       std::cerr << "エラー: 地図データ構造のためのメモリ確保に失敗 ("
                 << (static_cast<size_t>(map_params.width_pixels) * static_cast<size_t>(map_params.height_pixels) * sizeof(int8_t))
                 << " bytes requested): " << e.what() << std::endl;
       return 1;
   } catch (const std::length_error& e) {
        std::cerr << "エラー: 地図データ構造のサイズが大きすぎます ("
                  << (static_cast<size_t>(map_params.width_pixels) * static_cast<size_t>(map_params.height_pixels))
                  << " elements): " << e.what() << std::endl;
        return 1;
   }
   std::cout << "占有格子地図を初期化。リクエストサイズ: " << map_params.width_pixels << "x" << map_params.height_pixels
             << ", 実際のグリッド要素数: " << occupancy_grid.size()
             << ", 初期値: " << static_cast<int>(map_io_util::GRID_VALUE_UNKNOWN) << std::endl;
   std::cout << "--- 占有格子地図の初期化終了 ---" << std::endl;

   // OCCUPANCY GRID POPULATION
   if (non_horizontal_filtered_cloud && !non_horizontal_filtered_cloud->points.empty()) {
       if (map_params.width_pixels > 0 && map_params.height_pixels > 0 && !occupancy_grid.empty()) {
           std::cout << "\n--- 占有格子地図への障害物プロット開始 ---" << std::endl;
           std::cout << "処理対象の点群 (フィルタリング済み非水平要素): " << non_horizontal_filtered_cloud->size() << " 点" << std::endl;
           int obstacles_marked = 0;
           int points_outside_map = 0;

           for (const auto& point : non_horizontal_filtered_cloud->points) {
               int map_x = static_cast<int>(std::floor((point.x - map_params.origin_x) / map_params.resolution));
               int map_y_ros = static_cast<int>(std::floor((point.y - map_params.origin_y) / map_params.resolution));
               int map_y_pgm = map_params.height_pixels - 1 - map_y_ros;

               if (map_x >= 0 && map_x < map_params.width_pixels &&
                   map_y_pgm >= 0 && map_y_pgm < map_params.height_pixels) {
                   size_t index = static_cast<size_t>(map_y_pgm) * map_params.width_pixels + map_x;
                   if (index < occupancy_grid.size()) { // Safety check
                       occupancy_grid[index] = map_io_util::GRID_VALUE_OCCUPIED;
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
            std::cout << "情報: 地図の次元が無効 (" << map_params.width_pixels << "x" << map_params.height_pixels
                      << ") または占有格子が空のため、障害物プロットをスキップします。" << std::endl;
       }
   } else {
       std::cout << "\n情報: プロット対象の障害物点群が空のため、占有格子地図への障害物プロットをスキップします。" << std::endl;
   }

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

   if (app_config.preview_map_on_exit) {
       std::cout << "\n情報: 地図のプレビュー表示が設定で有効になっています。" << std::endl;
       std::cout << "      しかし、このバージョンでは直接的なGUIプレビュー機能は実装されていません。" << std::endl;
       std::cout << "      保存された地図ファイルを確認してください: " << pgm_file_path << std::endl;
       // If a simple command-line viewer invocation were desired later, it would go here.
       // For now, just a message.
   }

    std::cout << "プログラムは正常に終了しました。" << std::endl;
    return 0;
}
