#include "point_cloud_processor.h"
// #include "config_loader.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h> // For pcl::SACSegmentation


namespace proc {

PointCloudProcessor::PointCloudProcessor() : tree_(new pcl::search::KdTree<pcl::PointXYZ>()) {
    std::cout << "PointCloudProcessor initialized (KdTree)." << std::endl;
}

PointCloudProcessor::~PointCloudProcessor() {
    std::cout << "PointCloudProcessor destroyed." << std::endl;
}

bool PointCloudProcessor::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
    float search_radius,
    pcl::PointCloud<pcl::Normal>::Ptr& output_normals) {

    if (!input_cloud || input_cloud->points.empty()) {
        std::cerr << "エラー (NormalEstimation): 入力点群が空または無効です。" << std::endl;
        return false;
    }
    if (!output_normals) {
        std::cerr << "エラー (NormalEstimation): 出力法線用のクラウドポインタが無効 (null) です。" << std::endl;
        return false;
    }
    output_normals->clear();

    if (search_radius <= 0) {
        std::cerr << "エラー (NormalEstimation): 探索半径は正の値でなければなりません。指定値: " << search_radius << std::endl;
        return false;
    }

    std::cout << "法線推定を開始します... 入力点数: " << input_cloud->size()
              << ", 探索半径: " << search_radius << std::endl;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    tree_->setInputCloud(input_cloud);
    ne.setSearchMethod(tree_);
    ne.setRadiusSearch(search_radius);

    try {
        ne.compute(*output_normals);
    } catch (const std::exception& e) {
        std::cerr << "エラー (NormalEstimation): 法線推定中に例外が発生しました: " << e.what() << std::endl;
        return false;
    }

    if (output_normals->points.empty() && !input_cloud->points.empty()) {
        std::cerr << "警告 (NormalEstimation): 法線推定の結果が空ですが、入力点群には点が含まれていました。" << std::endl;
    } else if (output_normals->points.size() != input_cloud->points.size()) {
        std::cerr << "警告 (NormalEstimation): 出力された法線の数が入力点群の点の数と異なります。" << std::endl;
        std::cerr << "  出力法線数: " << output_normals->points.size() << ", 入力点数: " << input_cloud->points.size() << std::endl;
    }

    int nan_normals = 0;
    for(size_t i = 0; i < output_normals->points.size(); ++i) {
        if (!pcl::isFinite(output_normals->points[i])) {
            nan_normals++;
        }
    }
    if (nan_normals > 0) {
        std::cout << "警告 (NormalEstimation): 推定された法線の中に " << nan_normals
                  << " 個のNaN値または非有限値が見つかりました (総法線数: "
                  << output_normals->points.size() << ")。" << std::endl;
    }

    std::cout << "法線推定が完了しました。出力法線数: " << output_normals->points.size() << std::endl;
    return true;
}

bool PointCloudProcessor::extractGroundCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
    const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
    float ground_normal_z_thresh,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_candidates_cloud) {

    if (!input_cloud || input_cloud->points.empty()) {
        std::cerr << "エラー (GroundCandidates): 入力点群が空または無効です。" << std::endl;
        return false;
    }
    if (!normals || normals->points.empty()) {
        std::cerr << "エラー (GroundCandidates): 法線情報が空または無効です。" << std::endl;
        return false;
    }
    if (input_cloud->points.size() != normals->points.size()) {
        std::cerr << "エラー (GroundCandidates): 入力点群と法線情報の点数が一致しません。" << std::endl;
        std::cerr << "  入力点数: " << input_cloud->points.size() << ", 法線数: " << normals->points.size() << std::endl;
        return false;
    }
    if (!ground_candidates_cloud) {
        std::cerr << "エラー (GroundCandidates): 地面候補点群ポインタが無効 (null) です。" << std::endl;
        return false;
    }
    ground_candidates_cloud->clear();

    if (ground_normal_z_thresh < 0.0f || ground_normal_z_thresh > 1.0f) {
         std::cerr << "警告 (GroundCandidates): ground_normal_z_thresh (" << ground_normal_z_thresh
                   << ") が不正な範囲です。通常は [0, 1] の範囲で指定します。" << std::endl;
    }

    std::cout << "地面候補点の抽出を開始します... Z法線閾値 (絶対値比較): " << ground_normal_z_thresh << std::endl;

    ground_candidates_cloud->points.reserve(input_cloud->points.size());

    for (size_t i = 0; i < input_cloud->points.size(); ++i) {
        const auto& point = input_cloud->points[i];
        const auto& normal = normals->points[i];

        if (!pcl::isFinite(normal)) {
            continue;
        }
        if (std::abs(normal.normal_z) > ground_normal_z_thresh) {
            ground_candidates_cloud->points.push_back(point);
        }
    }

    ground_candidates_cloud->width = ground_candidates_cloud->points.size();
    ground_candidates_cloud->height = 1;
    ground_candidates_cloud->is_dense = true;

    std::cout << "地面候補点の抽出が完了しました。抽出された点数: " << ground_candidates_cloud->points.size() << std::endl;

    if (ground_candidates_cloud->points.empty()){
        std::cout << "警告 (GroundCandidates): 地面候補点が抽出されませんでした。" << std::endl;
    }
    return true;
}

bool PointCloudProcessor::extractMainGroundCluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground_candidates_cloud,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& main_ground_cluster_cloud) {

    if (!ground_candidates_cloud) {
        std::cerr << "エラー (MainCluster): 入力された地面候補点群ポインタがnullです。" << std::endl;
        return false;
    }
    if (!main_ground_cluster_cloud) {
        std::cerr << "エラー (MainCluster): 主地面クラスタ出力用の点群ポインタがnullです。" << std::endl;
        return false;
    }
    main_ground_cluster_cloud->clear();

    if (ground_candidates_cloud->points.empty()) {
        std::cout << "情報 (MainCluster): 地面候補点群が空のため、クラスタリングをスキップします。" << std::endl;
        main_ground_cluster_cloud->width = 0;
        main_ground_cluster_cloud->height = 1;
        main_ground_cluster_cloud->is_dense = true;
        return true;
    }

    std::cout << "主地面クラスタの特定を開始します... 入力候補点数: " << ground_candidates_cloud->size()
              << ", 許容距離: " << cluster_tolerance
              << ", 最小クラスタサイズ: " << min_cluster_size
              << ", 最大クラスタサイズ: " << max_cluster_size << std::endl;

    tree_->setInputCloud(ground_candidates_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree_);
    ec.setInputCloud(ground_candidates_cloud);

    try {
        ec.extract(cluster_indices);
    } catch (const std::exception& e) {
        std::cerr << "エラー (MainCluster): クラスタ抽出中に例外が発生しました: " << e.what() << std::endl;
        return false;
    }

    if (cluster_indices.empty()) {
        std::cout << "警告 (MainCluster): 地面候補点から条件に合うクラスタが見つかりませんでした。" << std::endl;
        main_ground_cluster_cloud->width = 0;
        main_ground_cluster_cloud->height = 1;
        main_ground_cluster_cloud->is_dense = true;
        return true;
    }

    std::cout << "発見されたクラスタ数: " << cluster_indices.size() << std::endl;

    size_t largest_cluster_size = 0;
    int largest_cluster_idx = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (cluster_indices[i].indices.size() > largest_cluster_size) {
            largest_cluster_size = cluster_indices[i].indices.size();
            largest_cluster_idx = static_cast<int>(i);
        }
    }

    std::cout << "最大のクラスタのインデックス: " << largest_cluster_idx << ", 点数: " << largest_cluster_size << std::endl;

    pcl::PointIndices::Ptr main_cluster_indices(new pcl::PointIndices(cluster_indices[largest_cluster_idx]));

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(ground_candidates_cloud);
    extract.setIndices(main_cluster_indices);
    extract.setNegative(false);
    extract.filter(*main_ground_cluster_cloud);

    std::cout << "主地面クラスタの特定が完了しました。抽出された点数: " << main_ground_cluster_cloud->points.size() << std::endl;
    return true;
}

bool PointCloudProcessor::fitGlobalGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& main_ground_cluster_cloud,
    float distance_threshold,
    pcl::ModelCoefficients::Ptr& output_plane_coefficients,
    pcl::PointIndices::Ptr& output_plane_inliers) {

    if (!main_ground_cluster_cloud) {
        std::cerr << "エラー (PlaneFitting): 入力された主地面クラスタ点群がnullです。" << std::endl;
        return false;
    }
    if (!output_plane_coefficients || !output_plane_inliers) {
        std::cerr << "エラー (PlaneFitting): 出力用の係数またはインライアのポインタがnullです。" << std::endl;
        return false;
    }
    // Clear previous results
    output_plane_coefficients->values.clear();
    output_plane_inliers->indices.clear();

    if (main_ground_cluster_cloud->points.size() < 3) {
        std::cout << "情報 (PlaneFitting): 主地面クラスタの点数が少なすぎる ("
                  << main_ground_cluster_cloud->points.size()
                  << "点) ため、平面フィッティングをスキップします。" << std::endl;
        return false; // Not enough points to define a plane
    }

    std::cout << "グローバル地面平面のフィッティングを開始します... 入力点数: " << main_ground_cluster_cloud->size()
              << ", 距離閾値: " << distance_threshold << std::endl;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    // seg.setMaxIterations(1000); // Consider making this configurable

    seg.setInputCloud(main_ground_cluster_cloud);

    try {
        seg.segment(*output_plane_inliers, *output_plane_coefficients);
    } catch (const pcl::PCLException& e) { // Catch PCL specific exceptions
        std::cerr << "エラー (PlaneFitting): PCL平面セグメンテーション中に例外が発生しました: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) { // Catch other standard exceptions
        std::cerr << "エラー (PlaneFitting): 平面セグメンテーション中に一般的な例外が発生しました: " << e.what() << std::endl;
        return false;
    }

    if (output_plane_inliers->indices.empty()) {
        std::cout << "警告 (PlaneFitting): 地面平面のフィッティング後、インライアが見つかりませんでした。" << std::endl;
        return false;
    }

    if (output_plane_coefficients->values.empty() || output_plane_coefficients->values.size() != 4) {
        std::cout << "警告 (PlaneFitting): 地面平面のフィッティング後、有効な平面係数 (4つ) が得られませんでした。" << std::endl;
        if (!output_plane_coefficients->values.empty()) {
            std::cout << "  得られた係数 (" << output_plane_coefficients->values.size() << "個): ";
            for(float val : output_plane_coefficients->values) std::cout << val << " ";
            std::cout << std::endl;
        }
        return false;
    }

    std::cout << "グローバル地面平面のフィッティングが完了しました。" << std::endl;
    std::cout << "  平面インライア数: " << output_plane_inliers->indices.size() << std::endl;
    std::cout << "  平面モデル係数 (A, B, C, D): "
              << output_plane_coefficients->values[0] << ", "
              << output_plane_coefficients->values[1] << ", "
              << output_plane_coefficients->values[2] << ", "
              << output_plane_coefficients->values[3] << std::endl;

    return true;
}

} // namespace proc
