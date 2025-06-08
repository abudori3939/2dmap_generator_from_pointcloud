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

void PointCloudProcessor::visualizePlane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const pcl::ModelCoefficients::ConstPtr& plane_coefficients) {

    if (!cloud) {
        std::cerr << "VisualizePlane: Input cloud is null." << std::endl;
        return;
    }
    if (!plane_coefficients) {
        std::cerr << "VisualizePlane: Plane coefficients are null." << std::endl;
        return;
    }
    if (plane_coefficients->values.size() != 4) {
        std::cerr << "VisualizePlane: Invalid plane coefficients (size != 4)." << std::endl;
        return;
    }

    std::cout << "Visualizing plane and cloud..." << std::endl;
    std::cout << "  Cloud points: " << cloud->size() << std::endl;
    std::cout << "  Plane coefficients: "
              << plane_coefficients->values[0] << ", "
              << plane_coefficients->values[1] << ", "
              << plane_coefficients->values[2] << ", "
              << plane_coefficients->values[3] << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Add point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 255, 255); // White
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, "input_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

    // Add plane
    if (!plane_coefficients->values.empty()) {
        viewer->addPlane(*plane_coefficients, "ground_plane");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "ground_plane"); // Red
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "ground_plane"); // Semi-transparent
    } else {
        std::cerr << "VisualizePlane: Plane coefficients are empty, cannot add plane to viewer." << std::endl;
    }


    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Set camera position (example)
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0); // Adjust as needed

    std::cout << "Starting visualization loop. Close the viewer window to continue." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100); // 100 ms
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // If using C++11 thread
    }
    std::cout << "Visualization stopped." << std::endl;
}

bool PointCloudProcessor::rotateCloudToHorizontal(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
    const pcl::ModelCoefficients::ConstPtr& plane_coefficients,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    Eigen::Matrix4f& final_transformation) { // Renamed to avoid conflict

    if (!input_cloud || input_cloud->points.empty()) {
        std::cerr << "RotateCloud: Input cloud is null or empty." << std::endl;
        return false;
    }
    if (!plane_coefficients || plane_coefficients->values.size() != 4) {
        std::cerr << "RotateCloud: Plane coefficients are null or invalid." << std::endl;
        return false;
    }
    if (!output_cloud) {
        std::cerr << "RotateCloud: Output cloud pointer is null." << std::endl;
        return false;
    }
    output_cloud->clear();
    std::cout << "Rotating cloud to horizontal..." << std::endl;

    // 1. Calculate the centroid of the input_cloud
    Eigen::Vector4f centroid_vec;
    pcl::compute3DCentroid(*input_cloud, centroid_vec);
    pcl::PointXYZ centroid_pt(centroid_vec[0], centroid_vec[1], centroid_vec[2]);
    std::cout << "  Centroid: " << centroid_pt.x << ", " << centroid_pt.y << ", " << centroid_pt.z << std::endl;

    // 2. Extract the plane normal vector (a, b, c)
    Eigen::Vector3f current_normal(
        plane_coefficients->values[0],
        plane_coefficients->values[1],
        plane_coefficients->values[2]
    );
    current_normal.normalize(); // Ensure it's a unit vector
    std::cout << "  Current normal: " << current_normal.x() << ", " << current_normal.y() << ", " << current_normal.z() << std::endl;

    // 3. Define the target normal vector
    Eigen::Vector3f target_normal(0.0f, 0.0f, 1.0f);
    if (current_normal.z() < 0.0f) { // If original normal points "downwards"
        target_normal.z() = -1.0f; // Target "downwards" to minimize rotation
        std::cout << "  Current normal's Z is negative, target normal set to <0,0,-1>" << std::endl;
    }
     // Ensure D coefficient is positive if normal is <0,0,1> or negative if normal is <0,0,-1>
    // This is a common convention for plane equations (e.g. Ax+By+Cz+D=0, where D is distance from origin if normal is unit)
    // If normal is (0,0,1) and D is positive, plane is below origin. If D is negative, plane is above.
    // To align with XY plane *at z=0 after rotation*, the D term of rotated plane should be 0.
    // The current D value (plane_coefficients->values[3]) relates to the *original* normal.

    // Check for collinearity / no rotation needed
    if (current_normal.isApprox(target_normal, 1e-4f)) {
        std::cout << "  Cloud is already horizontal or very close. Copying input to output." << std::endl;
        pcl::copyPointCloud(*input_cloud, *output_cloud);
        final_transformation = Eigen::Matrix4f::Identity();
        return true;
    }
    if (current_normal.isApprox(-target_normal, 1e-4f)) {
         std::cout << "  Cloud is already horizontal but upside down. Rotating 180 degrees around X-axis." << std::endl;
         // Rotate 180 degrees around X-axis to flip it up if it's upside down and aligned with Z
        Eigen::AngleAxisf rotation(M_PI, Eigen::Vector3f::UnitX());
        Eigen::Matrix3f rotation_matrix_3d = rotation.toRotationMatrix();
        final_transformation = Eigen::Matrix4f::Identity();
        final_transformation.block<3,3>(0,0) = rotation_matrix_3d;
    } else {
        // 4. Calculate the rotation axis
        Eigen::Vector3f rotation_axis = current_normal.cross(target_normal);
        rotation_axis.normalize();
        std::cout << "  Rotation axis: " << rotation_axis.x() << ", " << rotation_axis.y() << ", " << rotation_axis.z() << std::endl;

        // 5. Calculate the rotation angle
        float rotation_angle = acos(current_normal.dot(target_normal));
        std::cout << "  Rotation angle (rad): " << rotation_angle << " (" << rotation_angle * 180.0 / M_PI << " deg)" << std::endl;

        if (std::isnan(rotation_angle) || std::abs(rotation_angle) < 1e-6) {
             std::cout << "  No rotation needed (angle is zero or NaN). Copying input to output." << std::endl;
             pcl::copyPointCloud(*input_cloud, *output_cloud);
             final_transformation = Eigen::Matrix4f::Identity();
             return true;
        }


        // 6. Create an Eigen::AngleAxisf object
        Eigen::AngleAxisf rotation(rotation_angle, rotation_axis);

        // 7. Convert to a 4x4 transformation matrix (rotation around origin)
        Eigen::Matrix4f rotation_around_origin = Eigen::Matrix4f::Identity();
        rotation_around_origin.block<3,3>(0,0) = rotation.toRotationMatrix();
        final_transformation = rotation_around_origin; // Store this part first
    }


    // 8. Rotate around the cloud centroid
    Eigen::Matrix4f t_centroid_to_origin = Eigen::Matrix4f::Identity();
    t_centroid_to_origin(0,3) = -centroid_pt.x;
    t_centroid_to_origin(1,3) = -centroid_pt.y;
    t_centroid_to_origin(2,3) = -centroid_pt.z;

    Eigen::Matrix4f t_origin_to_centroid = Eigen::Matrix4f::Identity();
    t_origin_to_centroid(0,3) = centroid_pt.x;
    t_origin_to_centroid(1,3) = centroid_pt.y;
    t_origin_to_centroid(2,3) = centroid_pt.z;

    // The final transformation is T_origin_to_centroid * RotationMatrixAroundOrigin * T_centroid_to_origin
    // However, PCL's transformPointCloud applies the transformation *as is* to each point.
    // To rotate around centroid, we need to translate to origin, rotate, then translate back.
    // So the matrix applied to points should be T_origin_to_centroid * RotationMatrix * T_centroid_to_origin
    final_transformation = t_origin_to_centroid * final_transformation * t_centroid_to_origin;


    // 9. Apply this transformation to input_cloud
    pcl::transformPointCloud(*input_cloud, *output_cloud, final_transformation);
    std::cout << "  Cloud rotated. Output cloud points: " << output_cloud->size() << std::endl;

    // 10. Store the combined rotation transformation (already done in final_transformation)
    return true;
}


bool PointCloudProcessor::translateCloudToZZero(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud, // Should be the rotated cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    Eigen::Matrix4f& translation_transform) {

    if (!input_cloud || input_cloud->points.empty()) {
        std::cerr << "TranslateCloud: Input cloud is null or empty." << std::endl;
        return false;
    }
    if (!output_cloud) {
        std::cerr << "TranslateCloud: Output cloud pointer is null." << std::endl;
        return false;
    }
    output_cloud->clear();
    std::cout << "Translating cloud to Z=0..." << std::endl;

    // 1. Calculate the centroid of the input_cloud (the already rotated cloud).
    // The z-value of this centroid is the amount we need to translate by.
    Eigen::Vector4f centroid_vec;
    pcl::compute3DCentroid(*input_cloud, centroid_vec);
    // pcl::PointXYZ rotated_centroid_pt(centroid_vec[0], centroid_vec[1], centroid_vec[2]);
    float translate_z = -centroid_vec[2]; // Translate by negative of current average Z

    std::cout << "  Rotated cloud centroid Z: " << centroid_vec[2] << ", Translation Z: " << translate_z << std::endl;

    // 2. Create an Eigen::Translation3f for this translation along the Z-axis.
    Eigen::Translation3f translation(0.0f, 0.0f, translate_z);

    // 3. Convert this to a 4x4 transformation matrix
    translation_transform = Eigen::Matrix4f::Identity();
    translation_transform.block<3,1>(0,3) = translation.vector();

    // 4. Apply this transformation
    pcl::transformPointCloud(*input_cloud, *output_cloud, translation_transform);
    std::cout << "  Cloud translated. Output cloud points: " << output_cloud->size() << std::endl;

    // Verify new centroid Z (optional debug)
    Eigen::Vector4f new_centroid_vec;
    pcl::compute3DCentroid(*output_cloud, new_centroid_vec);
    std::cout << "  New centroid Z after translation: " << new_centroid_vec[2] << std::endl;

    return true;
}

void PointCloudProcessor::visualizeCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const std::string& window_title) {

    if (!cloud) {
        std::cerr << "VisualizeCloud: Input cloud is null." << std::endl;
        return;
    }

    std::cout << "Visualizing cloud in window: " << window_title << "..." << std::endl;
    std::cout << "  Cloud points: " << cloud->size() << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_title));
    viewer->setBackgroundColor(0.1, 0.1, 0.1); // Darker background

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 0, 255, 0); // Green
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color_handler, "cloud_to_viz");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_to_viz");

    // 2. Create and add the XY plane at Z=0
    pcl::ModelCoefficients::Ptr xy_plane(new pcl::ModelCoefficients());
    xy_plane->values.resize(4);
    xy_plane->values[0] = 0; // X component of normal
    xy_plane->values[1] = 0; // Y component of normal
    xy_plane->values[2] = 1; // Z component of normal (points upwards)
    xy_plane->values[3] = 0; // d (distance from origin, so Z=0)
    viewer->addPlane(*xy_plane, "xy_plane_z0");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "xy_plane_z0"); // Blue
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "xy_plane_z0"); // Semi-transparent


    // 3. Add a coordinate system marker
    viewer->addCoordinateSystem(1.0); // Scale of 1.0 for the axis
    viewer->initCameraParameters();
    // Example camera position - adjust as needed for better view, ensuring origin and plane are visible
    viewer->setCameraPosition(0, -3, 3, 0, 0, 0); // Looking towards origin from -Y, Z slightly up


    std::cout << "Starting visualization loop for window '" << window_title << "'. Close the viewer window to continue." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    std::cout << "Visualization stopped for window '" << window_title << "'." << std::endl;
}

} // namespace proc
