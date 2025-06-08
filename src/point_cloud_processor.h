#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h> // For plane coefficients
#include <pcl/PointIndices.h>      // For inliers
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/visualization/pcl_visualizer.h> // For PCLVisualizer
#include <pcl/common/common.h>                // For centroid calculation
#include <pcl/common/transforms.h>            // For point cloud transformation
#include <Eigen/Geometry>                     // For AngleAxisf, Matrix4f etc.

namespace map_config {
    struct Config;
}

namespace proc {

class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    bool estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
        float search_radius,
        pcl::PointCloud<pcl::Normal>::Ptr& output_normals
    );

    bool extractGroundCandidates(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
        const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
        float ground_normal_z_thresh,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_candidates_cloud,
        pcl::PointIndices::Ptr& ground_candidate_indices // Added output for indices
    );

    bool extractMainGroundCluster(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground_candidates_cloud,
        float cluster_tolerance,
        int min_cluster_size,
        int max_cluster_size,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& main_ground_cluster_cloud
    );

    /**
     * @brief 主地面クラスタの点群から単一の平面モデルをフィッティングします。
     * @param main_ground_cluster_cloud 平面フィッティングの対象となる主地面クラスタの点群。
     * @param distance_threshold 平面からの距離の許容閾値 [m]。
     * @param output_plane_coefficients 推定された平面モデルの係数 (Ax+By+Cz+D=0)。このポインタは呼び出し側で初期化されている必要があります。
     * @param output_plane_inliers 平面モデルに属すると判定された点のインデックス。このポインタは呼び出し側で初期化されている必要があります。
     * @return 処理に成功し、平面が見つかった場合は true、それ以外は false。
     */
    bool fitGlobalGroundPlane(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& main_ground_cluster_cloud,
        float distance_threshold,
        pcl::ModelCoefficients::Ptr& output_plane_coefficients,
        pcl::PointIndices::Ptr& output_plane_inliers
    );

    void visualizePlane(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        const pcl::ModelCoefficients::ConstPtr& plane_coefficients
    );

    bool rotateCloudToHorizontal(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud,
        const pcl::ModelCoefficients::ConstPtr& plane_coefficients,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        Eigen::Matrix4f& rotation_transform // Output parameter
    );

    bool translateCloudToZZero(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud, // Should be the rotated cloud
        // const Eigen::Matrix4f& rotation_transform, // May not be needed if cloud is already rotated
        // const pcl::ModelCoefficients::ConstPtr& original_plane_coefficients, // May not be needed
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        Eigen::Matrix4f& translation_transform      // Output parameter
    );

    void visualizeCloud(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        const std::string& window_title
    );

    bool extractNonHorizontalPoints(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& original_cloud,
        const pcl::PointIndices::ConstPtr& ground_candidate_indices,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& non_horizontal_cloud
    );

    void visualizeCombinedClouds(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& nonground_cloud,
        const std::string& window_title
    );

private:
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
};

} // namespace proc

#endif // POINT_CLOUD_PROCESSOR_H
