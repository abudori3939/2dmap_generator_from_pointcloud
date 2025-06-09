#ifndef PIPELINE_H
#define PIPELINE_H

#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "config_loader.h"
#include "map_parameters.h"

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <Eigen/Core> // For Eigen::Matrix4f
#include "point_cloud_processor.h" // Forward declaration or include for proc::PointCloudProcessor

namespace pipeline {

struct BlockOutput {
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_ground_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_obstacle_points;

    BlockOutput() :
        global_ground_points(new pcl::PointCloud<pcl::PointXYZ>()),
        global_obstacle_points(new pcl::PointCloud<pcl::PointXYZ>())
    {}
};

struct PipelineData {
    // Input and Config
    std::string pointcloud_file_path;
    std::string config_file_path;
    map_config::Config app_config;

    // Raw and Processed Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates;
    pcl::PointIndices::Ptr ground_candidate_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr main_ground_cluster;

    // Plane and Transformation Data
    pcl::ModelCoefficients::Ptr plane_coefficients;
    pcl::PointIndices::Ptr plane_inliers;
    Eigen::Matrix4f rotation_matrix;
    Eigen::Matrix4f translation_matrix; // To Z=0

    // Transformed Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_transformed_cloud; // Main ground cluster after full transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_ground_candidates_transformed_cloud; // All ground candidates after full transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_transformed_cloud; // Non-horizontal points after full transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_downsampled_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_horizontal_filtered_cloud; // Final obstacles

    // Map Data
    map_params_util::MapParameters map_params;
    std::vector<int8_t> occupancy_grid;

    PipelineData() :
        raw_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        normals(new pcl::PointCloud<pcl::Normal>()),
        ground_candidates(new pcl::PointCloud<pcl::PointXYZ>()),
        ground_candidate_indices(new pcl::PointIndices()),
        non_horizontal_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        main_ground_cluster(new pcl::PointCloud<pcl::PointXYZ>()),
        plane_coefficients(new pcl::ModelCoefficients()),
        plane_inliers(new pcl::PointIndices()),
        ground_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        all_ground_candidates_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        non_horizontal_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        non_horizontal_downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
        non_horizontal_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>())
    {}
};

bool parseArguments(int argc, char* argv[], std::string& pointcloud_file_path, std::string& config_file_path);
bool loadConfiguration(const std::string& config_file_path, map_config::Config& app_config);
bool loadPointCloud(const std::string& pointcloud_file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
bool processPointCloud(PipelineData& data, proc::PointCloudProcessor& processor);
bool generateOccupancyGrid(PipelineData& data);
bool saveMap(const PipelineData& data, const std::string& output_dir = "output");

} // namespace pipeline

#endif // PIPELINE_H
