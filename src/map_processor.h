#ifndef MAP_PROCESSOR_H
#define MAP_PROCESSOR_H

#include <vector>
#include <cstdint> // For int8_t
#include <opencv2/opencv.hpp> // Forward declare or include fully? Include for now.

// Basic OccupancyGrid structure (mimicking nav_msgs/msg/OccupancyGrid)
// This might be replaced or adapted if actual ROS nav_msgs::msg::OccupancyGrid is used directly.
struct OccupancyGrid {
    struct Info {
        int width;
        int height;
        float resolution; // Example: meters/pixel
        // Add other fields like origin if necessary
    } info;
    std::vector<int8_t> data;

    // Default constructor
    OccupancyGrid() : info{0, 0, 0.0f}, data{} {}
};

class MapProcessor {
public:
    MapProcessor(); // Constructor, if needed for setup

    // Converts our OccupancyGrid structure to cv::Mat
    cv::Mat toCvMat(const OccupancyGrid& grid_msg);

    // Fills unknown areas that are likely free space
    cv::Mat fillFreeSpace(const cv::Mat& original_map, int kernel_size);

    // Fills gaps in obstacles or unknown areas likely to be obstacles
    cv::Mat fillObstacleSpace(const cv::Mat& current_map, int kernel_size);

    // Converts processed cv::Mat back to OccupancyGrid data vector
    // The original_grid_msg can be used to retain metadata like info.
    std::vector<int8_t> toOccupancyGridData(const cv::Mat& processed_map, const OccupancyGrid& original_grid_msg);

    // Overloaded version if we only need the data vector without needing original_grid_msg
    std::vector<int8_t> toOccupancyGridData(const cv::Mat& processed_map);
};

#endif // MAP_PROCESSOR_H
