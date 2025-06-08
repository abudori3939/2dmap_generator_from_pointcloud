#include "map_processor.h"
#include <iostream> // For potential debugging, can be removed later

// Constructor
MapProcessor::MapProcessor() {
    // Initialization if needed
}

// Converts our OccupancyGrid structure to cv::Mat
cv::Mat MapProcessor::toCvMat(const OccupancyGrid& grid_msg) {
    cv::Mat opencv_map(grid_msg.info.height, grid_msg.info.width, CV_8UC1);
    for (int r = 0; r < grid_msg.info.height; ++r) {
        for (int c = 0; c < grid_msg.info.width; ++c) {
            int8_t value = grid_msg.data[r * grid_msg.info.width + c];
            if (value == 0) { // Free
                opencv_map.at<uchar>(r, c) = 255; // White
            } else if (value == 100) { // Obstacle
                opencv_map.at<uchar>(r, c) = 0;   // Black
            } else { // Unknown (-1) or other values
                opencv_map.at<uchar>(r, c) = 128; // Gray
            }
        }
    }
    return opencv_map;
}

// Fills unknown areas that are likely free space
cv::Mat MapProcessor::fillFreeSpace(const cv::Mat& original_map, int kernel_size) {
    cv::Mat image_free_only;
    // Free is 255 (White) in the map
    cv::inRange(original_map, cv::Scalar(255), cv::Scalar(255), image_free_only);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::Mat dilated_free;
    cv::dilate(image_free_only, dilated_free, kernel);

    cv::Mat final_map = original_map.clone();
    for (int r = 0; r < final_map.rows; ++r) {
        for (int c = 0; c < final_map.cols; ++c) {
            // If original was Unknown (128) and dilated_free is White (255)
            if (final_map.at<uchar>(r, c) == 128 && dilated_free.at<uchar>(r, c) == 255) {
                final_map.at<uchar>(r, c) = 255; // Set to Free (White)
            }
        }
    }
    return final_map;
}

// Fills gaps in obstacles or unknown areas likely to be obstacles
cv::Mat MapProcessor::fillObstacleSpace(const cv::Mat& current_map, int kernel_size) {
    cv::Mat image_obstacle_only;
    // Obstacles are 0 (Black) in the map. cv::inRange will make these 255 in the mask.
    cv::inRange(current_map, cv::Scalar(0), cv::Scalar(0), image_obstacle_only);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::Mat closed_obstacle;
    // MORPH_CLOSE is a dilation followed by an erosion.
    // It's useful for closing small holes inside foreground objects, or small black points on the object.
    cv::morphologyEx(image_obstacle_only, closed_obstacle, cv::MORPH_CLOSE, kernel);

    cv::Mat final_map = current_map.clone();
    for (int r = 0; r < final_map.rows; ++r) {
        for (int c = 0; c < final_map.cols; ++c) {
            // If morphologyEx made it an "obstacle" (255 in closed_obstacle mask),
            // set it to Obstacle (0, Black) in the final_map.
            // This also covers areas that were originally unknown but became part of a closed obstacle region.
            if (closed_obstacle.at<uchar>(r, c) == 255) {
                 // Only change if it's not already free.
                // We don't want obstacle filling to overwrite known free space from fillFreeSpace.
                // However, the problem description implies fillObstacleSpace runs *after* fillFreeSpace.
                // The current_map for fillObstacleSpace would be the output of fillFreeSpace.
                // A cell that became free (255) should not become an obstacle (0).
                // A cell that was unknown (128) and became an obstacle (255 in closed_obstacle) should become 0.
                // A cell that was already an obstacle (0) and is 255 in closed_obstacle should remain 0.
                if (final_map.at<uchar>(r,c) != 255) { // If not already marked as definitely Free
                    final_map.at<uchar>(r, c) = 0; // Set to Obstacle (Black)
                }
            }
        }
    }
    return final_map;
}

// Converts processed cv::Mat back to OccupancyGrid data vector
// The original_grid_msg can be used to retain metadata like info.
std::vector<int8_t> MapProcessor::toOccupancyGridData(const cv::Mat& processed_map, const OccupancyGrid& original_grid_msg) {
    std::vector<int8_t> new_data;
    new_data.reserve(processed_map.rows * processed_map.cols);
    for (int r = 0; r < processed_map.rows; ++r) {
        for (int c = 0; c < processed_map.cols; ++c) {
            uchar value = processed_map.at<uchar>(r, c);
            if (value == 255) { // White
                new_data.push_back(0);   // Free
            } else if (value == 0) { // Black
                new_data.push_back(100); // Obstacle
            } else { // Gray (128)
                new_data.push_back(-1);  // Unknown
            }
        }
    }
    return new_data;
}

// Overloaded version
std::vector<int8_t> MapProcessor::toOccupancyGridData(const cv::Mat& processed_map) {
    std::vector<int8_t> new_data;
    new_data.reserve(processed_map.rows * processed_map.cols);
    for (int r = 0; r < processed_map.rows; ++r) {
        for (int c = 0; c < processed_map.cols; ++c) {
            uchar value = processed_map.at<uchar>(r, c);
            if (value == 255) { // White
                new_data.push_back(0);   // Free
            } else if (value == 0) { // Black
                new_data.push_back(100); // Obstacle
            } else { // Gray (128)
                new_data.push_back(-1);  // Unknown
            }
        }
    }
    return new_data;
}
