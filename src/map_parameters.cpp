#include "map_parameters.h"
#include <algorithm> // For std::min and std::max
#include <cmath>     // For std::ceil
#include <limits>    // Required for std::numeric_limits
#include <iostream>  // For std::cerr in case of empty cloud or invalid resolution

namespace map_params_util {

bool calculateMapParameters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    double resolution,
    MapParameters& params) {

    if (!cloud) { // Check for null pointer first
        std::cerr << "エラー: 点群ポインタが無効です。地図パラメータを計算できません。" << std::endl;
        return false;
    }
    if (cloud->points.empty()) {
        std::cerr << "エラー: 点群データが空です。地図パラメータを計算できません。" << std::endl;
        return false;
    }

    if (resolution <= 0.0) {
        std::cerr << "エラー: 解像度は正の値でなければなりません。指定された値: " << resolution << std::endl;
        return false;
    }

    params.resolution = resolution;

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_x = std::max(max_x, static_cast<double>(point.x));
        max_y = std::max(max_y, static_cast<double>(point.y));
    }

    params.origin_x = min_x;
    params.origin_y = min_y;

    // 地図の幅と高さをピクセル単位で計算
    // ceil を使用して、点群全体をカバーできるようにする
    params.width_pixels = static_cast<int>(std::ceil((max_x - min_x) / params.resolution));
    params.height_pixels = static_cast<int>(std::ceil((max_y - min_y) / params.resolution));

    // 幅または高さが0ピクセルの場合のエッジケース処理 (例: 点群が直線状または一点)
    if (params.width_pixels <= 0) params.width_pixels = 1; // Ensure width is at least 1
    if (params.height_pixels <= 0) params.height_pixels = 1; // Ensure height is at least 1

    return true;
}

} // namespace map_params_util
