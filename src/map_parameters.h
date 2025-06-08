#ifndef MAP_PARAMETERS_H
#define MAP_PARAMETERS_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map_params_util {

/**
 * @brief 地図の幾何学的パラメータと解像度を保持する構造体。
 */
struct MapParameters {
    double origin_x = 0.0;      // 地図の原点のX座標 [m]
    double origin_y = 0.0;      // 地図の原点のY座標 [m]
    int width_pixels = 0;       // 地図の幅 [ピクセル]
    int height_pixels = 0;      // 地図の高さ [ピクセル]
    double resolution = 0.05;   // 地図の解像度 [m/pixel]
};

/**
 * @brief 与えられた点群データと解像度から、地図の原点と寸法を計算します。
 *
 * @param cloud 入力となる点群 (pcl::PointXYZ型)。
 * @param resolution 地図の解像度 [m/pixel]。
 * @param params 計算された地図パラメータを格納するMapParameters構造体への参照。
 * @return 点群が空でなく、計算に成功した場合は true、失敗した場合は false。
 */
bool calculateMapParameters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    double resolution,
    MapParameters& params
);

} // namespace map_params_util

#endif // MAP_PARAMETERS_H
