#ifndef MAP_IO_H
#define MAP_IO_H

#include <string>
#include <vector>
#include <cstdint> // For int8_t, uint8_t
#include "map_parameters.h" // Uses map_params_util::MapParameters

namespace map_io_util {

// 地図セル値を定義 (PGM出力用)
// ROS PGM standard values:
const extern uint8_t UNKNOWN_VALUE_PGM; // = 205; (PGM表現、ROS標準の灰色)
const extern uint8_t FREE_VALUE_PGM;    // = 254; (PGM表現、ROS標準の空き)
const extern uint8_t OCCUPIED_VALUE_PGM; // = 0;   (PGM表現、占有)

// 内部グリッド値 (map_io_util の外でも使うならより共通の場所に置くべき)
const extern int8_t GRID_VALUE_UNKNOWN;  // = -1;
const extern int8_t GRID_VALUE_OCCUPIED; // = 100; (nav_msgs/OccupancyGrid style)
const extern int8_t GRID_VALUE_FREE;     // = 0;   (nav_msgs/OccupancyGrid style)
                                       // Note: PGM values are 0-255, grid values are -1 to 100.
                                       // The saveMapAsPGM needs to handle this mapping.

/**
 * @brief 占有格子地図をPGM (Portable Graymap) ファイルとして保存します。
 *
 * @param grid 保存する占有格子データ (int8_tのベクター)。
 * @param params 地図のパラメータ (解像度、原点、寸法)。
 * @param filepath 保存先のPGMファイルパス。
 * @return 保存に成功した場合は true、失敗した場合は false。
 */
bool saveMapAsPGM(
    const std::vector<int8_t>& grid,
    const map_params_util::MapParameters& params,
    const std::string& filepath
);

/**
 * @brief 地図のメタデータをROS互換のYAMLファイルとして保存します。
 *
 * @param params 地図のパラメータ。
 * @param pgm_filename 地図画像ファイル名 (例: "map.pgm")。YAMLファイルに記録されます。
 * @param yaml_filepath 保存先のYAMLファイルパス。
 * @return 保存に成功した場合は true、失敗した場合は false。
 */
bool saveMapMetadataYAML(
    const map_params_util::MapParameters& params,
    const std::string& pgm_filename,
    const std::string& yaml_filepath
);

} // namespace map_io_util

#endif // MAP_IO_H
