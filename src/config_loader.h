#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <string>
#include "yaml-cpp/yaml.h" // YAML::Node is part of the function signature / implementation details

namespace map_config {

/**
 * @brief アプリケーションの設定値を保持する構造体。
 */
struct Config {
    double map_resolution = 0.05; // 地図の解像度 [m/pixel]、デフォルト値
    double robot_height = 0.5;    // ロボットの高さ [m]、デフォルト値
    double normal_estimation_radius = 0.1; // 法線推定時の近傍探索半径 [m]
    double ground_normal_z_threshold = 0.9; // 地面と判定するための法線Z成分の閾値
    double block_size = 10.0; // 並列処理で分割するブロックのサイズ [m]

    // クラスタリングパラメータ
    int min_cluster_size = 50;    // クラスタを構成する最小点数
    int max_cluster_size = 25000; // クラスタを構成する最大点数

    // 今後、他のパラメータもここに追加
};

/**
 * @brief 指定されたYAMLファイルから設定を読み込み、Config構造体に格納します。
 *
 * @param filepath 設定ファイルのパス。
 * @param config 読み込まれた設定値を格納するConfig構造体への参照。
 * @return 読み込みに成功した場合は true、失敗した場合は false。
 */
bool loadConfig(const std::string& filepath, Config& config);

} // namespace map_config

#endif // CONFIG_LOADER_H
