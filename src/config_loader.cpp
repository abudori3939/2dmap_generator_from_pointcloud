#include "config_loader.h"
#include <iostream> // For std::cerr, std::endl

namespace map_config {

bool loadConfig(const std::string& filepath, Config& config) {
    try {
        YAML::Node yaml_file_root = YAML::LoadFile(filepath);

        // Helper lambda to load a double value or use default and print warning
        auto load_double_param = [&](const std::string& key, double& param_to_set, const double& default_val) {
            if (yaml_file_root[key]) {
                try {
                    param_to_set = yaml_file_root[key].as<double>();
                } catch (const YAML::TypedBadConversion<double>& e) {
                    std::cerr << "警告: 設定ファイル内のキー '" << key << "' の型変換に失敗しました。"
                              << "期待する型: double, 実際の値: '" << yaml_file_root[key].Scalar() << "' (" << e.what() << "). "
                              << "デフォルト値 (" << default_val << ") を使用します。" << std::endl;
                    param_to_set = default_val;
                } catch (const YAML::Exception& e) { // Catch other YAML errors for this key
                    std::cerr << "警告: 設定項目 '" << key << "' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                              << default_val << ") を使用します。エラー: " << e.what() << std::endl;
                    param_to_set = default_val;
                }
            } else {
                std::cout << "情報: 設定ファイルに '" << key << "' が見つかりません。デフォルト値 ("
                          << default_val << ") を使用します。" << std::endl;
                param_to_set = default_val;
            }
        };

        // Helper lambda to load a bool value or use default and print warning/info
        auto load_bool_param = [&](const std::string& key, bool& param_to_set, const bool& default_val) {
            if (yaml_file_root[key]) {
                try {
                    param_to_set = yaml_file_root[key].as<bool>();
                } catch (const YAML::TypedBadConversion<bool>& e) {
                    std::cerr << "警告: 設定ファイル内のキー '" << key << "' の型変換に失敗しました。"
                              << "期待する型: bool, 実際の値: '" << yaml_file_root[key].Scalar() << "' (" << e.what() << "). "
                              << "デフォルト値 (" << default_val << ") を使用します。" << std::endl;
                    param_to_set = default_val;
                } catch (const YAML::Exception& e) { // Catch other YAML errors for this key
                     std::cerr << "警告: 設定項目 '" << key << "' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                               << default_val << ") を使用します。エラー: " << e.what() << std::endl;
                    param_to_set = default_val;
                }
            } else {
                std::cout << "情報: 設定ファイルに '" << key << "' が見つかりません。デフォルト値 ("
                          << default_val << ") を使用します。" << std::endl;
                param_to_set = default_val;
            }
        };

        // Helper lambda to load an int value or use default and print warning/info
        auto load_int_param = [&](const std::string& key, int& param_to_set, const int& default_val) {
            if (yaml_file_root[key]) {
                try {
                    param_to_set = yaml_file_root[key].as<int>();
                } catch (const YAML::TypedBadConversion<int>& e) {
                    std::cerr << "警告: 設定ファイル内のキー '" << key << "' の型変換に失敗しました。"
                              << "期待する型: int, 実際の値: '" << yaml_file_root[key].Scalar() << "' (" << e.what() << "). "
                              << "デフォルト値 (" << default_val << ") を使用します。" << std::endl;
                    param_to_set = default_val;
                } catch (const YAML::Exception& e) { // Catch other YAML errors for this key
                     std::cerr << "警告: 設定項目 '" << key << "' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                              << default_val << ") を使用します。エラー: " << e.what() << std::endl;
                    param_to_set = default_val;
                }
            } else {
                std::cout << "情報: 設定ファイルに '" << key << "' が見つかりません。デフォルト値 ("
                          << default_val << ") を使用します。" << std::endl;
                param_to_set = default_val;
            }
        };

        load_double_param("map_resolution", config.map_resolution, config.map_resolution);
        load_double_param("robot_height", config.robot_height, config.robot_height);
        load_double_param("normal_estimation_radius", config.normal_estimation_radius, config.normal_estimation_radius);
        load_double_param("ground_normal_z_threshold", config.ground_normal_z_threshold, config.ground_normal_z_threshold);
        load_double_param("block_size", config.block_size, config.block_size);
        load_bool_param("force_single_block_processing", config.force_single_block_processing, config.force_single_block_processing);
        load_int_param("min_points_for_block_processing", config.min_points_for_block_processing, config.min_points_for_block_processing);

        // Load new integer parameters for clustering
        load_int_param("min_cluster_size", config.min_cluster_size, config.min_cluster_size);
        load_int_param("max_cluster_size", config.max_cluster_size, config.max_cluster_size);

        // Load map processing kernel sizes
        load_int_param("free_space_kernel_size", config.free_space_kernel_size, config.free_space_kernel_size);
        load_int_param("obstacle_space_kernel_size", config.obstacle_space_kernel_size, config.obstacle_space_kernel_size);

        // Load Outlier Removal Parameters
        load_int_param("outlier_removal_mean_k", config.outlier_removal_mean_k, config.outlier_removal_mean_k);
        load_double_param("outlier_removal_std_dev_mul_thresh", config.outlier_removal_std_dev_mul_thresh, config.outlier_removal_std_dev_mul_thresh);

        if (yaml_file_root["outlier_removal_enable"]) {
            try {
                config.outlier_removal_enable = yaml_file_root["outlier_removal_enable"].as<bool>();
            } catch (const YAML::TypedBadConversion<bool>& e) {
                std::cerr << "警告: 設定ファイル内のキー 'outlier_removal_enable' の型変換に失敗しました。"
                          << "期待する型: bool, 実際の値: '" << yaml_file_root["outlier_removal_enable"].Scalar() << "' (" << e.what() << "). "
                          << "デフォルト値 (" << config.outlier_removal_enable << ") を使用します。" << std::endl;
                // Default value is already set in struct, so no need to re-assign unless specifically handling error differently.
            } catch (const YAML::Exception& e) {
                 std::cerr << "警告: 設定項目 'outlier_removal_enable' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                           << config.outlier_removal_enable << ") を使用します。エラー: " << e.what() << std::endl;
            }
        } else {
            std::cout << "情報: 設定ファイルに 'outlier_removal_enable' が見つかりません。デフォルト値 ("
                      << config.outlier_removal_enable << ") を使用します。" << std::endl;
        }
        std::cout << "  外れ値除去 (StatisticalOutlierRemoval): " << (config.outlier_removal_enable ? "有効" : "無効") << std::endl;
        if (config.outlier_removal_enable) {
            std::cout << "    MeanK: " << config.outlier_removal_mean_k << std::endl;
            std::cout << "    StdDevMulThresh: " << config.outlier_removal_std_dev_mul_thresh << std::endl;
        }

       // Load the new boolean parameter for map preview
       if (yaml_file_root["enable_visualization"]) {
           try {
               config.enable_visualization = yaml_file_root["enable_visualization"].as<bool>();
           } catch (const YAML::TypedBadConversion<bool>& e) {
               std::cerr << "警告: 設定ファイル内のキー 'enable_visualization' の型変換に失敗しました。"
                         << "期待する型: bool, 実際の値: '" << yaml_file_root["enable_visualization"].Scalar() << "' (" << e.what() << "). "
                         << "デフォルト値 (" << false << ") を使用します。" << std::endl;
               config.enable_visualization = false; // Default to false on error
           } catch (const YAML::Exception& e) {
               std::cerr << "警告: 設定項目 'enable_visualization' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                         << false << ") を使用します。エラー: " << e.what() << std::endl;
               config.enable_visualization = false; // Default to false on error
           }
       } else {
           std::cout << "情報: 設定ファイルに 'enable_visualization' が見つかりません。デフォルト値 ("
                     << false << ") を使用します。" << std::endl;
           config.enable_visualization = false; // Default to false if not found
       }
       // Also print the loaded value
       std::cout << "  可視化 (PCLVisualizer): " << (config.enable_visualization ? "有効" : "無効") << std::endl;
        std::cout << "  単一ブロック強制処理: " << (config.force_single_block_processing ? "有効" : "無効") << std::endl;
        std::cout << "  ブロック処理最小点数: " << config.min_points_for_block_processing << " 点" << std::endl;
        std::cout << "  フリースペース充填カーネルサイズ: " << config.free_space_kernel_size << std::endl;
        std::cout << "  障害物スペース充填カーネルサイズ: " << config.obstacle_space_kernel_size << std::endl;


        return true;
    } catch (const YAML::BadFile& e) {
        std::cerr << "エラー: 設定ファイルが見つからないか、読み取り権限がありません: " << filepath
                  << " (" << e.what() << ")" << std::endl;
        return false;
    } catch (const YAML::ParserException& e) {
        std::cerr << "エラー: YAML設定ファイルの解析中に構文エラーが発生しました: " << filepath
                  << " (行: " << e.mark.line + 1 << ", 列: " << e.mark.column + 1 << "): "
                  << e.msg << std::endl;
        return false;
    } catch (const YAML::Exception& e) {
        std::cerr << "エラー: YAML設定ファイルの処理中に予期せぬyaml-cpp例外が発生しました: " << filepath
                  << " (" << e.what() << ")" << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "エラー: 設定ファイルの読み込み中に予期せぬ標準例外が発生しました: " << filepath
                  << " (" << e.what() << ")" << std::endl;
        return false;
    }
}

} // namespace map_config
