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

        // Load new integer parameters for clustering
        load_int_param("min_cluster_size", config.min_cluster_size, config.min_cluster_size);
        load_int_param("max_cluster_size", config.max_cluster_size, config.max_cluster_size);

        // Load map processing kernel sizes
        load_int_param("free_space_kernel_size", config.free_space_kernel_size, config.free_space_kernel_size);
        load_int_param("obstacle_space_kernel_size", config.obstacle_space_kernel_size, config.obstacle_space_kernel_size);

       // Load the new boolean parameter for map preview
       if (yaml_file_root["preview_map_on_exit"]) {
           try {
               config.preview_map_on_exit = yaml_file_root["preview_map_on_exit"].as<bool>();
           } catch (const YAML::TypedBadConversion<bool>& e) {
               std::cerr << "警告: 設定ファイル内のキー 'preview_map_on_exit' の型変換に失敗しました。"
                         << "期待する型: bool, 実際の値: '" << yaml_file_root["preview_map_on_exit"].Scalar() << "' (" << e.what() << "). "
                         << "デフォルト値 (" << false << ") を使用します。" << std::endl;
               config.preview_map_on_exit = false; // Default to false on error
           } catch (const YAML::Exception& e) {
               std::cerr << "警告: 設定項目 'preview_map_on_exit' の読み込み中にYAMLエラーが発生しました。デフォルト値 ("
                         << false << ") を使用します。エラー: " << e.what() << std::endl;
               config.preview_map_on_exit = false; // Default to false on error
           }
       } else {
           std::cout << "情報: 設定ファイルに 'preview_map_on_exit' が見つかりません。デフォルト値 ("
                     << false << ") を使用します。" << std::endl;
           config.preview_map_on_exit = false; // Default to false if not found
       }
       // Also print the loaded value
       std::cout << "  プレビュー表示 (終了時): " << (config.preview_map_on_exit ? "有効" : "無効") << std::endl;
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
