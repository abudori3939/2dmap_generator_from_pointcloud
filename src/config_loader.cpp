#include "config_loader.h"
#include <iostream> // For std::cerr, std::endl

namespace map_config {

bool loadConfig(const std::string& filepath, Config& config) {
    try {
        YAML::Node yaml_file_root = YAML::LoadFile(filepath);

        // Helper lambda to load a value or use default and print warning
        auto load_param = [&](const std::string& key, double& param_to_set, const double& default_val) {
            if (yaml_file_root[key]) {
                try {
                    param_to_set = yaml_file_root[key].as<double>();
                } catch (const YAML::TypedBadConversion<double>& e) {
                    std::cerr << "警告: 設定ファイル内のキー '" << key << "' の型変換に失敗しました。"
                              << "期待する型: double, 実際の値: '" << yaml_file_root[key].Scalar() << "' (" << e.what() << "). "
                              << "デフォルト値 (" << default_val << ") を使用します。" << std::endl;
                    param_to_set = default_val; // Ensure default is set on conversion error
                }
            } else {
                std::cerr << "警告: 設定ファイルに '" << key << "' が見つかりません。デフォルト値 ("
                          << default_val << ") を使用します。" << std::endl;
                param_to_set = default_val; // Ensure default is set if key is missing
            }
        };

        load_param("map_resolution", config.map_resolution, config.map_resolution);
        load_param("robot_height", config.robot_height, config.robot_height);
        load_param("normal_estimation_radius", config.normal_estimation_radius, config.normal_estimation_radius);
        load_param("ground_normal_z_threshold", config.ground_normal_z_threshold, config.ground_normal_z_threshold);
        load_param("block_size", config.block_size, config.block_size);

        // TODO: 他のパラメータも同様に読み込む (現在は上記ですべて)

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
    } catch (const YAML::Exception& e) { // Catch-all for other yaml-cpp exceptions
        std::cerr << "エラー: YAML設定ファイルの処理中に予期せぬyaml-cpp例外が発生しました: " << filepath
                  << " (" << e.what() << ")" << std::endl;
        return false;
    } catch (const std::exception& e) { // Catch-all for other std exceptions
        std::cerr << "エラー: 設定ファイルの読み込み中に予期せぬ標準例外が発生しました: " << filepath
                  << " (" << e.what() << ")" << std::endl;
        return false;
    }
}

} // namespace map_config
