#include "map_io.h"
#include <fstream>   // For std::ofstream
#include <iostream>  // For std::cerr, std::cout
#include <vector>    // For std::vector (though already in map_io.h via map_parameters.h indirectly)

namespace map_io_util {

// Define constants
const uint8_t UNKNOWN_VALUE_PGM = 205;
const uint8_t FREE_VALUE_PGM = 254;
const uint8_t OCCUPIED_VALUE_PGM = 0;

const int8_t GRID_VALUE_UNKNOWN = -1;
const int8_t GRID_VALUE_OCCUPIED = 100;
const int8_t GRID_VALUE_FREE = 0;

bool saveMapAsPGM(
    const std::vector<int8_t>& grid,
    const map_params_util::MapParameters& params,
    const std::string& filepath) {

    std::cout << "PGMファイルとして地図を保存中: " << filepath << std::endl;
    std::ofstream pgm_file(filepath, std::ios::binary);
    if (!pgm_file.is_open()) {
        std::cerr << "エラー: PGMファイルを開けませんでした: " << filepath << std::endl;
        return false;
    }

    pgm_file << "P5\n"; // PGMヘッダー (P5 format: binary grayscale)
    pgm_file << params.width_pixels << " " << params.height_pixels << "\n";
    pgm_file << "255\n"; // Max gray value

    std::vector<uint8_t> pgm_data;
    pgm_data.reserve(grid.size()); // Pre-allocate memory

    for (int8_t cell_value : grid) {
        if (cell_value == GRID_VALUE_UNKNOWN) { // Internal unknown
            pgm_data.push_back(UNKNOWN_VALUE_PGM);
        } else if (cell_value == GRID_VALUE_OCCUPIED) { // Internal occupied
            pgm_data.push_back(OCCUPIED_VALUE_PGM);
        } else if (cell_value == GRID_VALUE_FREE) { // Internal free
            pgm_data.push_back(FREE_VALUE_PGM);
        } else {
            // Handle other potential values if necessary, or default to unknown
            std::cerr << "警告: 予期しないグリッド値 " << static_cast<int>(cell_value)
                      << " を検出しました。PGMでは「未知」(" << static_cast<int>(UNKNOWN_VALUE_PGM)
                      << ") として扱います。" << std::endl;
            pgm_data.push_back(UNKNOWN_VALUE_PGM);
        }
    }

    pgm_file.write(reinterpret_cast<const char*>(pgm_data.data()), pgm_data.size());

    if (pgm_file.fail()) {
        std::cerr << "エラー: PGMファイルへの書き込み中にエラーが発生しました。" << std::endl;
        pgm_file.close();
        return false;
    }

    pgm_file.close();
    std::cout << "PGMファイルを正常に保存しました。" << std::endl;
    return true;
}

bool saveMapMetadataYAML(
    const map_params_util::MapParameters& params,
    const std::string& pgm_filename,
    const std::string& yaml_filepath) {

    std::cout << "地図メタデータYAMLファイルを保存中: " << yaml_filepath << std::endl;
    std::ofstream yaml_file(yaml_filepath);
    if (!yaml_file.is_open()) {
        std::cerr << "エラー: 地図メタデータYAMLファイルを開けませんでした: " << yaml_filepath << std::endl;
        return false;
    }

    yaml_file << "image: " << pgm_filename << std::endl;
    yaml_file << "resolution: " << params.resolution << std::endl;
    yaml_file << "origin: [" << params.origin_x << ", " << params.origin_y << ", 0.0]" << std::endl;
    yaml_file << "negate: 0" << std::endl;
    yaml_file << "occupied_thresh: 0.65" << std::endl;
    yaml_file << "free_thresh: 0.196" << std::endl;
    yaml_file << "mode: trinary" << std::endl;

    if (yaml_file.fail()) {
        std::cerr << "エラー: 地図メタデータYAMLファイルへの書き込み中にエラーが発生しました。" << std::endl;
        yaml_file.close();
        return false;
    }

    yaml_file.close();
    std::cout << "地図メタデータYAMLファイルを正常に保存しました。" << std::endl;
    return true;
}

} // namespace map_io_util
