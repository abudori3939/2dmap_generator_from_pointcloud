#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <string>

namespace utils {
/**
 * @brief 指定されたパスにディレクトリが存在することを確認し、存在しない場合は作成を試みます。
 *
 * @param path 作成または確認するディレクトリのパス。
 * @note この関数は現在POSIXシステム（Linux, macOS）に依存しています。
 *       Windows環境では異なる実装が必要です。
 */
void ensureDirectoryExists(const std::string& path);

} // namespace utils

#endif // FILE_UTILS_H
