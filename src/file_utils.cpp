#include "file_utils.h"
#include <iostream>
#include <sys/stat.h> // For mkdir, stat (Linux/Mac)
#include <errno.h>    // For errno
#include <cstring>   // For strerror

namespace utils {

void ensureDirectoryExists(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0) { // ディレクトリが存在しない
        // 作成を試みる。必要に応じてより堅牢なエラー処理を追加する。
        // Windowsでのmkdirは異なるヘッダ/呼び出しが必要。これはPOSIX用。
        if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
            std::cerr << "警告: 出力ディレクトリ " << path << " の作成に失敗しました。errno: " << strerror(errno) << std::endl;
            // 要件によっては、ここで例外をスローするか終了する。
        } else {
            // errnoがEEXISTでなくても、mkdirが0を返せば成功とみなす
            // (例えば、別のプロセスがほぼ同時に作成した場合など、statの後でmkdirの前に作られた場合)
            if (errno == EEXIST) {
                 std::cout << "情報: ディレクトリ " << path << " は既に存在していました (mkdirがEEXISTを返しました)。" << std::endl;
                 errno = 0; // EEXIST状態をクリア
            } else {
                 std::cout << "出力ディレクトリ " << path << " を作成しました。" << std::endl;
            }
        }
    } else if (!S_ISDIR(st.st_mode)) {
        std::cerr << "警告: " << path << " は既に存在しますが、ディレクトリではありません。" << std::endl;
        // エラー処理: パスは存在するがディレクトリではない
    } else {
        // ディレクトリは既に存在し、問題ない
        std::cout << "情報: 出力ディレクトリ " << path << " は既に存在しています。" << std::endl;
    }
}

} // namespace utils
