# 3D Point Cloud to 2D Occupancy Grid Map Generator

このプロジェクトは、3D点群データから2Dの占有格子地図を生成します。

## インストール

1.  リポジトリをクローンします。
2.  インストールスクリプトを実行して、依存関係（PCL、Eigen、YAML-CPP、OpenCV）をインストールします。
    ```bash
    ./install.bash
    ```
    *注意: `install.bash` は `sudo apt-get` を使用します。OpenCVが手動でインストールされている場合は、スクリプトの該当部分をスキップまたはコメントアウトしてください。*

## ビルド方法

1.  ビルドディレクトリを作成します。
    ```bash
    mkdir build
    cd build
    ```
2.  CMakeを実行し、プロジェクトをビルドします。
    ```bash
    cmake ..
    make
    ```
実行可能ファイル `occupancy_grid_map_generator` は `build` ディレクトリ内に作成されます。

## 実行方法

ビルド後、`build` ディレクトリから実行可能ファイルを実行できます。プログラムは2つの引数を取ります。

1.  入力PCDまたはPLYファイルへのパス
2.  設定YAMLファイルへのパス

例:
```bash
# PCDファイルの場合
./build/occupancy_grid_map_generator data/input.pcd config/config.yaml

# PLYファイルの場合
./build/occupancy_grid_map_generator data/input.ply config/config.yaml
```

処理が成功すると、`output` ディレクトリに以下のファイルが生成されます。
- `map.pgm`: 生成された占有格子地図画像
- `map_metadata.yaml`: 地図のメタデータ (解像度、原点など)

### 地図の後処理
生成された2D占有格子地図には、以下の後処理ステップが適用されます。
1.  **フリースペース充填**: 地図上の未知の領域（センシングされなかったが、自由空間である可能性が高い領域）を、検出された自由空間に基づいて拡張して充填します。この処理のカーネルサイズは `config/config.yaml` の `free_space_kernel_size` で設定可能です。
2.  **障害物スペース充填**: 障害物間の小さな隙間や、障害物として検出されるべきだった未知の領域を充填します。この処理のカーネルサイズは `config/config.yaml` の `obstacle_space_kernel_size` で設定可能です。

これらの処理は、OpenCVライブラリを使用して行われます。

ダミーの入力ファイルとして、`data/sample.pcd` や `data/sample.ply`、`config/config.yaml` を作成・配置してテストすると良いでしょう。(これらのサンプルファイルは現時点ではリポジトリに存在しません。`sample.pcd` は現在の環境ではPCLのパーサーとの相性問題がある点に注意してください。)
