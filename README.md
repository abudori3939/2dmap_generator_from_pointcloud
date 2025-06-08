# 2D OccupancyGridMap generator from PointCloud

このプロジェクトは、3D点群データから2Dの占有格子地図を生成します。

## インストール

1.  リポジトリをクローンします。

2.  インストールスクリプトを実行して、依存関係（PCL、Eigen、YAML-CPP）をインストールします。
    ```bash
    ./install.bash
    ```
    *注意: `install.bash` は `sudo apt-get` を使用します。*


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

ダミーの入力ファイルとして、`data/sample.pcd` や `data/sample.ply`、`config/config.yaml` を作成・配置してテストすると良いでしょう。(これらのサンプルファイルは現時点ではリポジトリに存在しません。`sample.pcd` は現在の環境ではPCLのパーサーとの相性問題がある点に注意してください。)

