# 3D点群からの2D占有格子地図（Occupancy Grid Map）生成ツール

このプロジェクトは、3D点群データから2Dの占有格子地図を生成します。

## インストール

1.  リポジトリをクローンします。
2.  インストールスクリプトを実行して、依存関係（PCL、Eigen）をインストールします。
    ```bash
    ./install.bash
    ```

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
