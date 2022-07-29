# 概要

Amberのurdfファイルおよび確認のためのlaunchファイルが格納されているROSパッケージです。

# パッケージの構成

- launch launchファイルが格納されたフォルダ
- urdf   urdfファイルが格納されたフォルダ
- meshes 各リンクのmeshファイル（stl形式）が格納されたフォルダ

# Launch Files

## amber_upload.launch

robot_descriptionにロボットモデルを読み込むためのlaunchファイル

## amber_display.launch

robot_descriptionにロボットモデルを読み込み、rviz上で関節動作を確認するためのlaunchファイル

# モデルの確認方法

``` bash
$ roslaunch amber_description amber_display.launch
```
