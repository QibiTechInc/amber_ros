# 概要

Amberのgazebo用の設定および確認のためのlaunchファイルが格納されているROSパッケージです。

# パッケージの構成

- launch launchファイルが格納されたフォルダ
- config gazebo用のコントローラ設定が格納されたフォルダ
- world  シミュレーションで読み込むworldファイルが格納されたフォルダ

# Launch Files

## amber_controller.launch

gazebo用のコントローラを起動するためのlaunchファイル

## amber_model4_gazebo.launch

empty.world上にamberを配置して起動するためのlaunchファイル

# モデルの確認方法

通常モデルの場合は以下のコマンドで確認できる。

``` bash
$ roslaunch amber_gazebo amber_model4_gazebo.launch
```

簡易モデルを使用する場合は以下のコマンドで起動する。

``` bash
$ roslaunch amber_gazebo amber_model4_gazebo.launch model:=amber_model4_sim_prim
```
