# 概要

Amberのmoveit用の設定および確認のためのlaunchファイルが格納されているROSパッケージです。
大部分はMoveIt Setup Assistantにより自動生成されたファイルになります。

# パッケージの構成

- launch launchファイルが格納されたフォルダ
- config moveit用のconfig設定が格納されたフォルダ

# 代表的なlaunch file

自動生成されたファイルが多いため実際のフロントエンドとして使用するlaunchファイルのみ以下に記載します。

## demo_gazebo.launch

gazeboからmoveitを利用するためのlaunchファイル

## demo_hw.launch

実機からmoveitを利用するためのlaunchファイル

# 動作確認方法

## 実機とMoveIt!との立ち上げ

本プログラムを作動するコンピュータとロボット制御ボードがネットワーク接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch amber_moveit_config demo_hw.launch
```

## GazeboシミュレーターとMoveIt!との立ち上げ

本プログラムを作動するコンピュータとロボット制御ボードが接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch amber_moveit_config demo_gazebo.launch
```

*Gazeboが立ち上がったら、再生ボタンを押してください。*
