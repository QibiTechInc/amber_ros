# amber_ros #

amberシリーズのロボットをROSで制御するためのパッケージ群です。

### 依存ライブラリ ###

* libhr4c_comm: hr4cプロトコル実装ライブラリ。バイナリファイルの形式で提供されます。

### ビルド ###

catkinワークスペースで以下を入力します。

```bash
catkin_make
source devel/setup.bash
```
### 実機とコントローラ単独での立ち上げ

本プログラムを作動するコンピュータとロボット制御ボードがネットワーク接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch amber_ros_driver amber_control.launch
```

### 実機とMoveIt!との立ち上げ

本プログラムを作動するコンピュータとロボット制御ボードがネットワーク接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch amber_moveit_config demo_hw.launch
```

### GazeboシミュレーターとMoveIt!との立ち上げ

本プログラムを作動するコンピュータとロボット制御ボードが接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch amber_moveit_config demo_gazebo.launch
```

*Gazeboが立ち上がったら、再生ボタンを押してください。*

### modelの指定 ###

"model"オプションで、robot_descriptionで利用するurdfを指定できます。拡張子は入力しないで下さい。
`amber_model4_hw_prim`を指定すると、ブリミティブ形状(バウンディングボックス)でロボットを簡略化した
モデルを利用できます。

```bash
roslaunch amber_moveit_config demo_hw.launch model:=amber_model4_hw_prim
```

以上。
