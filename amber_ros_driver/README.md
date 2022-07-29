# 概要

Amber実機のROSインタフェースを提供するROSパッケージです。

# Nodes

## amber_ros_driver_node

Amber実機と通信を行い、センサ値をトピックの形で出力するとともに各種機能をサービスで提供するノード。

### Subscribed topics

なし

### Published topics

- joint_states (sensor_msgs/JointState)

 関節角度、トルク、関節速度を配信するトピック

- joint_currents (std_msgs/Float64MultiArray)

 関節の電流値を配信するトピック

### Services

- servo_all_on(std_srvs/Empty)

 全軸のサーボをONにする

- servo_all_off(std_srvs/Empty)

 全軸のサーボをOFFにする

- wait_interpolation(std_srvs/Empty)

 補間動作が終了するのを待つ

- go_to_home_position(std_srvs/Empty)

 ホーム位置に移動する

- go_to_rest_position(std_srvs/Empty)

 アームレスト上の待機位置に移動する

- alarm_reset(amber_ros_driver/SetJointNo)

 関節を指定してアラームをリセットする

- servo_on(amber_ros_driver/SetJointNo)

 関節を指定してサーボをONにする

- servo_off(amber_ros_driver/SetJointNo)

 関節を指定してサーボをOFFにする

- set_control_mode(amber_ros_driver/SetInt8Array)

 各関節の制御モードを設定する。1: 位置制御、2: 速度制御、3: 電流制御、4: トルク制御。

- get_control_mode(amber_ros_driver/GetInt8Array)

 各関節の制御モードを取得する

- set_joint_trajectory(amber_ros_driver/SetJointTrajectory)

 各関節に対して目標値と遷移時間を指定して補間動作を実行する

### Parameters

- ip_addr 接続しようとする腕を制御するコントローラのIPアドレス。デフォルト値は '172.16.1.101'
- port 接続しようとする腕を制御するコントローラのポート。デフォルト値は54321
- joint_names 接続しようとする腕の関節名のリスト。デフォルト値は　['j1', 'j2', 'j3', 'j4', 'j5', 'j6_a', 'j6_b']
- sampling_rate センサー値を出力する周期。デフォルトは100Hz

## amber_action_server_node

amber_ros_driver_nodeと連携し、Actionのインタフェースを提供するノード。

### Subscribed topics
- joint_states (sensor_msgs/JointState)

 対応するamber_ros_driver_nodeが配信する関節角度、トルク、関節速度のトピック

- joint_currents (std_msgs/Float64MultiArray)

 対応するamber_ros_driver_nodeが配信する関節の電流値のトピック

### Published topics

なし

### Actions

- {パラメータarm_trajectory_action_name}(control_msgs/FollowJointTrajectoryAction)

 moveitの設定に合わせて5自由度の腕をFollowJointTrajectoryActionで動かすためのアクション

- {パラメータgripper_action_name}(control_msgs/GripperCommandAction)

 moveitの設定に合わせて2自由度のGripperをGripperCommandActionで動かすためのアクション

- {パラメータgohome_action_name}(amber_ros_driver/EmptyAction)

 ホーム位置への移動を行うアクション

- {パラメータgorest_action_name}(amber_ros_driver/EmptyAction)

 アームレスト上の待機位置への移動を行うアクション

### Parameters

 - arm_trajectory_action_name 腕の軌道を指定して動かすアクションの名前。デフォルト値は'trajectory_joints'
 - gripper_action_name Gripperを動かすアクションの名前。デフォルト値は'gripper_command'
 - gohome_action_name ホーム位置への移動を行うアクションの名前。デフォルト値は'go_to_home_position'
 - gorest_action_name アームレスト上の待機位置への移動を行うアクションの名前。デフォルト値は'go_to_rest_position'
 - service_namespace 接続する腕の提供するサービスのネームスペース。デフォルト値は'amber_left'
 - joint_names 接続しようとする腕の関節名のリスト。デフォルト値は　['j1', 'j2', 'j3', 'j4', 'j5', 'j6_a', 'j6_b']
 - gripper_target_time GripperをGripperCommandActionで動かす際の遷移時間。デフォルト値は2秒
 - sampling_rate データの送られてくる周期。デフォルト値は100Hz。
 - gripper_close_offset GripperCommandActionでmax_effortに達した後に指関節に与えるオフセット。デフォルト値は0.25ラジアン
 - gripper_adapt_time GripperCommandActionでmax_effortに達した後に指関節をオフセット分動かす際の遷移時間。デフォルト値は0.2秒
 - follow_mode FollowJointTrajectoryの軌道のなぞり方。デフォルト値は'waypoint'。'direct'を指定すると途中経路は通らず最終目標値を目指して遷移する
 - moveit_control_rate moveitの制御周期。デフォルトは25Hz
 - minimum_control_duration follow_modeにてwaypoint指定時にどのぐらいの間隔でwaypointを作成するか決めるための最小動作時間。moveitの制御周期で直接安定的に動かすことは難しいため、ある程度データを間引きしている

## amber_jointstates_integrator_node

moveit設定と合わせるために左右の腕のjoint_stateを統合し一つにまとめたjoint_stateとしてpublishするノード。

### Subscribed topics
- {パラメータleft_group_name}/joint_states (sensor_msgs/JointState)

 左腕のjoint_statesトピック

- {パラメータright_group_name}/joint_states (sensor_msgs/JointState)

 右腕のjoint_statesトピック

### Published topics

- joint_states (sensor_msgs/JointState)

 左右の腕が統合されて配信されるJointStateトピック。

### Parameters

- left_group_name 左腕のグループ名。デフォルト値は'amber_left'
- right_group_name 右腕のグループ名。デフォルト値は'amber_right'
- fps 期待するトピック出力周期。デフォルトは100.0Hz

# Launch Files

## amber_ros_driver.launch

左右の腕に関してそれぞれパラメータを設定してamber_ros_driver_node, amber_action_server_node, amber_jointstates_integrator_nodeを起動するlaunchファイル

## amber_control.launch

上記amber_ros_driver.launchで起動されるノード群に加えてモデルの読み込みとrobot_state_publisherの起動も行うlaunchファイル。amber_moveit_configからはこのlaunchファイルが呼ばれる。

## amber_joint_check.launch

rvizを起動し、現在の関節角度を視覚的に確認するためのlaunchファイル

# 動作確認方法

AmberのROSのインタフェースを立ち上げるには以下のlaunchファイルを起動します。

``` bash
$ roslaunch amber_ros_driver amber_control.launch
```

キャリブレーション後など、関節角度が意図した形になっていることを確認するには、amber_control.launchの代わりに以下のlaunchファイルを起動します。

``` bash
$ roslaunch amber_ros_driver amber_joint_check.launch
```
