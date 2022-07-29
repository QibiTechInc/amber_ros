# 概要

目標関節角度とゴール到達時間をトピックで受け取り、FollowJointTrajectoryActionのデータにして目標値を送信するROSパッケージです。

# Nodes

## amber_simple_joint_command_sender_node

### Subscribed topics

- simple_trajectory_command (amber_simple_joint_command_sender/SimpleJointTrajectory)

目標関節角度とゴール到達時間が定義されたトピック。

### Published topics

なし

### Published actions

プライベートパラメータaction_nameで指定されたActionに対してsend_goal()を行う

### Parameters

- action_name 対応するFollowJointTrajectoryActionの名前
- time_tolerance  FollowJointTrajectoryActionのgoal_time_toleranceに設定する許容時間。デフォルトは0.1秒。

# Launch Files

## simple_joint_command.launch

双腕のそれぞれに対して設定されたFollowJointTrajectoryActionに対して接続するノードを起動するlaunchファイル


# 動作確認方法

``` bash
$ roslaunch amber_simple_joint_command_sender simple_joint_command.launch
```
