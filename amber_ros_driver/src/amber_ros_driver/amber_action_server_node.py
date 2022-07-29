# -*- coding: utf-8 -*-
import rospy
import actionlib
import copy
import numpy as np
import math
from amber_ros_driver.msg import (
    EmptyAction,
    EmptyFeedback,
    EmptyResult)
from amber_ros_driver.srv import SetJointTrajectory, SetJointTrajectoryRequest
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult)
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty


class AmberActionServerNode(object):
    def __init__(self):
        rospy.init_node('amber_action_server_node')

        # parameters
        arm_trajectory_action_name = rospy.get_param('~arm_trajectory_action_name',
                                                     'trajectory_joints')
        gripper_action_name = rospy.get_param('~gripper_action_name',
                                              'gripper_command')
        gohome_action_name = rospy.get_param('~gohome_action_name',
                                             'go_to_home_position')
        gorest_action_name = rospy.get_param('~gorest_action_name',
                                             'go_to_rest_position')
        self._srv_namespace = rospy.get_param('~service_namespace',
                                              'amber_left')
        self._joint_names = rospy.get_param('~joint_names', ['j1',
                                                             'j2',
                                                             'j3',
                                                             'j4',
                                                             'j5',
                                                             'j6_a',
                                                             'j6_b'])
        self._gripper_target_time = rospy.get_param('~gripper_target_time', 2.0)
        self._sampling_rate = rospy.get_param('~sampling_rate', 100)
        self._gripper_close_offset = rospy.get_param('~gripper_close_offset', 0.25)
        self._gripper_adapt_time = rospy.get_param('~gripper_adapt_time', 0.2)
        self._follow_mode = rospy.get_param('~follow_mode', 'waypoint')
        self._moveit_control_rate = rospy.get_param('~moveit_control_rate', 25.0)
        self._minimum_control_duration = rospy.get_param('~minimum_control_duration', 0.6)

        # attributes
        self._arm_mask = [0, 0, 0, 0, 0, 1, 1]
        self._gripper_mask = [1, 1, 1, 1, 1, 0, 0]
        self._gripper_stall_current = 0.7
        self._gripper_command_tolerance = 0.05
        self._close_hand_angle = 0.6
        self._finger_link_length = 0.05
        self._finger_base_length = 0.06
        self._joint_angles = None
        self._joint_currents = None
        self._joint_torques = None
        self._joint_speeds = None
        self._joint_accs = None

        # subscribers
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)
        rospy.Subscriber("joint_currents", Float64MultiArray, self._joint_current_cb)

        # initlize actions
        self._arm_trajectory_as = \
            actionlib.SimpleActionServer(arm_trajectory_action_name,
                                         FollowJointTrajectoryAction,
                                         execute_cb=self.execute_arm_trajectory_cb,
                                         auto_start=False)
        self._arm_trajectory_as_feedback = FollowJointTrajectoryFeedback()
        self._arm_trajectory_as_feedback.joint_names = self._joint_names
        self._arm_trajectory_as_feedback.desired = JointTrajectoryPoint()
        self._arm_trajectory_as_feedback.actual = JointTrajectoryPoint()
        self._arm_trajectory_as_feedback.error = JointTrajectoryPoint()
        self._arm_trajectory_as_result = FollowJointTrajectoryResult()

        self._gripper_command_as = \
            actionlib.SimpleActionServer(gripper_action_name,
                                         GripperCommandAction,
                                         execute_cb=self.execute_gripper_command_cb,
                                         auto_start=False)
        self._gripper_command_as_feedback = GripperCommandFeedback()
        self._gripper_command_as_result = GripperCommandResult()

        self._go_home_pos_as = \
            actionlib.SimpleActionServer(gohome_action_name,
                                         EmptyAction,
                                         execute_cb=self.execute_go_homepos_cb,
                                         auto_start=False)

        self._go_rest_pos_as = \
            actionlib.SimpleActionServer(gorest_action_name,
                                         EmptyAction,
                                         execute_cb=self.execute_go_restpos_cb,
                                         auto_start=False)

        # start actions
        self._arm_trajectory_as.start()
        self._gripper_command_as.start()
        self._go_home_pos_as.start()
        self._go_rest_pos_as.start()

    def set_joint_treajectory_srv_client(self, target_angles, goal_time, mask, method, relative):
        srv_name = self._srv_namespace + '/set_joint_trajectory'
        rospy.wait_for_service(srv_name)
        try:
            set_joint_trajectory = rospy.ServiceProxy(srv_name,
                                                      SetJointTrajectory)
            set_joint_trajectory(target_angles, goal_time, mask, method, relative)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def go_home_srv_client(self):
        srv_name = self._srv_namespace + '/go_to_home_position'
        rospy.wait_for_service(srv_name)
        try:
            go_to_home_position = rospy.ServiceProxy(srv_name, Empty)
            go_to_home_position()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def go_rest_srv_client(self):
        srv_name = self._srv_namespace + '/go_to_rest_position'
        rospy.wait_for_service(srv_name)
        try:
            go_to_rest_position = rospy.ServiceProxy(srv_name, Empty)
            go_to_rest_position()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def _joint_state_cb(self, msg):
        self._joint_angles = msg.position
        self._joint_torques = msg.effort
        if self._joint_speeds is None:
            prev_joint_speed = msg.velocity
        else:
            prev_joint_speed = np.array(self._joint_speeds)

        self._joint_speeds = msg.velocity
        accs_np = (np.array(self._joint_speeds) - prev_joint_speed) / self._sampling_rate
        self._joint_accs = accs_np.tolist()

    def _joint_current_cb(self, msg):
        self._joint_currents = msg.data

    def _get_gripper_position(self, hand_angles):
        base_length = self._finger_base_length
        link_length = self._finger_link_length
        return base_length - link_length * (math.sin(hand_angles[0]) + math.sin(hand_angles[1]))

    def _calc_hand_angles_from_gripper_position(self, position):
        try:
            base_length = self._finger_base_length
            link_length = self._finger_link_length
            theta = -1 * (math.asin((position - base_length) / (2.0 * link_length)))
            if theta > self._close_hand_angle:
                theta = self._close_hand_angle
        except ValueError:
            rospy.logerr('Invalid position!')
            return None

        return [theta, theta]

    def _get_gripper_effort(self, hand_torques):
        np_torques = np.array(hand_torques)
        return np.average(np_torques)

    def update_arm_trajectory_feedback(self, start_time):
        time_from_start = rospy.Time.now() - start_time
        self._arm_trajectory_as_feedback.header.stamp = rospy.Time.now()
        self._arm_trajectory_as_feedback.desired.time_from_start = time_from_start
        self._arm_trajectory_as_feedback.actual.time_from_start = time_from_start
        self._arm_trajectory_as_feedback.error.time_from_start = time_from_start

        # update only actual data
        if self._joint_angles is not None:
            self._arm_trajectory_as_feedback.actual.positions = self._joint_angles[0:5]
        if self._joint_speeds is not None:
            self._arm_trajectory_as_feedback.actual.velocities = self._joint_speeds[0:5]
        if self._joint_accs is not None:
            self._arm_trajectory_as_feedback.actual.accelerations = self._joint_accs[0:5]
        if self._joint_torques is not None:
            self._arm_trajectory_as_feedback.actual.effort = self._joint_torques[0:5]

        # publish feedback
        self._arm_trajectory_as.publish_feedback(self._arm_trajectory_as_feedback)

    def get_gripper_status(self, target_pos):
        hand_angles = self._joint_angles[5:]
        hand_torques = self._joint_torques[5:]
        current_position = self._get_gripper_position(hand_angles)
        current_effort = self._get_gripper_effort(hand_torques)
        hand_currents = self._joint_currents[5:]
        if any([x > self._gripper_stall_current for x in hand_currents]):
            stalled = True
        else:
            stalled = False

        if abs(target_pos - current_position) < self._gripper_command_tolerance:
            reached_goal = True
        else:
            reached_goal = False

        return current_position, current_effort, stalled, reached_goal

    def update_gripper_command_feedback(self, target_pos):
        position, effort, stalled, readed_goal = self.get_gripper_status(target_pos)
        self._gripper_command_as_feedback.position = position
        self._gripper_command_as_feedback.effort = effort
        self._gripper_command_as_feedback.stalled = stalled
        self._gripper_command_as_feedback.reached_goal = readed_goal

        # publish feedback
        self._gripper_command_as.publish_feedback(self._gripper_command_as_feedback)

    def update_gripper_comand_result(self, target_pos):
        position, effort, stalled, readed_goal = self.get_gripper_status(target_pos)
        self._gripper_command_as_result.position = position
        self._gripper_command_as_result.effort = effort
        self._gripper_command_as_result.stalled = stalled
        self._gripper_command_as_result.reached_goal = readed_goal

    def execute_arm_trajectory_waypoint(self, goal):
        rospy.loginfo('waypoint mode')
        control_cycle_time = 1.0 / self._moveit_control_rate

        # 短時間で繰り返し送るケースは不安定になりがちなので間引きする
        thining_rate = int(self._minimum_control_duration / control_cycle_time)
        thinned_points = goal.trajectory.points[::thining_rate]

        # 最初の点を除き、丁度間引き点が最終点にならない場合は、最終点を追加する
        thinned_points = thinned_points[1:]
        if (len(goal.trajectory.points) - 1) % thining_rate != 0:
            thinned_points.append(goal.trajectory.points[-1])

        for pt in thinned_points:
            start_time = rospy.Time.now()
            target_angles = list(pt.positions)
            target_angles.extend([0.0, 0.0])
            goal_time_sec = self._minimum_control_duration

            rospy.loginfo('Waypoint: ' + str(target_angles))
            rospy.loginfo('Goaltime: ' + str(goal_time_sec))
            self.set_joint_treajectory_srv_client(target_angles,
                                                  goal_time_sec,
                                                  self._arm_mask,
                                                  SetJointTrajectoryRequest.MINJERK,
                                                  False)
            r = rospy.Rate(100)
            target_time = start_time + rospy.Duration(goal_time_sec)
            while rospy.Time.now() < target_time:
                self.update_arm_trajectory_feedback(start_time)
                r.sleep()
            rospy.loginfo('Reached waypoint')
            rospy.logdebug('elapsed: ' + str((rospy.Time.now() - start_time).to_sec()) + 's')

    def execute_arm_trajectory_direct(self, goal):
        rospy.loginfo('direct mode')
        pt = goal.trajectory.points[-1]
        start_time = rospy.Time.now()
        target_time = start_time + pt.time_from_start
        target_angles = list(pt.positions)
        target_angles.extend([0.0, 0.0])
        goal_time_sec = pt.time_from_start.to_sec()

        rospy.loginfo('Waypoint: ' + str(target_angles))
        rospy.loginfo('Goaltime: ' + str(goal_time_sec))
        self.set_joint_treajectory_srv_client(target_angles,
                                              goal_time_sec,
                                              self._arm_mask,
                                              SetJointTrajectoryRequest.MINJERK,
                                              False)
        r = rospy.Rate(100)
        while rospy.Time.now() < target_time:
            self.update_arm_trajectory_feedback(start_time)
            r.sleep()

    def execute_arm_trajectory_cb(self, goal):
        rospy.loginfo('Accept goal for arm trajectory')
        if self._follow_mode == 'direct':
            self.execute_arm_trajectory_direct(goal)
        else:
            # waypoint
            self.execute_arm_trajectory_waypoint(goal)

        rospy.loginfo('Finished following trajectory')
        self._arm_trajectory_as_result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self._arm_trajectory_as.set_succeeded(self._arm_trajectory_as_result)

    def check_reached_max_effort(self, max_effort):
        if max_effort <= 0.0:
            # max_effortが0以下の場合は位置チェックのみで動かす
            return False

        hand_torques = self._joint_torques[5:]
        current_effort = self._get_gripper_effort(hand_torques)
        if current_effort > max_effort:
            return True
        else:
            return False

    def execute_gripper_command_cb(self, goal):
        target_angles = [0] * 5
        hand_angles = self._calc_hand_angles_from_gripper_position(goal.command.position)
        if hand_angles is not None:
            target_angles.extend(hand_angles)
        else:
            target_angles = [0] * 7

        self.set_joint_treajectory_srv_client(target_angles,
                                              self._gripper_target_time,
                                              self._gripper_mask,
                                              SetJointTrajectoryRequest.MINJERK,
                                              False)
        r = rospy.Rate(100)
        target_time = rospy.Time.now() + rospy.Duration.from_sec(self._gripper_target_time)
        while rospy.Time.now() < target_time:
            self.update_gripper_command_feedback(goal.command.position)
            if self.check_reached_max_effort(goal.command.max_effort):
                break
            r.sleep()

        if self.check_reached_max_effort(goal.command.max_effort):
            rospy.loginfo('Beyond max effort')
            adapt_angles = copy.copy(list(self._joint_angles))
            adapt_angles[5] += self._gripper_close_offset
            adapt_angles[6] += self._gripper_close_offset
            self.set_joint_treajectory_srv_client(adapt_angles,
                                                  self._gripper_adapt_time,
                                                  self._gripper_mask,
                                                  SetJointTrajectoryRequest.MINJERK,
                                                  False)

        self.update_gripper_comand_result(goal.command.position)
        self._gripper_command_as.set_succeeded(self._gripper_command_as_result)

    def execute_go_homepos_cb(self, goal):
        self.go_home_srv_client()
        self._go_home_pos_as.publish_feedback(EmptyFeedback())
        self._go_home_pos_as.set_succeeded(EmptyResult())

    def execute_go_restpos_cb(self, goal):
        self.go_rest_srv_client()
        self._go_rest_pos_as.publish_feedback(EmptyFeedback())
        self._go_rest_pos_as.set_succeeded(EmptyResult())

    def run(self):
        rospy.spin()
