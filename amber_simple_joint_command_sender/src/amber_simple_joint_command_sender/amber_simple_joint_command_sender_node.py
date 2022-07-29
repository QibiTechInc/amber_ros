# -*- coding: utf-8 -*-
import rospy
import actionlib
import copy
import sys
from amber_simple_joint_command_sender.msg import SimpleJointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class AmberSimpleJointCommandSenderNode(object):
    def __init__(self):
        rospy.init_node('amber_simple_joint_command_sender_node')

        # parameters
        self._action_name = rospy.get_param('~action_name', 'trajectory_joints')
        self._time_tolerance = rospy.get_param('~time_tolerance', 0.1)

        # clients
        self._client = actionlib.SimpleActionClient(self._action_name,
                                                    FollowJointTrajectoryAction)
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(self._time_tolerance)
        self._goal.goal_time_tolerance = self._goal_time_tolerance

        # subscriber
        rospy.Subscriber("simple_trajectory_command", SimpleJointTrajectory, self._command_callback)

    def _command_callback(self, msg):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(msg.reference)
        point.time_from_start = rospy.Duration(msg.goal_time)
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory Action")
        else:
            self._goal.trajectory.points = [point]
            self._goal.trajectory.header.stamp = rospy.Time.now()
            self._client.send_goal(self._goal)

    def run(self):
        rospy.spin()
