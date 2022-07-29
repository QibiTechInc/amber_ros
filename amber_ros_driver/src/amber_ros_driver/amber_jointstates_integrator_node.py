# -*- coding: utf-8 -*-
import rospy
import message_filters
from sensor_msgs.msg import JointState


class AmberJointStatesIntegratorNode(object):
    def __init__(self):
        rospy.init_node('amber_jointstates_integrator_node')

        # parameters
        self._left_group_name = rospy.get_param('~left_group_name', 'amber_left')
        self._right_group_name = rospy.get_param('~right_group_name', 'amber_right')
        self._fps = rospy.get_param('~fps', 100.0)

        # publishers
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=100)

        # subscribers
        sub1 = message_filters.Subscriber(self._left_group_name + "/joint_states",
                                          JointState)
        sub2 = message_filters.Subscriber(self._right_group_name + "/joint_states",
                                          JointState)
        delay = 1 / self._fps
        ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 100, delay)
        ts.registerCallback(self._js_callback)

    def _js_callback(self, msg1, msg2):
        js = JointState()
        js.header = msg1.header

        integrated_names = list(msg1.name)
        integrated_names.extend(msg2.name)
        js.name = integrated_names

        integrated_position = list(msg1.position)
        integrated_position.extend(msg2.position)
        js.position = integrated_position

        integrated_velocity = list(msg1.velocity)
        integrated_velocity.extend(msg2.velocity)
        js.velocity = integrated_velocity

        integrated_effort = list(msg1.effort)
        integrated_effort.extend(msg2.effort)
        js.effort = integrated_effort

        self._joint_pub.publish(js)

    def run(self):
        rospy.spin()
