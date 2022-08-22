# -*- coding: utf-8 -*-
import rospy
from amber_ros_driver.amber_driver import AmberDriver
from amber_ros_driver.hr4c_comm import HR4CCOM
from amber_ros_driver.srv import (
    GetInt8Array,
    GetInt8ArrayResponse,
    SetInt8Array,
    SetInt8ArrayResponse,
    SetJointNo,
    SetJointNoResponse,
    SetJointTrajectory,
    SetJointTrajectoryResponse
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse


class AmberROSDriverNode(object):
    def __init__(self):
        rospy.init_node('amber_ros_driver_node')

        # parameters
        ipaddr = rospy.get_param('~ip_addr', '172.16.1.101')
        port = rospy.get_param('~port', 54321)
        self._joint_names = rospy.get_param('~joint_names', ['j1',
                                                             'j2',
                                                             'j3',
                                                             'j4',
                                                             'j5',
                                                             'j6_a',
                                                             'j6_b'])
        self._sampling_rate = rospy.get_param('~sampling_rate', 100)

        # attributes
        self._amber_driver = AmberDriver(HR4CCOM(), ipaddr, port)
        self._joint_angles = None
        self._joint_currents = None
        self._joint_torques = None
        self._joint_speeds = None

        # publishers
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=100)
        self._current_pub = rospy.Publisher('joint_currents', Float64MultiArray, queue_size=100)

        # services
        rospy.Service('servo_all_on', Empty, self.handle_servo_all_on)
        rospy.Service('servo_all_off', Empty, self.handle_servo_all_off)
        rospy.Service('wait_interpolation', Empty, self.handle_wait_interpolation)
        rospy.Service('go_to_home_position', Empty, self.handle_go_homepos)
        rospy.Service('go_to_rest_position', Empty, self.handle_go_restpos)
        rospy.Service('alarm_reset', SetJointNo, self.handle_alarm_reset)
        rospy.Service('servo_on', SetJointNo, self.handle_servo_on)
        rospy.Service('servo_off', SetJointNo, self.handle_servo_off)
        rospy.Service('set_control_mode', SetInt8Array, self.handle_set_control_mode)
        rospy.Service('get_control_mode', GetInt8Array, self.handle_get_control_mode)
        rospy.Service('set_joint_trajectory', SetJointTrajectory, self.handle_set_joint_trajectory)
        rospy.Service('enable_zerog_mode', SetBool, self.handle_enable_zerog_mode)

    def shutdown(self):
        self._amber_driver.close_device()

    def handle_servo_all_on(self, req):
        self._amber_driver.servo_all_on()
        return EmptyResponse()

    def handle_servo_all_off(self, req):
        self._amber_driver.servo_all_off()
        return EmptyResponse()

    def handle_wait_interpolation(self, req):
        self._amber_driver.wait_interpolation()
        return EmptyResponse()

    def handle_go_homepos(self, req):
        self._amber_driver.go_to_home_position()
        return EmptyResponse()

    def handle_go_restpos(self, req):
        self._amber_driver.go_to_rest_position()
        return EmptyResponse()

    def handle_alarm_reset(self, req):
        self._amber_driver.alarm_reset(req.joint_no)
        return SetJointNoResponse(result=True)

    def handle_servo_on(self, req):
        self._amber_driver.servo_on(req.joint_no)
        return SetJointNoResponse(result=True)

    def handle_servo_off(self, req):
        self._amber_driver.servo_off(req.joint_no)
        return SetJointNoResponse(result=True)

    def handle_set_control_mode(self, req):
        control_modes = req.arg
        if len(control_modes) != self._amber_driver.get_dof():
            rospy.logwarn('Invalid data number: ' + str(len(control_modes)))
            return SetInt8ArrayResponse(result=False)
        else:
            self._amber_driver.set_control_mode(control_modes)

        return SetInt8ArrayResponse(result=True)

    def handle_get_control_mode(self, req):
        control_modes = self._amber_driver.get_control_mode()

        return GetInt8ArrayResponse(res=control_modes)

    def handle_set_joint_trajectory(self, req):
        self._amber_driver.set_joint_trajectory(req.target_angles,
                                                req.goal_time,
                                                mask=req.mask,
                                                interpolation_method=req.interpolation_method,
                                                relative=req.relative)
        return SetJointTrajectoryResponse()

    def handle_enable_zerog_mode(self, req):
        self._amber_driver.enable_zerog_mode(req.data)
        return SetBoolResponse(success=True, message='')

    def publish_joint_state(self, angles, velocities, efforts):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self._joint_names
        js.position = angles
        js.velocity = velocities
        js.effort = efforts
        self._joint_pub.publish(js)

    def publish_joint_current(self, currents):
        self._current_pub.publish(Float64MultiArray(data=currents))

    def update_sensor_data(self):
        self._joint_angles = self._amber_driver.get_joint_angle()
        self._joint_currents = self._amber_driver.get_joint_current()
        self._joint_torques = self._amber_driver.get_joint_torque()
        self._joint_speeds = self._amber_driver.get_joint_speed()

    def run(self):
        r = rospy.Rate(self._sampling_rate)
        while not rospy.is_shutdown():
            self.update_sensor_data()
            self.publish_joint_state(self._joint_angles,
                                     self._joint_speeds,
                                     self._joint_torques)
            self.publish_joint_current(self._joint_currents)
            r.sleep()
