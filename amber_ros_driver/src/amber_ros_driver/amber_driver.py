# -*- coding: utf-8 -*-
import rospy
import copy
import numpy as np
import time


class AmberDriver(object):
    LINEAR = 0
    MINJERK = 1
    CONTROLMODE_POSITION = 1
    CONTROLMODE_SPEED = 2
    CONTROLMODE_CURRENT = 3
    CONTROLMODE_TORQUE = 4

    def __init__(self, hr4c_comm, ipaddr, port):
        self._dof = 7
        self._hr4c_comm = hr4c_comm
        self._hr4c_comm.set_dof(self._dof)
        model = 'amber'
        self._dev = self._hr4c_comm.open(ipaddr.encode('ascii'),
                                         port,
                                         self._dof,
                                         model.encode('ascii'))
        if self._dev >= 0:
            connected = self._hr4c_comm.start(self._dev)
        else:
            connected = 0

        # 応答も確認する
        if self._dev >= 0 and connected and self.ping():
            rospy.logdebug('Connection success')
        else:
            rospy.logerr('Connection is already established!')
            if self._dev >= 0:
                self._hr4c_comm.stop(self._dev)
                self._hr4c_comm.close(self._dev)
                raise Exception('Connection Error')

    def ping(self):
        return self._hr4c_comm.ping(self._dev)

    def close_device(self):
        if self._dev >= 0:
            self._hr4c_comm.stop(self._dev)
            self._hr4c_comm.close(self._dev)

    def _get_relative_reference(self, goal_references_np):
        current_control_modes = self.get_control_mode()
        start_angles = self.get_joint_angle()
        start_speeds = self.get_joint_speed()
        start_currents = self.get_joint_current()
        start_torques = self.get_joint_torque()

        start_references = []
        for i, cm in enumerate(current_control_modes):
            if cm == self.CONTROLMODE_POSITION:
                start_references.append(start_angles[i])
            elif cm == self.CONTROLMODE_SPEED:
                start_references.append(start_speeds[i])
            elif cm == self.CONTROLMODE_CURRENT:
                start_references.append(start_currents[i])
            elif cm == self.CONTROLMODE_TORQUE:
                start_references.append(start_torques[i])
            else:
                start_references.append(0.0)
        start_references_np = np.array(start_references)
        goal_references_np += start_references_np

        return goal_references_np

    def calc_actual_angles(self, joints):
        j1 = joints[0]
        j2 = joints[1]
        j3 = joints[2]
        actual_j3 = j3 + j2
        j4 = joints[3]
        j5 = joints[4]
        j6 = joints[5]
        j7 = joints[6]
        return [j1, j2, actual_j3, j4, j5, j6, j7]

    def calc_model_angles(self, joints):
        j1 = joints[0]
        j2 = joints[1]
        j3 = joints[2]
        model_j3 = j3 - j2
        j4 = joints[3]
        j5 = joints[4]
        j6 = joints[5]
        j7 = joints[6]
        return [j1, j2, model_j3, j4, j5, j6, j7]

    def set_control_mode(self, control_modes, no_check=False, retry_count=3):
        current_cmodes = self.get_control_mode()
        try_count = retry_count + 1

        for i in range(try_count):
            if current_cmodes != control_modes:
                self._hr4c_comm.set_control_mode(self._dev, control_modes)
                if no_check:
                    return
                else:
                    time.sleep(0.3)
                    current_cmodes = self.get_control_mode()
            else:
                # 正しく設定できた場合は抜ける
                return

        if retry_count != 0:
            # 繰り返し回数分試行してもうまくいかない場合
            rospy.logwarn('Failed to set mode, Abort: ' + str(current_cmodes))

    def get_dof(self):
        return self._dof

    def get_control_mode(self, retry_count=3):
        try_count = retry_count + 1

        for i in range(try_count):
            results = self._hr4c_comm.get_control_mode(self._dev)

            if all([x != 0 for x in results]):
                return results
            else:
                time.sleep(0.1)

        rospy.logdebug('Failed to get mode, Abort')
        return results

    def get_joint_angle(self):
        joints = self._hr4c_comm.get_joint_angle(self._dev)
        return self.calc_model_angles(joints)

    def get_joint_current(self):
        return self._hr4c_comm.get_joint_current(self._dev)

    def get_joint_torque(self):
        return self._hr4c_comm.get_joint_torque(self._dev)

    def get_joint_speed(self):
        return self._hr4c_comm.get_joint_speed(self._dev)

    def set_joint_trajectory(self, goal_references, goal_time,
                             mask=None, relative=False,
                             interpolation_method=MINJERK):
        if interpolation_method is None:
            interpolation_method = self.MINJERK

        goal_references_np = np.array(goal_references)
        if relative:
            goal_references_np = self._get_relative_reference(goal_references_np)
        abs_goal_references = goal_references_np.tolist()

        control_modes = self.get_control_mode()

        tmp_angle = copy.copy(abs_goal_references)
        current_angles = copy.copy(self.get_joint_angle())
        for i in range(len(abs_goal_references)):
            if control_modes[i] != self.CONTROLMODE_POSITION:
                tmp_angle[i] = current_angles[i]

        actual_angle = self.calc_actual_angles(tmp_angle)
        for i in range(len(abs_goal_references)):
            if control_modes[i] == self.CONTROLMODE_POSITION:
                abs_goal_references[i] = actual_angle[i]

        if mask is None:
            mask_list = [0] * self._dof
        else:
            mask_list = mask

        self._hr4c_comm.set_joint_trajectory(self._dev,
                                             abs_goal_references,
                                             goal_time,
                                             interpolation_method,
                                             mask_list)

    def servo_on(self, joint_no):
        self._hr4c_comm.servo_on(self._dev, joint_no)

    def servo_off(self, joint_no):
        self._hr4c_comm.servo_off(self._dev, joint_no)

    def servo_all_on(self):
        self._hr4c_comm.servo_all_on(self._dev)

    def servo_all_off(self):
        self._hr4c_comm.servo_all_off(self._dev)

    def alarm_reset(self, joint_no):
        self._hr4c_comm.alarm_reset(self._dev, joint_no)

    def wait_interpolation(self):
        self._hr4c_comm.wait_interpolation(self._dev)

    def calibrate_joint(self, joint_no, calibration_angle):
        self._hr4c_comm.calibrate_joint(self._dev,
                                        joint_no,
                                        calibration_angle)

    def calibrate_joint_from_memory(self, joint_no, calibration_angle, memory_angle):
        self._hr4c_comm.calibrate_joint_from_memory(self._dev,
                                                    joint_no,
                                                    calibration_angle,
                                                    memory_angle)

    def go_to_home_position(self,
                            mode=[1, 1, 1, 1, 1, 1, 1],
                            goal_time=3.0,
                            servo_on=True,
                            wait_interpolation=True):
        home_pose = [0.0, -0.057, -2.4, 0.0, 0.0, 0.0, 0.0]
        self.set_control_mode(mode, no_check=True)
        time.sleep(0.2)
        if servo_on:
            self.servo_all_on()
            time.sleep(0.5)
        self.set_joint_trajectory(home_pose, goal_time)
        if wait_interpolation:
            self.wait_interpolation()

    def go_to_rest_position(self, wait_interpolation=True):
        self.set_control_mode([1, 3, 1, 1, 1, 1, 1], no_check=True)
        self.set_joint_trajectory([0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0, mask=[1, 0, 1, 1, 1])
        if wait_interpolation:
            self.wait_interpolation()
        self.set_control_mode([1, 3, 3, 1, 1, 1, 1], no_check=True)
        time.sleep(0.1)
        self.set_joint_trajectory([0.0, 2.0, -1.0, 0.0, 0.0, 0.0, 0.0], 3.0, mask=[1, 0, 0, 1, 1])
        if wait_interpolation:
            self.wait_interpolation()
