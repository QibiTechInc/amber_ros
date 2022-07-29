#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
import copy
import numpy as np
from amber_ros_driver.amber_driver import AmberDriver


class DummyHR4CCOM(object):
    def __init__(self):
        self._dof = 7
        self._control_modes = [1] * self._dof
        self._joint_angle = [0.0] * self._dof

    def set_dof(self, dof):
        self._dof = dof

    def open(self, ipaddr, port, dof, model):
        return 0

    def start(self, dev):
        return 1

    def stop(self, dev):
        pass

    def close(self, dev):
        pass

    def ping(self, dev):
        return 1

    def set_control_mode(self, dev, control_modes):
        self._control_modes = copy.copy(control_modes)

    def get_control_mode(self, dev):
        return self._control_modes

    def get_joint_angle(self, dev):
        return self._joint_angle

    def get_joint_current(self, dev):
        return [0.0] * self._dof

    def get_joint_torque(self, dev):
        return [0.0] * self._dof

    def get_joint_speed(self, dev):
        return [0.0] * self._dof

    def set_joint_trajectory(self, dev, abs_goal_references, goal_time,
                             interpolation_method, mask_list):
        self._joint_angle = copy.copy(abs_goal_references)

    def servo_on(self, dev, joint_no):
        pass

    def servo_off(self, dev, joint_no):
        pass

    def servo_all_on(self, dev):
        pass

    def servo_all_off(self, dev):
        pass

    def alarm_reset(self, dev, joint_no):
        pass

    def wait_interpolation(self, dev):
        pass

    def calibrate_joint(self, dev, joint_no, calibration_angle):
        pass

    def calibrate_joint_from_memory(self, dev, joint_no,
                                    calibration_angle, memory_angle):
        pass


class AmberDriverTestPy(unittest.TestCase):
    def setUp(self):
        self._hr4c_comm = DummyHR4CCOM()
        self._amber_driver = AmberDriver(self._hr4c_comm,
                                         "172.16.1.100",
                                         54321)

    def check_list_almost_equal(self, actual, expected):
        for i in range(len(actual)):
            self.assertAlmostEqual(actual[i],
                                   expected[i],
                                   delta=1e-8)

    def test_get_relative_reference(self):
        self._amber_driver.set_control_mode([1, 1, 1, 1, 1, 1, 1])
        goal_references = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        goal_references_np = np.array(goal_references)
        actual = self._amber_driver._get_relative_reference(goal_references_np)
        expected = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        self._hr4c_comm.set_joint_trajectory(0, actual, 3.0, 1, [0] * 7)
        actual = self._amber_driver._get_relative_reference(goal_references_np)
        expected = [0.2, 0.4, 0.4, 0.8, 1.0, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        self._hr4c_comm.set_joint_trajectory(0, actual, 3.0, 1, [0] * 7)
        self._amber_driver.set_control_mode([3, 1, 3, 2, 4, 1, 1])
        actual = self._amber_driver._get_relative_reference(goal_references_np)
        expected = [0.2, 0.8, 0.4, 0.8, 1.0, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

    def test_calc_actual_angles(self):
        joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_actual_angles(joints)
        expected = [0.1, 0.2, 0.5, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, -0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_actual_angles(joints)
        expected = [0.1, -0.2, 0.1, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, 0.2, -0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_actual_angles(joints)
        expected = [0.1, 0.2, -0.1, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, -0.2, -0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_actual_angles(joints)
        expected = [0.1, -0.2, -0.5, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

    def test_calc_model_angles(self):
        joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_model_angles(joints)
        expected = [0.1, 0.2, 0.1, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, -0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_model_angles(joints)
        expected = [0.1, -0.2, 0.5, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, 0.2, -0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_model_angles(joints)
        expected = [0.1, 0.2, -0.5, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, -0.2, -0.3, 0.4, 0.5, 0.0, 0.0]
        actual = self._amber_driver.calc_model_angles(joints)
        expected = [0.1, -0.2, -0.1, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

    def test_get_dof(self):
        self.assertEqual(self._amber_driver.get_dof(), 7)

    def test_set_get_joint_angle(self):
        joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        self._amber_driver.set_joint_trajectory(joints,
                                                3.0,
                                                relative=False)
        actual = self._amber_driver.get_joint_angle()
        expected = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)

        joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.0, 0.0]
        self._amber_driver.set_joint_trajectory(joints,
                                                3.0,
                                                relative=True)
        actual = self._amber_driver.get_joint_angle()
        expected = [0.2, 0.4, 0.6, 0.8, 1.0, 0.0, 0.0]
        self.check_list_almost_equal(actual, expected)


if __name__ == '__main__':
    import rostest
    rostest.unitrun('amber_ros_driver', 'amber_driver_test.py',
                    AmberDriverTestPy)
