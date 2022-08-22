# -*- coding: utf-8 -*-
import ctypes
import os
from ctypes import c_int, c_char_p, c_double, POINTER


class HR4CCOM(object):
    def __init__(self, dof=7):
        self._dof = dof

        hr4c_libdir = os.environ['HR4C_LIBDIR']
        self._hr4c_comm_so = ctypes.cdll.LoadLibrary(hr4c_libdir + "/libhr4c_comm.so")
        self._hr4c_comm_so.hr4capi_open.restype = c_int
        self._hr4c_comm_so.hr4capi_open.argtypes = [c_char_p, c_int, c_int, c_char_p]
        self._hr4c_comm_so.hr4capi_close.restype = c_int
        self._hr4c_comm_so.hr4capi_close.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_start.restype = c_int
        self._hr4c_comm_so.hr4capi_start.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_stop.restype = c_int
        self._hr4c_comm_so.hr4capi_stop.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_set_joint_reference.restype = None
        self._hr4c_comm_so.hr4capi_set_joint_reference.argtypes = [c_int,
                                                                   POINTER(c_double),
                                                                   POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_set_joint_trajectory.restype = None
        self._hr4c_comm_so.hr4capi_set_joint_trajectory.argtypes = [c_int,
                                                                    POINTER(c_double),
                                                                    c_double,
                                                                    c_int,
                                                                    POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_get_joint_angle.restype = None
        self._hr4c_comm_so.hr4capi_get_joint_angle.argtypes = [c_int, POINTER(c_double)]
        self._hr4c_comm_so.hr4capi_wait_interpolation.restype = None
        self._hr4c_comm_so.hr4capi_wait_interpolation.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_set_control_mode.restype = None
        self._hr4c_comm_so.hr4capi_set_control_mode.argtypes = [c_int, POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_start_logging.restype = None
        self._hr4c_comm_so.hr4capi_start_logging.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_stop_logging.restype = None
        self._hr4c_comm_so.hr4capi_stop_logging.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_clear_logs.restype = None
        self._hr4c_comm_so.hr4capi_clear_logs.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_get_lognum.restype = c_int
        self._hr4c_comm_so.hr4capi_get_lognum.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_get_loglist.restype = None
        self._hr4c_comm_so.hr4capi_get_loglist.argtypes = [c_int, c_char_p]
        self._hr4c_comm_so.hr4capi_get_log.restype = c_int
        self._hr4c_comm_so.hr4capi_get_log.argtypes = [c_int, c_char_p]
        self._hr4c_comm_so.hr4capi_servo_on.restype = None
        self._hr4c_comm_so.hr4capi_servo_on.argtypes = [c_int, POINTER(c_int), c_int]
        self._hr4c_comm_so.hr4capi_servo_all_on.restype = None
        self._hr4c_comm_so.hr4capi_servo_all_on.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_servo_off.restype = None
        self._hr4c_comm_so.hr4capi_servo_off.argtypes = [c_int, POINTER(c_int), c_int]
        self._hr4c_comm_so.hr4capi_servo_all_off.restype = None
        self._hr4c_comm_so.hr4capi_servo_all_off.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_get_control_mode.restype = None
        self._hr4c_comm_so.hr4capi_get_control_mode.argtypes = [c_int, POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_get_joint_current.restype = None
        self._hr4c_comm_so.hr4capi_get_joint_current.argtypes = [c_int, POINTER(c_double)]
        self._hr4c_comm_so.hr4capi_calibrate_joint.resttype = None
        self._hr4c_comm_so.hr4capi_calibrate_joint.argtypes = [c_int, c_int, c_double]
        self._hr4c_comm_so.hr4capi_calibrate_joint_from_memory.resttype = None
        self._hr4c_comm_so.hr4capi_calibrate_joint_from_memory.argtypes = [c_int,
                                                                           c_int,
                                                                           c_double,
                                                                           c_double]
        self._hr4c_comm_so.hr4capi_alarm_reset.restype = None
        self._hr4c_comm_so.hr4capi_alarm_reset.argtypes = [c_int, POINTER(c_int), c_int]
        self._hr4c_comm_so.hr4capi_get_motor_status.restype = None
        self._hr4c_comm_so.hr4capi_get_motor_status.argtypes = [c_int, POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_force_stop.restype = None
        self._hr4c_comm_so.hr4capi_force_stop.argtypes = [c_int, POINTER(c_int), c_int]
        self._hr4c_comm_so.hr4capi_get_joint_speed.restype = None
        self._hr4c_comm_so.hr4capi_get_joint_speed.argtypes = [c_int, POINTER(c_double)]
        self._hr4c_comm_so.hr4capi_get_joint_torque.restype = None
        self._hr4c_comm_so.hr4capi_get_joint_torque.argtypes = [c_int, POINTER(c_double)]
        self._hr4c_comm_so.hr4capi_start_teaching.restype = c_int
        self._hr4c_comm_so.hr4capi_start_teaching.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_stop_teaching.restype = c_int
        self._hr4c_comm_so.hr4capi_stop_teaching.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_replay_motion.restype = c_int
        self._hr4c_comm_so.hr4capi_replay_motion.argtypes = [c_int, c_int, POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_get_motion_list.restype = None
        self._hr4c_comm_so.hr4capi_get_motion_list.argtypes = [c_int, c_char_p]
        self._hr4c_comm_so.hr4capi_clear_motion.restype = c_int
        self._hr4c_comm_so.hr4capi_clear_motion.argtypes = [c_int, c_int]
        self._hr4c_comm_so.hr4capi_clear_all_motions.restype = None
        self._hr4c_comm_so.hr4capi_clear_all_motions.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_controller_shutdown.restype = None
        self._hr4c_comm_so.hr4capi_controller_shutdown.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_ping.restype = c_int
        self._hr4c_comm_so.hr4capi_ping.argtypes = [c_int]
        self._hr4c_comm_so.hr4capi_update_controller.restype = c_int
        self._hr4c_comm_so.hr4capi_update_controller.argtypes = [c_int, c_char_p]
        self._hr4c_comm_so.hr4capi_get_all_sensor_info.restype = None
        self._hr4c_comm_so.hr4capi_get_all_sensor_info.argtypes = [c_int,
                                                                   POINTER(c_double),
                                                                   POINTER(c_double),
                                                                   POINTER(c_double),
                                                                   POINTER(c_double),
                                                                   POINTER(c_int),
                                                                   POINTER(c_int)]
        self._hr4c_comm_so.hr4capi_enable_zerog_mode.restype = None
        self._hr4c_comm_so.hr4capi_enable_zerog_mode.argtypes = [c_int, c_int]

    def set_dof(self, dof):
        self._dof = dof

    def open(self, ipaddr, port, dof, model):
        return self._hr4c_comm_so.hr4capi_open(ipaddr, port, dof, model)

    def start(self, dev):
        return self._hr4c_comm_so.hr4capi_start(dev)

    def stop(self, dev):
        return self._hr4c_comm_so.hr4capi_stop(dev)

    def close(self, dev):
        return self._hr4c_comm_so.hr4capi_close(dev)

    def ping(self, dev):
        return self._hr4c_comm_so.hr4capi_ping(dev)

    def set_control_mode(self, dev, control_modes):
        ref_c_array = (ctypes.c_int * len(control_modes))(*control_modes)
        self._hr4c_comm_so.hr4capi_set_control_mode(dev, ref_c_array)

    def get_control_mode(self, dev):
        res_array = [0] * self._dof
        res_c_array = (ctypes.c_int * len(res_array))(*res_array)
        self._hr4c_comm_so.hr4capi_get_control_mode(dev, res_c_array)
        return [rc for rc in res_c_array]

    def get_joint_angle(self, dev):
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        self._hr4c_comm_so.hr4capi_get_joint_angle(dev, res_c_array)
        return [rc for rc in res_c_array]

    def get_joint_current(self, dev):
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        self._hr4c_comm_so.hr4capi_get_joint_current(dev, res_c_array)
        return [rc for rc in res_c_array]

    def get_joint_torque(self, dev):
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        self._hr4c_comm_so.hr4capi_get_joint_torque(dev, res_c_array)
        return [rc for rc in res_c_array]

    def get_joint_speed(self, dev):
        res_array = [0.0] * self._dof
        res_c_array = (ctypes.c_double * len(res_array))(*res_array)
        self._hr4c_comm_so.hr4capi_get_joint_speed(dev, res_c_array)
        return [rc for rc in res_c_array]

    def set_joint_trajectory(self, dev, abs_goal_references, goal_time,
                             interpolation_method, mask_list):
        ref_c_array = (ctypes.c_double * len(abs_goal_references))(*abs_goal_references)
        mask_c_array = (ctypes.c_int * len(mask_list))(*mask_list)
        self._hr4c_comm_so.hr4capi_set_joint_trajectory(dev,
                                                        ref_c_array,
                                                        goal_time,
                                                        interpolation_method,
                                                        mask_c_array)

    def servo_on(self, dev, joint_no):
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        self._hr4c_comm_so.hr4capi_servo_on(dev, jn_c_array, 1)

    def servo_off(self, dev, joint_no):
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        self._hr4c_comm_so.hr4capi_servo_off(dev, jn_c_array, 1)

    def servo_all_on(self, dev):
        self._hr4c_comm_so.hr4capi_servo_all_on(dev)

    def servo_all_off(self, dev):
        self._hr4c_comm_so.hr4capi_servo_all_off(dev)

    def alarm_reset(self, dev, joint_no):
        joint_nos = [joint_no]
        jn_c_array = (ctypes.c_int * len(joint_nos))(*joint_nos)
        self._hr4c_comm_so.hr4capi_alarm_reset(dev, jn_c_array, 1)

    def wait_interpolation(self, dev):
        self._hr4c_comm_so.hr4capi_wait_interpolation(dev)

    def calibrate_joint(self, dev, joint_no, calibration_angle):
        self._hr4c_comm_so.hr4capi_calibrate_joint(dev,
                                                   joint_no,
                                                   calibration_angle)

    def calibrate_joint_from_memory(self, dev, joint_no,
                                    calibration_angle, memory_angle):
        self._hr4c_comm_so.hr4capi_calibrate_joint_from_memory(dev,
                                                               joint_no,
                                                               calibration_angle,
                                                               memory_angle)

    def enable_zerog_mode(self, dev, on_off):
        if on_off:
            self._hr4c_comm_so.hr4capi_enable_zerog_mode(dev, 1)
        else:
            self._hr4c_comm_so.hr4capi_enable_zerog_mode(dev, 0)
