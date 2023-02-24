# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import threading

from pyPS4Controller.controller import Controller

import control.arm_control.arm_controller
from control.arm_control.config_and_enums.joystick_input_types import (
    Button,
    ControllerStates,
    JoystickAxis,
)
from control.arm_control.interface_classes import input_device_interface


class Ps4Joystick(Controller, input_device_interface.InputDeviceInterface):
    def __init__(self, interface):
        self.interface = interface

        # shared by arm_servo_controller thread
        self.__joystick_internal_states = {
            Button.CROSS: False,
            Button.SQUARE: False,
            Button.TRIANGLE: False,
            Button.CIRCLE: False,
            Button.DOWN: False,
            Button.LEFT: False,
            Button.UP: False,
            Button.RIGHT: False,
            Button.L1: False,
            Button.R1: False,
            Button.L3: False,
            Button.R3: False,
            JoystickAxis.LEFT_HORIZONTAL: None,
            JoystickAxis.LEFT_VERTICAL: None,
            JoystickAxis.RIGHT_HORIZONTAL: None,
            JoystickAxis.RIGHT_VERTICAL: None,
            JoystickAxis.L2R2: None,
        }
        Controller.__init__(self, interface=interface, connecting_using_ds4drv=False)
        self.axis_max_value = 32767.0
        self.__arm_controller_obj: control.arm_control.arm_controller.ArmController = (
            None
        )
        self.__thread = None

    # to handle new button release inputs (on opposed to persistent button holding)
    # we would call hooks to arm_controller_obj
    # an alternative is to gather state changes in the dict object, and expect arm to handle
    # but without semaphores, there could be issues handling new inputs in quick successions
    def connect_arm_controller(self, arm_controller_obj):
        self.__arm_controller_obj = arm_controller_obj

    @property
    def arm_controller_obj(self):
        assert self.__arm_controller_obj is not None
        return self.__arm_controller_obj

    @property
    def input_states(self):
        return self.__arm_controller_obj.get_joystick_input_states()

    @property
    def controller_states(self):
        return self.__arm_controller_obj.get_controller_states()

    @property
    def arm_controller_running(self):
        return (
            self.__arm_controller_obj is not None
            and self.__arm_controller_obj.is_thread_running()
        )

    def start_thread(self):
        self.__thread = threading.Thread(target=self.listen, args={})
        self.__thread.start()
        logging.info(f"Thread for Joystick Listener started.")

    def stop_thread(self):
        if self.__thread is not None:
            self.stop = True
            # stop should terminate pyPS4Controller.controller.Controller
            # however sometimes this read_events() would hang, so we timeout at 3s
            self.__thread.join(timeout=3)
            # self.__thread = None
            logging.info(f"Thread for Joystick Listener stopped.")
        else:
            logging.warning(f"Thread for Joystick Listener ALREADY stopped.")

    def __update_input_states(self, key, value):
        self.__joystick_internal_states[key] = value
        # external
        self.input_states[key] = value

    @property
    def in_setting_mode(self):
        return self.__joystick_internal_states[Button.DOWN]

    def __str__(self):
        states = ""
        joystick_state_display = ["Joystick inputs:"]
        joystick_state_display += [
            f"\t{key}: {value}"
            for key, value in self.__joystick_internal_states.items()
            if not key.startswith("__DEBUG_STATE__")
        ]
        # derived states
        joystick_state_display += [f"\tIN_SETTING_MODE: {self.in_setting_mode}"]
        states += "\n".join(joystick_state_display)
        return states

    def on_x_press(self):
        self.__update_input_states(key=Button.CROSS, value=True)
        logging.debug("on_x_press")

    def on_x_release(self):
        self.__update_input_states(key=Button.CROSS, value=False)
        logging.debug("on_x_release")

    def on_triangle_press(self):
        self.__update_input_states(key=Button.TRIANGLE, value=True)
        logging.debug("on_triangle_press")

    def on_triangle_release(self):
        self.__update_input_states(key=Button.TRIANGLE, value=False)
        logging.debug("on_triangle_release")

    def on_circle_press(self):
        self.__update_input_states(key=Button.CIRCLE, value=True)
        logging.debug("on_circle_press")

    def on_circle_release(self):
        self.__update_input_states(key=Button.CIRCLE, value=False)
        logging.debug("on_circle_release")

    def on_square_press(self):
        self.__update_input_states(key=Button.SQUARE, value=True)
        logging.debug("on_square_press")

    def on_square_release(self):
        self.__update_input_states(key=Button.SQUARE, value=False)
        logging.debug("on_square_release")

    def on_L1_press(self):
        self.__update_input_states(key=Button.L1, value=True)
        logging.debug("on_L1_press")

    def on_L1_release(self):
        self.__update_input_states(key=Button.L1, value=False)
        if self.in_setting_mode:
            self.__arm_controller_obj.save_controller_states()
        else:
            if not self.controller_states[
                ControllerStates.IN_CARTESIAN_NOT_JOINT_SPACE_MODE
            ]:
                self.__arm_controller_obj.change_segment_pointer(-1)
        logging.debug("on_L1_release")

    def on_L2_press(self, value):
        # the original value is (-axis_max_value, axis_max_value)
        normalized_value = value / self.axis_max_value / 2 + 0.5
        self.__update_input_states(key=JoystickAxis.L2R2, value=-normalized_value)
        logging.debug(f"on_L2_press: {normalized_value}")

    def on_L2_release(self):
        self.__update_input_states(key=JoystickAxis.L2R2, value=None)
        logging.debug("on_L2_release")

    def on_R1_press(self):
        self.__update_input_states(key=Button.R1, value=True)
        logging.debug("on_R1_press")

    def on_R1_release(self):
        self.__update_input_states(key=Button.R1, value=False)
        if self.in_setting_mode:
            self.__arm_controller_obj.load_controller_states()
        else:
            if not self.controller_states[
                ControllerStates.IN_CARTESIAN_NOT_JOINT_SPACE_MODE
            ]:
                self.__arm_controller_obj.change_segment_pointer(1)
        logging.debug("on_R1_release")

    def on_R2_press(self, value):
        normalized_value = value / self.axis_max_value / 2 + 0.5
        self.__update_input_states(key=JoystickAxis.L2R2, value=normalized_value)
        logging.debug(f"on_R2_press: {normalized_value}")

    def on_R2_release(self):
        self.__update_input_states(key=JoystickAxis.L2R2, value=None)
        logging.debug("on_R2_release")

    def on_up_arrow_press(self):
        self.__update_input_states(key=Button.UP, value=True)
        logging.debug("on_up_arrow_press")

    def on_up_down_arrow_release(self):
        self.__update_input_states(key=Button.UP, value=False)
        self.__update_input_states(key=Button.DOWN, value=False)
        logging.debug("on_up_down_arrow_release")

    def on_down_arrow_press(self):
        self.__update_input_states(key=Button.DOWN, value=True)
        logging.debug("on_down_arrow_press")

    def on_left_arrow_press(self):
        self.__arm_controller_obj.change_velocity_level(-1)
        self.__update_input_states(key=Button.LEFT, value=True)
        logging.debug("on_left_arrow_press")

    def on_left_right_arrow_release(self):
        self.__update_input_states(key=Button.LEFT, value=False)
        self.__update_input_states(key=Button.RIGHT, value=False)
        logging.debug("on_left_right_arrow_release")

    def on_right_arrow_press(self):
        self.__arm_controller_obj.change_velocity_level(1)
        self.__update_input_states(key=Button.RIGHT, value=True)
        logging.debug("on_right_arrow_press")

    def on_L3_up(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.LEFT_VERTICAL, value=normalized_value
        )
        logging.debug(f"on_L3_up: {value}")

    def on_L3_down(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.LEFT_VERTICAL, value=normalized_value
        )
        logging.debug(f"on_L3_down: {value}")

    def on_L3_left(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.LEFT_HORIZONTAL, value=normalized_value
        )
        logging.debug(f"on_L3_left: {value}")

    def on_L3_right(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.LEFT_HORIZONTAL, value=normalized_value
        )
        logging.debug(f"on_L3_right: {value}")

    def on_L3_y_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.LEFT_VERTICAL, value=None)
        self.__arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_L3_y_at_rest")

    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.LEFT_HORIZONTAL, value=None)
        self.__arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_L3_x_at_rest")

    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        self.__update_input_states(key=Button.L3, value=True)
        logging.debug("on_L3_press")

    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        self.__update_input_states(key=Button.L3, value=False)
        if self.in_setting_mode:
            self.__arm_controller_obj.move_servos_to_calibration_position()
        else:
            if self.controller_states[ControllerStates.IN_SAVING_TRAJECTORY_MODE]:
                self.__arm_controller_obj.save_trajectory()
            else:
                self.__arm_controller_obj.start_saving_trajectory()
        logging.debug("on_L3_release")

    def on_R3_up(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.RIGHT_VERTICAL, value=normalized_value
        )
        logging.debug(f"on_R3_up: {value}")

    def on_R3_down(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.RIGHT_VERTICAL, value=normalized_value
        )
        logging.debug(f"on_R3_down: {value}")

    def on_R3_left(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.RIGHT_HORIZONTAL, value=normalized_value
        )
        logging.debug(f"on_R3_left: {value}")

    def on_R3_right(self, value):
        normalized_value = value / self.axis_max_value
        self.__update_input_states(
            key=JoystickAxis.RIGHT_HORIZONTAL, value=normalized_value
        )
        logging.debug(f"on_R3_right: {value}")

    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.RIGHT_VERTICAL, value=None)
        self.__arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_R3_y_at_rest")

    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.RIGHT_HORIZONTAL, value=None)
        self.__arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_R3_x_at_rest")

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        self.__joystick_internal_states[Button.R3] = True
        logging.debug("on_R3_press")

    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        self.__joystick_internal_states[Button.R3] = False
        if self.in_setting_mode:
            self.__arm_controller_obj.recalibrate_servos()
        else:
            self.__arm_controller_obj.replay_trajectory()
        logging.debug("on_R3_release")

    def on_options_press(self):
        logging.debug("on_options_press")

    def on_options_release(self):
        if self.arm_controller_running:
            self.__arm_controller_obj.stop_controller_thread()
        else:
            self.__arm_controller_obj.start_controller_thread()
        logging.debug("on_options_release")

    def on_share_press(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_share_press")

    def on_share_release(self):
        """this event is only detected when connecting without ds4drv"""
        if self.in_setting_mode:
            self.controller_states[
                ControllerStates.IN_CARTESIAN_NOT_JOINT_SPACE_MODE
            ] ^= True
            self.arm_controller_obj.reset_joystick_input_states()
        else:
            self.controller_states[ControllerStates.LOG_INFO_EACH_TENTHS_SECOND] ^= True
        logging.debug("on_share_release")

    def on_playstation_button_press(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_playstation_button_press")

    def on_playstation_button_release(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_playstation_button_release")
