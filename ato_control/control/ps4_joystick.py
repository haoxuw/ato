# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import threading

from control.config_and_enums.controller_enums import ControllerStates
from control.config_and_enums.joystick_input_types import Button, JoystickAxis
from control.interface_classes.input_device_interface import InputDeviceInterface
from pyPS4Controller.controller import Controller


class Ps4Joystick(Controller, InputDeviceInterface):
    def __init__(self, interface):
        self.interface = interface

        # shared by arm_servo_controller thread
        self.__joystick_internal_states = {
            # when in_trajectory_editing_mode:
            Button.CROSS: False,  # set next waypoint
            Button.SQUARE: False,  # start recording
            Button.TRIANGLE: False,  # pause 1s
            Button.CIRCLE: False,  # repeat last N waypoint
            Button.L1: False,  # N += 1
            Button.R1: False,  # N = max(0, N-1)
            Button.DOWN: False,
            Button.LEFT: False,
            Button.UP: False,
            Button.RIGHT: False,
            Button.L3: False,
            Button.R3: False,
            JoystickAxis.LEFT_HORIZONTAL: None,
            JoystickAxis.LEFT_VERTICAL: None,
            JoystickAxis.RIGHT_HORIZONTAL: None,
            JoystickAxis.RIGHT_VERTICAL: None,
            JoystickAxis.L2R2: None,
        }
        Controller.__init__(self, interface=interface, connecting_using_ds4drv=False)
        InputDeviceInterface.__init__(self)
        self.axis_max_value = 32767.0

        self.__thread = None

    @property
    def arm_controller_obj(self):
        assert self._arm_controller_obj is not None
        return self._arm_controller_obj

    @property
    def input_states(self):
        return self._arm_controller_obj.get_input_states()

    @property
    def controller_states(self):
        return self._arm_controller_obj.controller_states

    @property
    def arm_controller_running(self):
        return (
            self._arm_controller_obj is not None
            and self._arm_controller_obj.is_thread_running()
        )

    @property
    def in_cartesian_mode(self):
        return (
            self._arm_controller_obj.controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_CARTESIAN_MODE
        )

    @property
    def in_joint_space_mode(self):
        return (
            self._arm_controller_obj.controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_JOINT_SPACE_MODE
        )

    @property
    def in_setting_mode(self):
        return (
            self._arm_controller_obj.controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_SETTING_MODE
        )

    @property
    def in_trajectory_editing_mode(self):
        return (
            self._arm_controller_obj.controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_TRAJECTORY_EDITING_MODE
        )

    @property
    def recording_on(self):
        return self._arm_controller_obj.controller_states[ControllerStates.RECORDING_ON]

    def start_thread(self):
        self.__thread = threading.Thread(target=self.listen, kwargs={"timeout": 600})
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

    @property
    def __controller_mode_to_button_mapping(self):
        mapping = dict(
            zip(
                [Button.UP, Button.RIGHT, Button.DOWN, Button.LEFT],
                [
                    ControllerStates.IN_CARTESIAN_MODE,
                    ControllerStates.IN_JOINT_SPACE_MODE,
                    ControllerStates.IN_SETTING_MODE,
                    ControllerStates.IN_TRAJECTORY_EDITING_MODE,
                ],
            )
        )
        return mapping

    def __update_input_states(self, key, value):
        self.__joystick_internal_states[key] = value
        # external
        self.input_states[key] = value
        if key in self.__controller_mode_to_button_mapping:
            if value is True:
                self.arm_controller_obj.set_controller_mode(
                    self.__controller_mode_to_button_mapping[key]
                )

    def __str__(self):
        states = ""
        joystick_state_display = ["Joystick inputs:"]
        joystick_state_display += [
            f"\t{key}: {value}"
            for key, value in self.__joystick_internal_states.items()
            if not key.startswith("__DEBUG_STATE__")
        ]
        # derived states
        joystick_state_display += [f"{self.arm_controller_obj.controller_mode}"]
        states += "\n".join(joystick_state_display)
        return states

    def on_x_press(self):
        self.__update_input_states(key=Button.CROSS, value=True)
        logging.debug("on_x_press")

    def on_x_release(self):
        self.__update_input_states(key=Button.CROSS, value=False)
        if self.in_trajectory_editing_mode:
            self.arm_controller_obj.trajectory_in_editing_append_waypoint()
        logging.debug("on_x_release")

    def on_triangle_press(self):
        self.__update_input_states(key=Button.TRIANGLE, value=True)
        if self.in_trajectory_editing_mode:
            self.arm_controller_obj.trajectory_in_editing_append_pause()
        logging.debug("on_triangle_press")

    def on_triangle_release(self):
        self.__update_input_states(key=Button.TRIANGLE, value=False)
        logging.debug("on_triangle_release")

    def on_circle_press(self):
        self.__update_input_states(key=Button.CIRCLE, value=True)
        if self.in_trajectory_editing_mode:
            self._arm_controller_obj.reset_trajectory_in_editing()
        logging.debug("on_circle_press")

    def on_circle_release(self):
        self.__update_input_states(key=Button.CIRCLE, value=False)
        logging.debug("on_circle_release")

    def on_square_press(self):
        self.__update_input_states(key=Button.SQUARE, value=True)
        logging.debug("on_square_press")

    def on_square_release(self):
        self.__update_input_states(key=Button.SQUARE, value=False)
        if self.in_trajectory_editing_mode:
            if self.recording_on:
                self.arm_controller_obj.stop_recording_trajectory()
            else:
                self.arm_controller_obj.start_recording_trajectory()
        logging.debug("on_square_release")

    def on_L1_press(self):
        self.__update_input_states(key=Button.L1, value=True)
        logging.debug("on_L1_press")

    def on_L1_release(self):
        self.__update_input_states(key=Button.L1, value=False)
        if self.in_setting_mode:
            self._arm_controller_obj.change_velocity_level(-1)

            # todo: reassign
            # self._arm_controller_obj.recalibrate_servos()
            # self._arm_controller_obj.save_controller_states()
        elif self.in_cartesian_mode or self.in_joint_space_mode:
            self._arm_controller_obj.change_segment_pointer(-1)
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
            self._arm_controller_obj.change_velocity_level(1)

            # todo: reassign
            # self._arm_controller_obj.load_controller_states()
        elif self.in_cartesian_mode or self.in_joint_space_mode:
            self._arm_controller_obj.change_segment_pointer(1)
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
        self.__update_input_states(key=Button.LEFT, value=True)
        logging.debug("on_left_arrow_press")

    def on_left_right_arrow_release(self):
        self.__update_input_states(key=Button.LEFT, value=False)
        self.__update_input_states(key=Button.RIGHT, value=False)
        logging.debug("on_left_right_arrow_release")

    def on_right_arrow_press(self):
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
        self._arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_L3_y_at_rest")

    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.LEFT_HORIZONTAL, value=None)
        self._arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_L3_x_at_rest")

    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        self.__update_input_states(key=Button.L3, value=True)
        logging.debug("on_L3_press")

    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        self.__update_input_states(key=Button.L3, value=False)
        if self.in_setting_mode:
            self._arm_controller_obj.move_to_installation_position()
        elif self.in_trajectory_editing_mode:
            self._arm_controller_obj.save_trajectory()
        elif self.in_cartesian_mode or self.in_joint_space_mode:
            self._arm_controller_obj.move_to_home_positions_otherwise_zeros()
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
        self._arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_R3_y_at_rest")

    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        self.__update_input_states(key=JoystickAxis.RIGHT_HORIZONTAL, value=None)
        self._arm_controller_obj.stop_to_play_trajectory()
        logging.debug("on_R3_x_at_rest")

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        self.__joystick_internal_states[Button.R3] = True
        logging.debug("on_R3_press")

    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        self.__joystick_internal_states[Button.R3] = False
        self._arm_controller_obj.replay_trajectory()
        logging.debug("on_R3_release")

    def on_options_press(self):
        logging.debug("on_options_press")

    def on_options_release(self):
        if self.arm_controller_running:
            self._arm_controller_obj.stop_threads()
        else:
            self._arm_controller_obj.start_threads()
        logging.debug("on_options_release")

    def on_share_press(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_share_press")

    def on_share_release(self):
        """this event is only detected when connecting without ds4drv"""
        self.controller_states[ControllerStates.LOG_INFO_EACH_TENTH_SECOND] ^= True
        logging.debug("on_share_release")

    def on_playstation_button_press(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_playstation_button_press")

    def on_playstation_button_release(self):
        """this event is only detected when connecting without ds4drv"""
        logging.debug("on_playstation_button_release")
