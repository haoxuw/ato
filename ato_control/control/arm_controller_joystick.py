# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import copy

from control import arm_controller
from control.config_and_enums.arm_connection_config import ActuatorPurpose
from control.config_and_enums.joystick_input_types import Button, JoystickAxis


class ArmControllerJoystick(arm_controller.ArmController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def reset_input_states(self):
        self._input_states = {
            JoystickAxis.LEFT_HORIZONTAL: None,  # along X axis, i.e. shift towards left/right
            JoystickAxis.LEFT_VERTICAL: None,  # along Y axis, i.e. shift backward/forward
            JoystickAxis.RIGHT_HORIZONTAL: None,  # intrinsic yaw
            JoystickAxis.RIGHT_VERTICAL: None,  # intrinsic pitch
            JoystickAxis.L2R2: None,  # gripper close/open
            Button.CROSS: False,  # along Z axis, i.e. descend
            Button.SQUARE: False,  # rotate counterclockwise
            Button.TRIANGLE: False,  # along Z axis, i.e. ascend
            Button.CIRCLE: False,  # rotate clockwise
            Button.DOWN: False,  # settings mode (while holding)
            Button.LEFT: False,
            Button.UP: False,
            Button.RIGHT: False,
            Button.L1: False,  # backwards along endeffector orientation
            Button.R1: False,  # forwards along endeffector orientation
            Button.L3: False,  # start recording trajectory, save trajectory
            Button.R3: False,  # replay trajectory
        }

    def _parse_movement_updates_in_cartesian_space(self, time_delta_ms):
        # cartesian_movements = self.__parse_movements_xyz() + self.__parse_movements_rpy()
        # actuator_positions_delta = motion.EndeffectorPose(cartesian_movements).inverse_kinematics()

        # mapping defined according to reset_input_states()
        mapping = {
            0: JoystickAxis.LEFT_HORIZONTAL,  # x
            1: JoystickAxis.LEFT_VERTICAL,  # y
            2: (Button.CROSS, Button.TRIANGLE),  # z
            # gripper reference frame is intrinsic, and its initial frame is different from camera
            3: JoystickAxis.RIGHT_VERTICAL,  # pitch
            4: JoystickAxis.RIGHT_HORIZONTAL,  # yaw
            5: (Button.SQUARE, Button.CIRCLE),  # roll
            6: JoystickAxis.L2R2,  # gripper
        }
        to_move = False
        cartesian_delta = [0] * 7
        for index, designated_axis in mapping.items():
            if isinstance(designated_axis, tuple):
                offset_ratio = 0
                if self._input_states[designated_axis[0]]:
                    offset_ratio -= 1
                if self._input_states[designated_axis[1]]:
                    offset_ratio += 1
                # e.g. CROSS+TRIANGLE would cancel out
            else:
                offset_ratio = self._input_states[designated_axis] or 0
            if offset_ratio != 0:
                to_move = True
                cartesian_delta[index] = offset_ratio
                cartesian_delta[index] *= time_delta_ms
                cartesian_delta[index] *= self.cartesian_velocity
        if to_move:
            return copy.deepcopy(self._intended_pose).apply_delta(
                pose_delta=cartesian_delta[:6], gripper_delta=cartesian_delta[-1]
            )
        else:
            return None

    def __locate_designated_axis(self, segment_id: int, enabler_axis: ActuatorPurpose):
        if enabler_axis == ActuatorPurpose.GRIPPER:
            # L2R2 is a special case
            return JoystickAxis.qL2R2
        else:
            # if not L2 or R2, then it could be controlled by the two joysticks
            if segment_id not in self.activated_segment_ids:
                # the segment is not actively controlled
                return None

            if segment_id == self.activated_segment_ids[0]:
                mapping = {
                    ActuatorPurpose.ROLL: JoystickAxis.LEFT_HORIZONTAL,
                    ActuatorPurpose.PITCH: JoystickAxis.LEFT_VERTICAL,
                }
            else:  # segment_id == self.activated_segment_ids[1]:
                mapping = {
                    ActuatorPurpose.ROLL: JoystickAxis.RIGHT_HORIZONTAL,
                    ActuatorPurpose.PITCH: JoystickAxis.RIGHT_VERTICAL,
                }
            return mapping[enabler_axis]

    def _parse_movement_updates_in_joint_space(self, time_delta_ms, show_info):
        joint_positions_delta = [0] * len(self._indexed_servo_names)
        # each joint is controlled by some designated_axis on the joystick
        for index, unique_name in enumerate(self._indexed_servo_names):
            segment_id, enabler_axis = self.deserialize_unique_name(
                unique_name=unique_name
            )
            designated_axis = self.__locate_designated_axis(
                segment_id=segment_id, enabler_axis=enabler_axis
            )
            if designated_axis is not None:
                stick_offset_ratio = self._input_states[designated_axis]
                if stick_offset_ratio is None:
                    continue  # no movements

                position_delta = stick_offset_ratio * time_delta_ms
                position_delta *= (
                    self._indexed_servo_configs[index].velocity_magnifier
                    * self.actuator_velocity
                )
                joint_positions_delta[index] = position_delta
        return self.__move_servos_by_joint_space_delta(
            joint_positions_delta=joint_positions_delta, show_info=show_info
        )
