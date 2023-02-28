# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import copy
import json
import logging
import numbers
import os
import pathlib
import threading
import time
from datetime import datetime
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
from control import motion, ps4_joystick, raspberry_pi
from control.config_and_enums.arm_connection_config import (
    ActuatorPurpose,
    SegmentConfigTypes,
    ServoConnectionConfig,
)
from control.config_and_enums.joystick_input_types import (
    Button,
    ControllerStates,
    JoystickAxis,
)
from control.interface_classes import arm_controller_interface, servo_interface
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation


class ArmController(arm_controller_interface.ArmControllerInterface):
    def __init__(
        self,
        pi_obj: raspberry_pi.RaspberryPi,
        joystick_obj: ps4_joystick.Ps4Joystick,
        arm_segments_config: Dict[Tuple[int, str], ServoConnectionConfig],
        frame_rate=100,
        frame_rate_for_solving_cartesian=10,
        auto_save_controller_states_to_file=True,
        actuator_velocities_deg_per_ms=tuple(
            i * 0.01 for i in range(1, 10)
        ),  # angular velocity
        cartesian_velocities_mm_or_deg_per_ms=tuple(i * 0.1 for i in range(1, 10)),
        initial_velocity_level=2,
        home_position=(0, 40, 0, 60, 0, -35, 0),  # (0, 60, 0, 80, 0, -50, 0),
    ):
        self.home_position = home_position
        assert isinstance(pi_obj, raspberry_pi.RaspberryPi)
        self.__pi_obj = pi_obj

        assert isinstance(joystick_obj, ps4_joystick.Ps4Joystick)
        self.__joystick_obj = joystick_obj
        self.__joystick_obj.connect_arm_controller(self)

        self._indexed_servo_names: Tuple[str]
        self._indexed_servo_objs: Tuple[servo_interface.ServoInterface]
        self._indexed_servo_configs: Tuple[ServoConnectionConfig]
        self._indexed_segment_lengths: Tuple[float]
        self.__gripper_length: float

        (
            self._indexed_servo_names,
            self._indexed_servo_objs,
            self._indexed_servo_configs,
            self._indexed_segment_lengths,
            self.__gripper_length,
            self.__actuator_indices_mapping,
            self.__gripper_index_mapping,
            self.num_segments,
        ) = ArmController.__register_servo_objs(
            arm_segments_config=arm_segments_config, pi_obj=self.pi_obj
        )
        self.__inverse_indexed_servo_names = {
            name: index for index, name in enumerate(self._indexed_servo_names)
        }

        motion.ActuatorPositions.set_arm_segment_lengths(
            arm_segment_lengths=self._indexed_segment_lengths
        )
        motion.ActuatorPositions.set_actuator_indices_mapping(
            actuator_indices_mapping=self.__actuator_indices_mapping
        )
        motion.ActuatorPositions.set_gripper_length(
            gripper_length=self.__gripper_length
        )
        motion.ActuatorPositions.set_gripper_index_mapping(
            gripper_index_mapping=self.__gripper_index_mapping
        )
        motion.ActuatorPositions.set_rotation_ranges(
            rotation_ranges=self._get_indexed_rotation_ranges()
        )

        motion.EndeffectorPose.set_ikpy_robot_chain(urdf_filename="ato_3_seg.urdf")

        self.frame_rate = frame_rate
        self.interval_ms = 1000 // frame_rate

        self.__solve_cartesian_once_in_x_cycles = max(
            frame_rate // frame_rate_for_solving_cartesian, 1
        )
        self.__auto_save_controller_states_to_file = auto_save_controller_states_to_file

        self.__controller_thread = None
        self.__to_stop_clock = False
        self.__segment_id_pointer = 0

        # velocity settings
        self.__actuator_velocities_deg_per_ms = actuator_velocities_deg_per_ms
        self.__cartesian_velocities_mm_or_deg_per_ms = (
            cartesian_velocities_mm_or_deg_per_ms
        )
        self.__velocity_level = initial_velocity_level

        # trajectory replay (joint space)
        self.__trajectory_saved: motion.TrajectoryActuatorPositions = None
        self.__trajectory_to_play: dict = None

        # joint space control: tracks current position of actuators
        self.__current_actuator_positions = None
        self.__current_actuator_position_forward_kinematics = None

        self.__moving_towards_positions_delta = None

        # cartesian space control, tracks:
        # where is the intended pose
        self.__intended_pose: motion.EndeffectorPose = None
        # where is the (realistic) attempted pose (best effort towards the intended pose, because not all pose are reachable)
        # self.__attempted_pose: motion.EndeffectorPose = None
        # where is the attempted joint positions == IK(attempted pose)
        # self.__attempted_joint_positions: motion.ActuatorPositions = None
        # where is the current joint positions, max velocity may not keep up achieving the attempted
        # is self.__current_actuator_positions

        self.reset_joystick_input_states()
        self.reset_controller_flags()
        self.__reset_trajectory()

        self.load_controller_states()
        self.__update_intended_pose_to_current_pose()

        self.__controller_start_time = datetime.now()
        self.__clock_counter = 0
        self.ready = True

    @property
    def is_at_home_position(self):
        return np.allclose(
            self.home_position,
            self.__get_indexed_actuator_positions(),
            rtol=0.001,
        )  # 0.1% tolerance

    def get_joystick_input_states(self):
        return self.__joystick_input_states

    def reset_joystick_input_states(self):
        self.__joystick_input_states = {
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

    def get_controller_states(self):
        return self.__controller_states

    def reset_controller_flags(self):
        self.__controller_states = {
            ControllerStates.LOG_INFO_EACH_TENTHS_SECOND: False,
            ControllerStates.IN_SAVING_TRAJECTORY_MODE: False,
            ControllerStates.IN_CARTESIAN_NOT_JOINT_SPACE_MODE: True,
        }

    def is_thread_running(self):
        if self.__controller_thread is None:
            return False
        return self.__controller_thread.is_alive()

    def save_controller_states(
        self, folder="~/.ato/", filename="ato_controller_states.json", dry_run=False
    ):
        try:
            arm_config = {
                "servo_names": (self._indexed_servo_names),
                "calibration_deltas": tuple(
                    servo_obj.calibration_delta
                    for servo_obj in self._indexed_servo_objs
                ),
                "payload_positions": tuple(
                    servo_obj.payload_position for servo_obj in self._indexed_servo_objs
                ),
                "saved_trajectory": self.__trajectory_saved.to_serializable()
                if self.__trajectory_saved is not None
                else None,
            }
            folder = os.path.expanduser(folder)

            pathlib.Path(folder).mkdir(parents=True, exist_ok=True)
            file_path = os.path.join(folder, filename)

            if not dry_run:
                with open(file_path, "w", encoding="utf-8") as fout:
                    json.dump(
                        arm_config,
                        fout,
                        indent=4,
                        sort_keys=True,
                        default=lambda var: var.item()
                        if isinstance(var, np.generic)
                        else var,
                    )
                    logging.debug(f"Saved to {file_path}")
                    fout.close()
        except Exception as e:
            logging.warning(f"Failed to save servos position, exception: {e}")

    def load_controller_states(
        self, folder="~/.ato/", filename="ato_controller_states.json"
    ):
        try:
            folder = os.path.expanduser(folder)

            file_path = os.path.join(folder, filename)
            if not os.path.isfile(file_path):
                return
            logging.info(f"Loading from {file_path}")
            with open(file_path, "r", encoding="utf-8") as fin:
                arm_config = json.load(fin)

                # servo_names
                self._indexed_servo_names = arm_config["servo_names"]

                # calibration_deltas
                for unique_name, position in zip(
                    self._indexed_servo_names, arm_config["calibration_deltas"]
                ):
                    self._indexed_servo_objs[
                        self.__get_index_by_servo_name(unique_name)
                    ].set_calibration_delta(calibration_delta=position)

                # payload_positions
                actuator_positions = motion.ActuatorPositions(
                    positions=arm_config["payload_positions"]
                )
                self.__move_servos_by_joint_space(
                    joint_positions=actuator_positions.positions
                )
                self.__do_internal_updates_for_current_actuator_positions()

                # saved_trajectory
                if arm_config["saved_trajectory"] is not None:
                    self.__trajectory_saved = motion.TrajectoryActuatorPositions()
                    for timestamp, positions in arm_config["saved_trajectory"]:
                        self.__trajectory_saved.append(
                            timestamp=timestamp,
                            positions=motion.ActuatorPositions(positions=positions),
                        )
        except Exception as e:
            logging.warning(f"Failed to load servos position, exception: {e}")

    def recalibrate_servos(self):
        for unique_name, servo_obj in zip(
            self._indexed_servo_names, self._indexed_servo_objs
        ):
            servo_obj.recalibrate()
            logging.info(
                f"Calibrated servo {unique_name} @{servo_obj.payload_position} - @{servo_obj.calibration_delta}"
            )

    def replay_trajectory(self):
        if self.__trajectory_saved is None:
            return
        logging.debug(f"Replaying: {self.__trajectory_saved}")
        trajectory = self.prepend_reposition_to_trajectory(self.__trajectory_saved)
        self.queue_up_trajectory(trajectory)

    def change_segment_pointer(self, delta):
        self.__segment_id_pointer += delta
        last_seg_id = max(
            *[
                self.deserialize_unique_name(unique_name=unique_name)[0]
                for unique_name in self._indexed_servo_names
            ]
        )
        self.__segment_id_pointer = min(last_seg_id, self.__segment_id_pointer)
        self.__segment_id_pointer = max(0, self.__segment_id_pointer)

    def change_velocity_level(self, delta):
        self.__velocity_level += delta
        self.__velocity_level = max(0, self.__velocity_level)
        self.__velocity_level = min(
            len(self.__actuator_velocities_deg_per_ms) - 1, self.__velocity_level
        )

    def __solve_valid_movements_from_cartesian_delta(
        self, cartesian_delta: Tuple[float]
    ):
        if cartesian_delta is None:
            return (None, None)
        target_pose = copy.deepcopy(self.__intended_pose).apply_delta(
            pose_delta=cartesian_delta[:6], gripper_delta=cartesian_delta[-1]
        )

        actuator_positions = self.__get_indexed_actuator_positions()
        initial_joint_positions = actuator_positions[:-1]  # remove gripper

        for orientation_mode in ("Z", "none"):  # todo try "all" axes
            target_positions = target_pose.inverse_kinematics_ikpy(
                initial_joint_positions=initial_joint_positions,
                orientation_mode=orientation_mode,
            )
            if target_positions is None:
                continue
            (
                validation_results,
                intended_pose_vector,
                actual_pose_vector,
            ) = target_pose.validate_ik_fk(
                new_joint_positions_sent_to_hardware=target_positions
            )
            if validation_results is True:
                logging.debug(
                    f"validate_ik_fk @{orientation_mode}: {intended_pose_vector} == {actual_pose_vector}"
                )
                positions_delta = target_positions - np.array(
                    self.__get_indexed_actuator_positions()
                )
                joint_velocity_limits = (
                    np.array(
                        [
                            config.velocity_magnifier
                            for config in self._indexed_servo_configs
                        ]
                    )
                    * self.actuator_velocity
                )
                positions_over_limit_ratio = np.max(
                    np.abs(positions_delta / joint_velocity_limits)
                )
                if positions_over_limit_ratio > 1:
                    positions_delta /= positions_over_limit_ratio / 2
                return (positions_delta, actual_pose_vector)
            else:
                logging.debug(
                    f"Failed to validate_ik_fk @{orientation_mode}: {intended_pose_vector} != {actual_pose_vector}"
                )
                # continue
        return (None, None)

    def move_to_installation_position(self):
        for servo in self._indexed_servo_objs:
            servo.move_to_installation_position()

    # if already at home position, move all actuators to logical zero (installation position)
    def move_to_home_position_otherwise_zeros(self):
        num_servos = 7
        assert (
            len(self._indexed_servo_names) == num_servos
        ), f"Currently this function only support for {(num_servos-1)/2} segment arm."

        if self.is_at_home_position:
            actuator_positions = motion.ActuatorPositions(positions=[0] * num_servos)
            self.move_to_actuator_positions(actuator_positions=actuator_positions)
        else:
            actuator_positions = motion.ActuatorPositions(positions=self.home_position)
            self.move_to_actuator_positions(actuator_positions=actuator_positions)

    def __reset_trajectory(self):
        self.__active_trajectory: motion.TrajectoryActuatorPositions = (
            motion.TrajectoryActuatorPositions()
        )
        self.__active_trajectory_start_time = datetime.now()

    def start_saving_trajectory(self):
        self.__controller_states[ControllerStates.IN_SAVING_TRAJECTORY_MODE] = True
        self.__reset_trajectory()

    def save_trajectory(self):
        self.__controller_states[ControllerStates.IN_SAVING_TRAJECTORY_MODE] = False
        self.__trajectory_saved = copy.deepcopy(self.__active_trajectory)
        logging.info(f"Saving: {self.__trajectory_saved}")

    def start_threads(self, start_joystick_thread=True):
        self.start_arm_controller_thread()
        if start_joystick_thread:
            self.joystick_obj.start_thread()

    def stop_threads(self):
        self.stop_arm_controller_thread()
        self.joystick_obj.stop_thread()

    def start_arm_controller_thread(self):
        self.__controller_thread = threading.Thread(
            target=self.arm_controller_thread_loop,
            args=(self.interval_ms,),
        )
        self.__controller_thread.start()
        logging.info(f"Thread for Arm controller started.")

    def stop_arm_controller_thread(self):
        if self.__controller_thread is not None:
            self.__to_stop_clock = True
            self.__controller_thread.join()
            self.__controller_thread = None
        else:
            logging.warning(f"Thread for Arm Controller ALREADY stopped.")

    @staticmethod
    # @functools.cache
    def serialize_to_unique_name(segment_id, enabler_axis):
        return f"{segment_id}_{enabler_axis}"

    @staticmethod
    # @functools.cache
    def deserialize_unique_name(unique_name):
        segment_id, enabler_axis = unique_name.split("_")
        return int(segment_id), enabler_axis

    @staticmethod
    def __register_servo_objs(
        arm_segments_config: Dict[Tuple[int, str], ServoConnectionConfig],
        pi_obj: raspberry_pi.RaspberryPi,
    ) -> Tuple[
        Dict[str, servo_interface.ServoInterface], Dict[str, ServoConnectionConfig]
    ]:
        # part 1: parse arm_segments_config
        # populate servo_objs and servo_configs
        indexed_servo_names = []
        indexed_servo_objs = []
        indexed_servo_configs = []
        indexed_segment_lengths = []

        num_segments = max(arm_segments_config.keys()) + 1
        for segment_id in range(num_segments):
            if segment_id < 0:
                # gripper
                continue
            assert (
                segment_id in arm_segments_config
            ), f"{segment_id} in {arm_segments_config}"
            segment_config = arm_segments_config[segment_id]
            assert (
                SegmentConfigTypes.PHYSICAL_LENGTH in segment_config
            ), f"{SegmentConfigTypes.PHYSICAL_LENGTH} in {segment_config}"
            indexed_segment_lengths.append(
                segment_config[SegmentConfigTypes.PHYSICAL_LENGTH]
            )
            for enabler_axis in [ActuatorPurpose.ROLL, ActuatorPurpose.PITCH]:
                assert (
                    enabler_axis in segment_config
                ), f"{enabler_axis} in {segment_config}"

                servo_config: ServoConnectionConfig = segment_config[enabler_axis]
                unique_name = ArmController.serialize_to_unique_name(
                    segment_id=segment_id, enabler_axis=enabler_axis
                )
                servo_obj = servo_config.servo_class(
                    unique_name=unique_name,
                    pi_obj=pi_obj,
                    installation_angle=servo_config.installation_angle,
                    header_id=int(servo_config.header_id),
                    rotation_range=servo_config.rotation_range,
                    servo_max_position=servo_config.servo_max_position,
                )
                indexed_servo_names.append(unique_name)
                indexed_servo_objs.append(servo_obj)
                indexed_servo_configs.append(servo_config)
                logging.info(f"Added servo controller {unique_name}")

        actuator_indices_mapping = tuple(
            ((index * 2, index * 2 + 1) for index in range(num_segments))
        )

        # gripper
        unique_name = ArmController.serialize_to_unique_name(
            segment_id=-1, enabler_axis=ActuatorPurpose.GRIPPER
        )
        servo_config = arm_segments_config[-1][ActuatorPurpose.GRIPPER]
        gripper_length = arm_segments_config[-1][SegmentConfigTypes.PHYSICAL_LENGTH]
        servo_obj = servo_config.servo_class(
            unique_name=unique_name,
            pi_obj=pi_obj,
            header_id=int(servo_config.header_id),
            installation_angle=servo_config.installation_angle,
            rotation_range=servo_config.rotation_range,
            servo_max_position=servo_config.servo_max_position,
        )

        indexed_servo_names.append(unique_name)
        indexed_servo_objs.append(servo_obj)
        indexed_servo_configs.append(servo_config)
        gripper_index_mapping = len(indexed_servo_names)

        return (
            tuple(indexed_servo_names),
            tuple(indexed_servo_objs),
            tuple(indexed_servo_configs),
            tuple(indexed_segment_lengths),
            gripper_length,
            actuator_indices_mapping,
            gripper_index_mapping,
            num_segments,
        )

    def prepend_reposition_to_trajectory(
        self,
        target_trajectory: motion.TrajectoryActuatorPositions,
        pause_sec: float = 1,
    ):
        point = target_trajectory.trajectory[0]
        destination: motion.ActuatorPositions = point[1]
        current: motion.ActuatorPositions = self.__get_current_actuator_position_obj()
        velocities = np.array(
            [
                self.actuator_velocity
                * self._indexed_servo_configs[
                    self.__get_index_by_servo_name(unique_name)
                ].velocity_magnifier
                for unique_name in self._indexed_servo_names
            ]
        )
        offsets = np.array(destination.positions) - np.array(current.positions)
        ms_per_s = 1000
        duration = np.abs(offsets / velocities / ms_per_s)
        max_duration = np.amax(duration)
        trajectory = motion.TrajectoryActuatorPositions()
        trajectory.append(timestamp=0, positions=current)
        trajectory.append(timestamp=max_duration, positions=destination)
        if pause_sec > 0:
            trajectory.append(timestamp=max_duration + pause_sec, positions=destination)
        else:
            pause_sec = 0
        for timestamp, actuator_positions in target_trajectory.trajectory[1:]:
            trajectory.append(
                timestamp=max_duration + pause_sec + timestamp,
                positions=actuator_positions,
            )
        return trajectory

    def queue_up_trajectory(self, trajectory: motion.Trajectory):
        if isinstance(trajectory, motion.TrajectoryEndeffectorPose):
            trajectory = trajectory.inverse_kinematics()
        # __trajectory_to_play is checked each cycle
        self.__trajectory_to_play = {
            "start_time": datetime.now(),
            "trajectory": copy.deepcopy(trajectory),
        }

    def move_to_actuator_positions(
        self, actuator_positions: motion.ActuatorPositions, pause_sec: float = 1
    ):
        trajectory = motion.TrajectoryActuatorPositions()
        trajectory.append(timestamp=0, positions=actuator_positions)
        trajectory = self.prepend_reposition_to_trajectory(
            target_trajectory=trajectory, pause_sec=pause_sec
        )
        self.queue_up_trajectory(trajectory)

    def is_playing_trajectory(self, current_time=None):
        if current_time is None:
            current_time = datetime.now()
        if self.__trajectory_to_play is not None:  # replay mode active
            since_replay_start_delta = (
                current_time - self.__trajectory_to_play["start_time"]
            )
            trajectory: motion.Trajectory = self.__trajectory_to_play["trajectory"]
            if (
                trajectory is None
                or since_replay_start_delta.total_seconds() >= trajectory.duration
            ):
                self.stop_to_play_trajectory(completed=True)

        return self.__trajectory_to_play is not None

    def stop_to_play_trajectory(self, completed=False):
        if self.__trajectory_to_play is not None:
            self.__trajectory_to_play = None
            if completed:
                logging.info(f"Completed playing trajectory")
            else:
                logging.info(f"Stopped to play trajectory")

    def __update_intended_pose_to_current_pose(self):
        self.__intended_pose = motion.EndeffectorPose(
            pose=self.__get_endeffector_pose()
        )

    def __advance_clock(self, current_time, time_delta_ms):
        self.__clock_counter += 1
        show_info = not self.__controller_states[
            ControllerStates.LOG_INFO_EACH_TENTHS_SECOND
        ]
        if self.is_playing_trajectory(current_time=current_time):
            trajectory: motion.Trajectory = self.__trajectory_to_play["trajectory"]
            since_replay_start_delta = (
                current_time - self.__trajectory_to_play["start_time"]
            )
            positions = trajectory.get(since_replay_start_delta.total_seconds())
            for index, unique_name in enumerate(self._indexed_servo_names):

                position = positions[index]
                self.__move_servo(
                    unique_name=unique_name, position=position, show_info=show_info
                )
            self.__update_intended_pose_to_current_pose()
        elif self.__controller_states[
            ControllerStates.IN_CARTESIAN_NOT_JOINT_SPACE_MODE
        ]:
            if self.__clock_counter % self.__solve_cartesian_once_in_x_cycles == 0:
                cartesian_delta = self.__parse_movement_updates_in_cartesian_space(
                    time_delta_ms=time_delta_ms,
                )
                (
                    positions_delta,
                    actual_pose_vector,
                ) = self.__solve_valid_movements_from_cartesian_delta(
                    cartesian_delta=cartesian_delta
                )
                self.__moving_towards_positions_delta = positions_delta

            if (
                self.__moving_towards_positions_delta is not None
            ):  # may inherit value from pervious cycles
                if (
                    self.__move_servos_by_joint_space_delta(
                        joint_positions_delta=self.__moving_towards_positions_delta
                    )
                    is not None
                ):
                    self.__update_intended_pose_to_current_pose()
            else:
                # no movement, don't update intended pose
                pass
        else:
            self.__parse_movement_updates_in_joint_space(
                time_delta_ms=time_delta_ms,
                show_info=show_info,
            )
            self.__update_intended_pose_to_current_pose()

    def __parse_movement_updates_in_cartesian_space(self, time_delta_ms):
        # cartesian_movements = self.__parse_movements_xyz() + self.__parse_movements_rpy()
        # actuator_positions_delta = motion.EndeffectorPose(cartesian_movements).inverse_kinematics()

        # mapping defined according to reset_joystick_input_states()
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
                if self.__joystick_input_states[designated_axis[0]]:
                    offset_ratio -= 1
                if self.__joystick_input_states[designated_axis[1]]:
                    offset_ratio += 1
                # e.g. CROSS+TRIANGLE would cancel out
            else:
                offset_ratio = self.__joystick_input_states[designated_axis] or 0
            if offset_ratio != 0:
                to_move = True
                cartesian_delta[index] = offset_ratio
                cartesian_delta[index] *= time_delta_ms
                cartesian_delta[index] *= self.cartesian_velocity
        if to_move:
            return cartesian_delta
        else:
            return None

    def __parse_movement_updates_in_joint_space(self, time_delta_ms, show_info):
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
                stick_offset_ratio = self.__joystick_input_states[designated_axis]
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

    def __move_servos_by_joint_space_delta(
        self, joint_positions_delta, show_info=False
    ):
        if joint_positions_delta is None:
            return None
        return [
            self.__move_servo_by_delta(
                unique_name=unique_name,
                position_delta=position_delta,
                show_info=show_info,
            )
            for unique_name, position_delta in zip(
                self._indexed_servo_names, joint_positions_delta
            )
        ]

    def __move_servos_by_joint_space(self, joint_positions, show_info=False):
        return [
            self.__move_servo(
                unique_name=unique_name,
                position=position,
                show_info=show_info,
            )
            for unique_name, position in zip(self._indexed_servo_names, joint_positions)
        ]

    def __locate_designated_axis(self, segment_id: int, enabler_axis: ActuatorPurpose):
        if enabler_axis == ActuatorPurpose.GRIPPER:
            # L2R2 is a special case
            return JoystickAxis.L2R2
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

    def __move_servo_by_delta(
        self, unique_name: str, position_delta: float, show_info: bool
    ):
        assert isinstance(position_delta, numbers.Number), position_delta
        # the difference between pos to position is similar to velocity to speed, or offset to distance
        # position of the joint is described as a vector
        # while position of the servo is a scalar value
        new_position = self._indexed_servo_objs[
            self.__get_index_by_servo_name(unique_name)
        ].move_to_position_delta(position_delta=position_delta, show_info=show_info)
        return new_position

    def __move_servo(self, unique_name: str, position: float, show_info: bool):
        assert isinstance(position, numbers.Number), position
        position = self._indexed_servo_objs[
            self.__get_index_by_servo_name(unique_name)
        ].move_to_position(position=position, show_info=show_info)
        return position

    def describe_current_actuator_positions(self):
        actuator_positions_dict = {}
        for unique_name, servo_obj in zip(
            self._indexed_servo_names, self._indexed_servo_objs
        ):
            actuator_positions_dict[unique_name] = servo_obj.payload_position
        return actuator_positions_dict

    def __do_internal_updates_for_current_actuator_positions(self):
        # calculate forward_kinematics and cache
        self.__get_current_actuator_position_obj(refresh=True)
        self.__get_current_actuator_position_forward_kinematics(refresh=True)

        # append to_trajectory
        delta = datetime.now() - self.__active_trajectory_start_time
        self.__active_trajectory.append(
            timestamp=delta.total_seconds(),
            positions=self.__get_current_actuator_position_obj(),
        )

    def arm_controller_thread_loop(
        self,
        interval_ms: float,
        print_initial_console_log=True,
    ):
        if print_initial_console_log:
            self.__refresh_states_display()
        last = datetime.now()
        while not self.__to_stop_clock:
            if not self.is_thread_running():
                break
            current = datetime.now()

            time_delta_tenths_sec = int(
                (last - self.__controller_start_time).total_seconds() * 10
            ) - int((current - self.__controller_start_time).total_seconds() * 10)
            time_delta_ms = (current - last).total_seconds() * 1000

            if (
                self.__controller_states[ControllerStates.LOG_INFO_EACH_TENTHS_SECOND]
                and time_delta_tenths_sec < 0
            ):
                self.__refresh_states_display()

            # this call could take time, so simply sleep(self.interval) wouldn't be precise
            if time_delta_ms >= interval_ms:
                self.__advance_clock(
                    current_time=current,
                    time_delta_ms=time_delta_ms,
                )
                self.__do_internal_updates_for_current_actuator_positions()
                self.save_controller_states(
                    dry_run=not self.__auto_save_controller_states_to_file
                )
                last = current
            time.sleep(interval_ms / 1000)
        self.__to_stop_clock = False
        logging.info(f"Thread for Arm Controller stopped.")

    def __refresh_states_display(self):
        states = "\n".join([str(self.joystick_obj), str(self)])
        logging.info(states)

    @property
    def pi_obj(self):
        assert self.__pi_obj is not None
        return self.__pi_obj

    @property
    def joystick_obj(self):
        assert self.__joystick_obj is not None
        return self.__joystick_obj

    @property
    # degree per ms
    def actuator_velocity(self):
        return self.__actuator_velocities_deg_per_ms[self.__velocity_level]

    @property
    # mm per ms
    def cartesian_velocity(self):
        return self.__cartesian_velocities_mm_or_deg_per_ms[self.__velocity_level]

    @property
    def seg_ids(self):
        return tuple(
            self.deserialize_unique_name(unique_name=unique_name)[0]
            for unique_name in self._indexed_servo_names
        )

    @property
    def activated_segment_ids(self):
        left_seg_id = self.__segment_id_pointer

        right_seg_id = (
            left_seg_id + 1 if left_seg_id + 1 <= max(*self.seg_ids) else None
        )
        # returns (0, 1), (1, 2), etc.
        return (left_seg_id, right_seg_id)

    def visualize_joints(self):
        fig = plt.figure()
        axes_limits = max(self._indexed_segment_lengths) * 1.5
        ax = plt.axes(projection="3d")
        # camera frame and robot frame is different, for robot:
        # axis are determined by left hand rule, X = thumb, Y = middle, Z = ring
        # assume the first segment of arm is installed upwards, so the X axis (roll) would be camera's Z
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(-axes_limits, axes_limits)
        ax.set_ylim(axes_limits, -axes_limits)
        ax.set_zlim(-axes_limits / 10, axes_limits * 2)

        plotted_segments = ax.plot([], [], [], marker="o", color="orange")

        def draw_3_axes(
            xyz,
            uvw,
            length,
            axes_mapping=(0, 1, 2),
            color=("red", "green", "blue"),
            is_degrees=True,
        ):
            if is_degrees:
                uvw = uvw / 180 * np.pi
            # there's a decade old known issue of quiver: arrow head and body may have different color
            # the fix is quite an overkill
            return ax.quiver(
                *[xyz[:, index] for index in axes_mapping],
                *[uvw[:, index] for index in axes_mapping],
                length=length,
                color=color,
                pivot="tail",
                normalize=True,
            )

        # we need to remove plotted segments and coordinate system from last plot
        # so we use this dict to serve as update.plotted_elements
        plotted_elements = {}

        def update(iteration, plotted_elements):
            kinematics = self.__get_current_actuator_position_forward_kinematics()
            plotted_segments[0].set_data_3d(
                kinematics["segments_xyz"][:, 0],
                kinematics["segments_xyz"][:, 1],
                kinematics["segments_xyz"][:, 2],
            )
            for plotted_element in plotted_elements.values():
                plotted_element.remove()
            for index, segments_reference_frame in enumerate(
                kinematics["segments_reference_frames"][
                    :, :-1
                ]  # [:, :-1] to remove z axis, which is overlapped with the next segment
            ):
                plotted_elements[f"segments_reference_frame{index}"] = draw_3_axes(
                    xyz=kinematics["segments_xyz"][index : index + 1],
                    uvw=segments_reference_frame,
                    length=self._indexed_segment_lengths[-1] / 2,
                )
            plotted_elements["endeffector_reference_frame"] = draw_3_axes(
                xyz=kinematics["segments_xyz"][-1:],
                uvw=kinematics["endeffector_reference_frame"][:],
                length=self.__gripper_length,
            )
            if self.__intended_pose is not None:
                global_frame = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                rpy = self.__intended_pose.rpy
                uvw = np.array(
                    [
                        Rotation.from_euler(
                            "XYZ",
                            rpy,
                            degrees=True,
                        ).apply(axis)
                        for axis in global_frame
                    ]
                )
                plotted_elements["intended_pose"] = draw_3_axes(
                    xyz=np.array([self.__intended_pose.xyz]),
                    uvw=np.array([uvw]),
                    length=self.__gripper_length,
                    color=["orange", "cyan", "purple"],
                )
            return iteration

        anim = FuncAnimation(
            fig, update, interval=5, fargs=(plotted_elements,), cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()
        return anim

    def __str__(self) -> str:
        if not self.ready:
            return ""
        time_str = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        actuator_states = "\n".join(
            [f"{str(servo_obj)}" for servo_obj in self._indexed_servo_objs]
        )
        math_based_current_pose = self.__get_endeffector_pose()
        # ml_fk_pose = (
        #     self.__get_current_actuator_position_obj().forward_kinematics_ml_based()
        # )
        return f"{time_str} @activated_segment_ids={self.activated_segment_ids} $velocity deg/ms={self.actuator_velocity} {self.frame_rate}hz\n{actuator_states}\nMath-based EE estimate={math_based_current_pose}\n"

    # the Sequence_Index would match self._indexed_ variables
    def __get_indexed_actuator_positions(self):
        return tuple((obj.payload_position for obj in self._indexed_servo_objs))

    def _get_indexed_rotation_ranges(self):
        return np.array(
            [config.rotation_range for config in self._indexed_servo_configs]
        )

    def __get_index_by_servo_name(self, name):
        return self.__inverse_indexed_servo_names[name]

    def __get_current_actuator_position_obj(
        self, refresh=False
    ) -> motion.ActuatorPositions:
        if refresh or self.__current_actuator_positions is None:
            positions = self.__get_indexed_actuator_positions()
            assert isinstance(positions[0], numbers.Number), type(positions[0])
            self.__current_actuator_positions = motion.ActuatorPositions(
                positions=positions
            )
        return self.__current_actuator_positions

    def __get_current_actuator_position_forward_kinematics(self, refresh=False):
        if motion.ActuatorPositions.is_ready() and (
            refresh or self.__current_actuator_position_forward_kinematics is None
        ):
            self.__current_actuator_position_forward_kinematics = (
                self.__get_current_actuator_position_obj().forward_kinematics_math_based()
            )
        return self.__current_actuator_position_forward_kinematics

    def __get_endeffector_pose(self):
        return self.__get_current_actuator_position_forward_kinematics()[
            "endeffector_pose_intrinsic"
        ]