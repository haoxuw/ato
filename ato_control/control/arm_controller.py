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
import math
import numbers
import os
import pathlib
import threading
import time
from datetime import datetime
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pinocchio
import pybullet
from control import arm_position, arm_trajectory, ps4_joystick, raspberry_pi
from control.config_and_enums.arm_connection_config import (
    ActuatorPurpose,
    ArmConnectionAttributes,
    ServoConnectionConfig,
)
from control.config_and_enums.controller_enums import ControllerStates
from control.interface_classes import servo_interface
from matplotlib.animation import FuncAnimation
from pinocchio import robot_wrapper
from scipy.spatial.transform import Rotation


class ArmController:
    def __init__(
        self,
        pi_obj: raspberry_pi.RaspberryPi,
        joystick_obj: ps4_joystick.Ps4Joystick,
        arm_segments_config: Dict[Tuple[int, str], ServoConnectionConfig],
        frame_rate=100,
        auto_save_controller_states_to_file=True,
        actuator_velocities_deg_per_ms=tuple(
            i * 0.02 for i in range(1, 10)
        ),  # angular velocity
        cartesian_velocities_mm__per_ms=tuple(i * 0.03 for i in range(1, 10)),
        initial_velocity_level=3,
        home_folder_path="~/.ato/",
    ):
        self._input_states = None
        assert isinstance(pi_obj, raspberry_pi.RaspberryPi)
        self.__pi_obj = pi_obj

        assert isinstance(joystick_obj, ps4_joystick.Ps4Joystick)
        self.__joystick_obj = joystick_obj
        self.__joystick_obj.connect_arm_controller(self)

        self._indexed_servo_names: Tuple[str]
        self._indexed_servo_objs: Tuple[servo_interface.ServoInterface]
        self._indexed_servo_configs: Tuple[ServoConnectionConfig]
        self._indexed_segment_lengths: Tuple[float]
        self._gripper_length: float

        (
            self._indexed_servo_names,
            self._indexed_servo_objs,
            self._indexed_servo_configs,
            self._indexed_segment_lengths,
            self._gripper_length,
            self.__actuator_indices_mapping,
            self.__gripper_index_mapping,
            self.num_segments,
        ) = ArmController.__register_servo_objs(
            arm_segments_config=arm_segments_config, pi_obj=self.pi_obj
        )

        self.home_positions = self.__initialize_home_positions(
            num_segments=self.num_segments
        )

        self._inverse_indexed_servo_names = {
            name: index for index, name in enumerate(self._indexed_servo_names)
        }

        arm_position.ActuatorPositions.set_arm_segment_lengths(
            arm_segment_lengths=self._indexed_segment_lengths
        )
        arm_position.ActuatorPositions.set_actuator_indices_mapping(
            actuator_indices_mapping=self.__actuator_indices_mapping
        )
        arm_position.ActuatorPositions.set_gripper_length(
            gripper_length=self._gripper_length
        )
        arm_position.ActuatorPositions.set_gripper_index_mapping(
            gripper_index_mapping=self.__gripper_index_mapping
        )
        arm_position.ActuatorPositions.set_rotation_ranges(
            rotation_ranges=self._get_indexed_rotation_ranges()
        )

        current_dir = pathlib.Path(__file__).parent
        self.urdf_filename = os.path.join(
            current_dir,
            "..",
            "..",
            "urdf",
            arm_segments_config[ArmConnectionAttributes.URDF_FILENAME],
        )

        try:
            self.robot: pinocchio.robot_wrapper.RobotWrapper = (
                robot_wrapper.RobotWrapper.BuildFromURDF(self.urdf_filename)
            )  # Add pinocchio wrapper
        except Exception as e:
            logging.warning(
                f"Failed to load robot from urdf_filename: {self.urdf_filename}, {e}"
            )
            logging.warning(f"Inverse kinematics will not be available.")
            self.robot = None

        self.frame_rate = frame_rate
        self.interval_ms = 1000 // frame_rate

        self.__auto_save_controller_states_to_file = auto_save_controller_states_to_file

        self.__controller_thread = None
        self.__to_stop_clock = False
        self.__to_stop_visualization = False
        self.__segment_id_pointer = 0

        # velocity settings
        self.__actuator_velocities_deg_per_ms = actuator_velocities_deg_per_ms
        self.__cartesian_velocities_mm__per_ms = cartesian_velocities_mm__per_ms
        self.__velocity_level = initial_velocity_level

        # trajectory replay (joint space)
        self.__trajectory_saved: arm_trajectory.TrajectoryActuatorPositions = None
        self.__trajectory_to_play: arm_trajectory.ArmTrajectory = None

        # joint space control: tracks current position of actuators
        self.__current_actuator_position_obj = None
        self.__current_robot_pose_detail = None
        self.__current_endeffector_pose = None

        self.__positions_momentum = None

        # cartesian space control, tracks:
        # where is the intended pose
        self._intended_pose: arm_position.EndeffectorPose = None
        # where is the (realistic) attempted pose (best effort towards the intended pose, because not all pose are reachable)
        # self.__attempted_pose: motion.EndeffectorPose = None
        # where is the attempted joint positions == IK(attempted pose)
        # self.__attempted_joint_positions: motion.ActuatorPositions = None
        # where is the current joint positions, max velocity may not keep up achieving the attempted
        # is self.__current_actuator_position_obj

        self.reset_input_states()
        self.reset_controller_flags()
        self.reset_trajectory_in_editing()

        self.load_controller_states()
        self.__update_intended_pose_to_current_pose()

        self._home_folder_path = home_folder_path
        self.create_folder(folder_path=home_folder_path)

        self.__controller_start_time = datetime.now()
        self.ready = True

    def __del__(self):
        try:
            pybullet.disconnect()
        except Exception as e:
            logging.warning(f"Failed to disconnect pybullet, exception: {e}")

    def reset_input_states(self):
        # to be override
        return

    @staticmethod
    def create_folder(folder_path):
        folder_path = os.path.expanduser(folder_path)
        pathlib.Path(folder_path).mkdir(parents=True, exist_ok=True)

    @property
    def is_at_home_positions(self):
        return np.allclose(
            self.home_positions,
            self.__fetch_indexed_actuator_positions(),
            rtol=0.001,
        )  # 0.1% tolerance

    def get_input_states(self):
        return self._input_states

    def __initialize_home_positions(self, num_segments):
        # customization of home_positions via config is not allowed
        # those hard coded values works with ik cache
        if num_segments == 3:
            return (
                0,
                60,
                0,
                -100,
                0,
                -30,
                0,
            )  # return (0, 60, 0, -100, 0, -50, 0) after fixing pitch bug
            # or (0, 20, 0, -80, 0, -30, 0),
        elif num_segments == 2:
            return (0, -20, 0, -70, 0)
        else:
            raise Exception(f"Unsupported num_segments == {num_segments}")

    @property
    def controller_states(self):
        return self.__controller_states

    def reset_controller_flags(self):
        self.__controller_states = {
            ControllerStates.LOG_INFO_EACH_TENTH_SECOND: False,
            ControllerStates.CURRENT_MODE: ControllerStates.DEFAULT_MODE,
            ControllerStates.RECORDING_ON: None,
        }

    @property
    def controller_modes(self):
        """
        - IN_CARTESIAN_MODE
        - Under development and disabled -- limited functionality with preformings to be improved
        - IN_JOINT_SPACE_MODE
        - Control each servo motor individually
        - IN_SETTING_MODE
        - Calibrating between physical and logical position of motors, change velocity
        - IN_TRAJECTORY_EDITING_MODE
        - To create a trajectory, either by setting waypoints and pauses, or recording one live.
        """
        return [
            ControllerStates.IN_CARTESIAN_MODE,
            ControllerStates.IN_JOINT_SPACE_MODE,
            ControllerStates.IN_SETTING_MODE,
            ControllerStates.IN_TRAJECTORY_EDITING_MODE,
        ]

    def set_controller_mode(self, mode):
        assert mode in self.controller_modes
        self.update_controller_state(key=ControllerStates.CURRENT_MODE, value=mode)
        logging.info(f"Switched to {mode} mode")
        self.reset_input_states()

    def update_controller_state(self, key, value):
        self.__controller_states[key] = value

    @property
    def controller_mode(self):
        return self.__controller_states[ControllerStates.CURRENT_MODE]

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
                loaded_indexed_servo_names = arm_config["servo_names"]
                if set(self._indexed_servo_names) != set(loaded_indexed_servo_names):
                    logging.warning(
                        f"Skipped loading states from {file_path} -- likely saved from a different arm configuration."
                    )
                    return

                # calibration_deltas
                for unique_name, position in zip(
                    self._indexed_servo_names, arm_config["calibration_deltas"]
                ):
                    self._indexed_servo_objs[
                        self.__get_index_by_servo_name(unique_name)
                    ].set_calibration_delta(calibration_delta=position)

                # payload_positions
                actuator_positions = arm_position.ActuatorPositions(
                    actuator_positions=arm_config["payload_positions"]
                )
                self.__move_servos_by_joint_space(
                    joint_positions=actuator_positions.actuator_positions
                )
                self.__do_internal_updates_for_current_actuator_positions()

                # saved_trajectory
                if arm_config["saved_trajectory"] is not None:
                    self.__trajectory_saved = (
                        arm_trajectory.TrajectoryActuatorPositions()
                    )
                    for timestamp, positions in arm_config["saved_trajectory"]:
                        self.__trajectory_saved.append(
                            timestamp_delta=timestamp,
                            positions=arm_position.ActuatorPositions(
                                actuator_positions=positions
                            ),
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
        logging.info(f"Replaying: {self.__trajectory_saved}")
        trajectory = (
            arm_trajectory.TrajectoryActuatorPositions.prepend_reposition_to_trajectory(
                target_trajectory=self.__trajectory_saved,
                current_positions=self.__fetch_cached_actuator_position_obj(),
                velocities=self.actuator_velocities_scaled,
            )
        )
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

    solver_perf_stats = {"count": 0, "avg_time_ms": None}

    def __solve_valid_movements_from_target_pose(
        self,
        target_pose: arm_position.EndeffectorPose,
        time_delta_ms: float,
        restrict_max_speed=False,
        use_math_based_impl=False,
    ):
        actuator_positions = self.__fetch_indexed_actuator_positions()

        ik_solved_positions = (
            arm_position.EndeffectorPose.inverse_kinematics_pinocchio_based(
                robot=self.robot,
                target_pose=target_pose,
                time_delta_ms=time_delta_ms,
                initial_joint_positions=actuator_positions[:-1],
                gripper_position=actuator_positions[-1],
            )
        )

        if ik_solved_positions is None:
            logging.info(f"Failed to solve @{target_pose}")
            return None, None
        else:
            start_time = datetime.now()
            current_actuator_positions = np.array(
                self.__fetch_indexed_actuator_positions()
            )
            positions_delta = (
                ik_solved_positions.actuator_positions - current_actuator_positions
            )
            resulted_pose = ik_solved_positions.expected_pose

            if restrict_max_speed:
                joint_velocity_limits = (
                    np.array(
                        [
                            config.velocity_magnifier
                            for config in self._indexed_servo_configs
                        ]
                    )
                    * self.actuator_velocity
                    * time_delta_ms
                )

                positions_normalization_ratio = np.max(
                    np.abs(positions_delta / joint_velocity_limits)
                )

                if positions_normalization_ratio > 1:
                    positions_delta /= positions_normalization_ratio
                    logging.info(
                        f"applied positions_normalization_ratio: {positions_normalization_ratio}"
                    )

            actuator_positions = current_actuator_positions + positions_delta
            if use_math_based_impl:
                resulted_pose = (
                    arm_position.ActuatorPositions.forward_kinematics_math_based(
                        joint_positions=actuator_positions[:-1],  # remove gripper
                    ).endeffector_pose_intrinsic
                )
            else:
                resulted_pose = arm_position.ActuatorPositions.forward_kinematics_pinocchio_based(
                    robot=self.robot,
                    joint_config_radians=arm_position.ActuatorPositions.convert_to_pinocchio_configuration(
                        joint_positions=actuator_positions[:-1]
                    ),
                )

            # update perf stats
            ik_solving_time = (datetime.now() - start_time).total_seconds() * 1000.0
            if self.solver_perf_stats["avg_time_ms"] is None:
                self.solver_perf_stats["avg_time_ms"] = ik_solving_time
            else:
                self.solver_perf_stats["avg_time_ms"] = (
                    self.solver_perf_stats["avg_time_ms"]
                    * self.solver_perf_stats["count"]
                    + ik_solving_time
                ) / (self.solver_perf_stats["count"] + 1)
            self.solver_perf_stats["count"] += 1
            logging.debug(self.solver_perf_stats)

            return positions_delta, resulted_pose

    def move_to_installation_position(self):
        for servo in self._indexed_servo_objs:
            servo.move_to_installation_position()

    # if already at home position, move all actuators to logical zero (installation position)
    def move_to_home_positions_otherwise_zeros(self):
        if self.is_at_home_positions:
            actuator_positions = arm_position.ActuatorPositions(
                actuator_positions=[0] * (self.num_segments * 2 + 1)
            )
            self.move_to_actuator_positions(actuator_positions=actuator_positions)
        else:
            actuator_positions = arm_position.ActuatorPositions(
                actuator_positions=self.home_positions
            )
            self.move_to_actuator_positions(actuator_positions=actuator_positions)

    def reset_trajectory_in_editing(self):
        self.__trajectory_in_editing: arm_trajectory.TrajectoryActuatorPositions = (
            arm_trajectory.TrajectoryActuatorPositions(start_time=datetime.now())
        )

    def stop_recording_trajectory(self):
        self.update_controller_state(key=ControllerStates.RECORDING_ON, value=None)
        logging.info(f"Stopped recording trajectory")

    def start_recording_trajectory(self):
        self.update_controller_state(
            key=ControllerStates.RECORDING_ON,
            value=(self.__trajectory_in_editing.latest_timestamp, datetime.now()),
        )
        self.trajectory_in_editing_append_waypoint()
        logging.info(f"Started recording trajectory")

    def save_trajectory(self):
        self.stop_recording_trajectory()
        self.__trajectory_saved = copy.deepcopy(self.__trajectory_in_editing)
        self.reset_trajectory_in_editing()
        logging.info(f"Saved: {self.__trajectory_saved}")

    def trajectory_in_editing_append_waypoint(self):
        if len(self.__trajectory_in_editing) == 0:
            self.__trajectory_in_editing.append(
                timestamp_delta=0,
                positions=self.__fetch_cached_actuator_position_obj(),
            )
        else:
            self.__trajectory_in_editing.append_waypoint(
                waypoint=self.__fetch_cached_actuator_position_obj(),
                velocities=self.actuator_velocities_scaled,
            )

    def trajectory_in_editing_append_pause(self, pause_sec=1):
        if len(self.__trajectory_in_editing) == 0:
            self.__trajectory_in_editing.append(
                timestamp_delta=0,
                positions=self.__fetch_cached_actuator_position_obj(),
            )
        self.__trajectory_in_editing.append_pause(pause_sec=pause_sec)

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
            self.__to_stop_visualization = True
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

        num_segments = (
            max((key for key in arm_segments_config.keys() if isinstance(key, int))) + 1
        )
        for segment_id in range(num_segments):
            if segment_id < 0:
                # gripper
                continue
            assert (
                segment_id in arm_segments_config
            ), f"{segment_id} in {arm_segments_config}"
            segment_config = arm_segments_config[segment_id]
            assert (
                ArmConnectionAttributes.PHYSICAL_LENGTH in segment_config
            ), f"{ArmConnectionAttributes.PHYSICAL_LENGTH} in {segment_config}"
            indexed_segment_lengths.append(
                segment_config[ArmConnectionAttributes.PHYSICAL_LENGTH]
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
        gripper_length = arm_segments_config[-1][
            ArmConnectionAttributes.PHYSICAL_LENGTH
        ]
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

    def queue_up_trajectory(self, trajectory: arm_trajectory.ArmTrajectory):
        if trajectory is None:
            return
        if isinstance(trajectory, arm_trajectory.TrajectoryEndeffectorPose):
            trajectory = trajectory.inverse_kinematics()
        # __trajectory_to_play is checked each cycle
        self.__trajectory_to_play = copy.deepcopy(trajectory)
        self.__trajectory_to_play.start_time = datetime.now()

    def move_to_actuator_positions(
        self, actuator_positions: arm_position.ActuatorPositions, pause_sec: float = 1
    ):
        trajectory = arm_trajectory.TrajectoryActuatorPositions(
            timestamps=[0], multiple_positions=[actuator_positions]
        )
        trajectory = (
            arm_trajectory.TrajectoryActuatorPositions.prepend_reposition_to_trajectory(
                target_trajectory=trajectory,
                current_positions=self.__fetch_cached_actuator_position_obj(),
                velocities=self.actuator_velocities_scaled,
                pause_sec=pause_sec,
            )
        )
        self.queue_up_trajectory(trajectory)

    def is_playing_trajectory(self, current_time=None):
        if current_time is None:
            current_time = datetime.now()
        if self.__trajectory_to_play is not None:  # replay mode active
            since_replay_start_delta = (
                current_time - self.__trajectory_to_play.start_time
            )
            trajectory: arm_position.Trajectory = self.__trajectory_to_play
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

    def __update_intended_pose_to_current_pose(self, override_pose=None):
        self._intended_pose = arm_position.EndeffectorPose(
            pose=self.__fetch_cached_endeffector_pose()
            if override_pose is None
            else override_pose
        )

    def __advance_clock(self, current_time, time_delta_ms):
        show_info = not self.__controller_states[
            ControllerStates.LOG_INFO_EACH_TENTH_SECOND
        ]
        if self.is_playing_trajectory(current_time=current_time):
            self.__positions_momentum = None

            trajectory: arm_position.Trajectory = self.__trajectory_to_play
            since_replay_start_delta = (
                current_time - self.__trajectory_to_play.start_time
            )
            positions = trajectory.get(since_replay_start_delta.total_seconds())
            for index, unique_name in enumerate(self._indexed_servo_names):
                position = positions[index]
                self.__move_servo(
                    unique_name=unique_name, position=position, show_info=show_info
                )
            self.__update_intended_pose_to_current_pose()
        elif (
            self.__controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_CARTESIAN_MODE
        ):
            intended_pose = self._parse_movement_updates_in_cartesian_space(
                time_delta_ms=time_delta_ms,
            )
            if intended_pose is None:
                # case 1: no user input
                #   Movement: no
                #   Reset intended: yes
                # never move if no user input, to prevent out of control scenario
                pass
            else:
                (
                    positions_delta,
                    resulted_pose,
                ) = self.__solve_valid_movements_from_target_pose(
                    target_pose=intended_pose, time_delta_ms=time_delta_ms
                )

                if positions_delta is not None:
                    # case 2: found valid movements
                    #   Movement: yes
                    #   Reset intended: yes
                    self.__positions_momentum = positions_delta
                    assert resulted_pose is not None
                    # self._intended_pose = arm_position.EndeffectorPose(
                    #     pose=resulted_pose
                    # )

                    self._move_servos_by_joint_space_delta(
                        joint_positions_delta=self.__positions_momentum
                    )
                else:
                    # case 3: can't solve for valid movements
                    #   Movement: no
                    #   Reset intended: no
                    # if user input continues, the intented pose and actual current pose will diverge
                    # this makes it easier recovering from bad states, i.e. states that leads to deadends
                    logging.info(
                        "No valid movements found __update_intended_pose_to_current_pose"
                    )

            self.__update_intended_pose_to_current_pose()

        elif (
            self.__controller_states[ControllerStates.CURRENT_MODE]
            == ControllerStates.IN_JOINT_SPACE_MODE
        ):
            self.__positions_momentum = None

            joint_positions_delta = self._parse_movement_updates_in_joint_space(
                time_delta_ms=time_delta_ms,
                show_info=show_info,
            )
            self._move_servos_by_joint_space_delta(
                joint_positions_delta=joint_positions_delta, show_info=show_info
            )
            # update intended pose
            # even if no user input was recorded
            self.__update_intended_pose_to_current_pose()

    def _move_servos_by_joint_space_delta(self, joint_positions_delta, show_info=False):
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
        self.__fetch_cached_actuator_position_obj(refresh=True)
        self.__fetch_cached_robot_pose_detail(refresh=True)
        self.__fetch_cached_endeffector_pose(refresh=True)

        recording_on: Tuple = self.__controller_states[ControllerStates.RECORDING_ON]
        if recording_on is not None:
            (
                trajectory_timestamp_at_recording_start,
                recording_start_clock_time,
            ) = recording_on
            delta = (
                trajectory_timestamp_at_recording_start
                + (datetime.now() - recording_start_clock_time).total_seconds()
            )
            self.__trajectory_in_editing.append(
                timestamp_delta=delta,
                positions=self.__fetch_cached_actuator_position_obj(),
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

            time_delta_sec = int(
                (last - self.__controller_start_time).total_seconds()
            ) - int((current - self.__controller_start_time).total_seconds())
            time_delta_tenths_sec = int(
                (last - self.__controller_start_time).total_seconds() * 10
            ) - int((current - self.__controller_start_time).total_seconds() * 10)
            time_delta_ms = (current - last).total_seconds() * 1000

            if (
                self.__controller_states[ControllerStates.LOG_INFO_EACH_TENTH_SECOND]
                and time_delta_tenths_sec < 0
            ):
                self.__refresh_states_display()
            if time_delta_sec < 0:
                self.save_controller_states(
                    dry_run=not self.__auto_save_controller_states_to_file
                )

            # this call could take time, so simply sleep(self.interval) wouldn't be precise
            if time_delta_ms >= interval_ms:
                self.__advance_clock(
                    current_time=current,
                    time_delta_ms=time_delta_ms,
                )
                self.__do_internal_updates_for_current_actuator_positions()
                last = datetime.now()

            time.sleep(interval_ms / 1000)
        self.__to_stop_clock = False
        logging.info(f"Thread for Arm Controller stopped.")

    def __refresh_states_display(self):
        logging.info(str(self.joystick_obj))
        logging.info(str(self))

    @property
    def trajectory_in_editing(self):
        return self.__trajectory_in_editing

    @property
    def pi_obj(self):
        assert self.__pi_obj is not None
        return self.__pi_obj

    @property
    def joystick_obj(self):
        assert self.__joystick_obj is not None
        return self.__joystick_obj

    @property
    # each actuator may move at a different speed
    # e.g. gripper move faster than joints
    def actuator_velocities_scaled(self):
        return np.array(
            [
                self.actuator_velocity
                * self._indexed_servo_configs[
                    self.__get_index_by_servo_name(unique_name)
                ].velocity_magnifier
                for unique_name in self._indexed_servo_names
            ]
        )

    @property
    # degree per ms
    def actuator_velocity(self):
        return self.__actuator_velocities_deg_per_ms[self.__velocity_level]

    @property
    # mm per ms
    def cartesian_velocity(self):
        return self.__cartesian_velocities_mm__per_ms[self.__velocity_level]

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

    def visualize_joints_pibullet_thread_loop(self, robot_id):
        while not self.__to_stop_visualization:
            joint_positions = self.__fetch_indexed_joint_positions()
            pybullet.stepSimulation()
            joint_positions_radians = [math.radians(deg) for deg in joint_positions]
            for joint_index, joint_position in enumerate(joint_positions_radians):
                pybullet.setJointMotorControl2(
                    robot_id,
                    joint_index,
                    pybullet.POSITION_CONTROL,
                    targetPosition=joint_position,
                )
            time.sleep(0.01)

    def visualize_joints_pibullet(
        self, scaling_factor=0.01, start_xyz=(0, 0, 0), start_rpy=(0, 0, 0)
    ):
        pybullet.connect(pybullet.GUI)
        pybullet.setGravity(0, 0, 0)  # (0,0,-10)
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=12,
            cameraYaw=15,
            cameraPitch=-75,
            cameraTargetPosition=[0, 0, 0],
        )

        startOrientation = pybullet.getQuaternionFromEuler(start_rpy)
        robot_id = pybullet.loadURDF(
            self.urdf_filename,
            start_xyz,
            startOrientation,
            globalScaling=scaling_factor,
        )

        # create thread
        # pybullet.setRealTimeSimulation(1)
        thread = threading.Thread(
            target=self.visualize_joints_pibullet_thread_loop, args=(robot_id,)
        )
        thread.start()
        return pybullet.getBasePositionAndOrientation(robot_id)

    def visualize_joints_matplotlib(self):
        fig = plt.figure()
        axes_limits = max(self._indexed_segment_lengths) * 3
        ax = plt.axes(projection="3d")
        # axis are determined by right hand rule, X = thumb = roll, Y = index = pitch, Z = middle = yaw
        # assume the first segment of arm is installed upwards, so the X axis (roll) would be camera's Z
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(-axes_limits, axes_limits)
        ax.set_ylim(-axes_limits, axes_limits)
        ax.set_zlim(-axes_limits / 10, axes_limits)

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
            pose_detail = self.__fetch_cached_robot_pose_detail()
            plotted_segments[0].set_data_3d(
                pose_detail.segments_xyz[:, 0],
                pose_detail.segments_xyz[:, 1],
                pose_detail.segments_xyz[:, 2],
            )
            for plotted_element in plotted_elements.values():
                plotted_element.remove()
            for index, segments_reference_frame in enumerate(
                pose_detail.segments_reference_frames[
                    :, :-1
                ]  # [:, :-1] to remove z axis, which is overlapped with the next segment
            ):
                plotted_elements[f"segments_reference_frame{index}"] = draw_3_axes(
                    xyz=pose_detail.segments_xyz[index : index + 1],
                    uvw=segments_reference_frame,
                    length=self._indexed_segment_lengths[-1] / 2,
                )
            plotted_elements["endeffector_reference_frame"] = draw_3_axes(
                xyz=pose_detail.segments_xyz[-1:],
                uvw=pose_detail.endeffector_reference_frame[:],
                length=self._gripper_length,
            )
            if self._intended_pose is not None:
                global_frame = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                rpy = self._intended_pose.rpy
                uvw = np.array(
                    [
                        Rotation.from_euler(
                            "XYZ",
                            rpy,
                            degrees=True,
                        ).apply(axis)
                        for axis in global_frame
                    ]
                ).transpose()
                plotted_elements["intended_pose"] = draw_3_axes(
                    xyz=np.array([self._intended_pose.xyz]),
                    uvw=np.array([uvw]),
                    length=self._gripper_length,
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
        estimated_pose = self.__fetch_cached_endeffector_pose()
        return f"{time_str} @activated_segment_ids={self.activated_segment_ids} $velocity deg/ms={self.actuator_velocity} {self.frame_rate}hz\n{actuator_states}\nestimated_pose_in_global_frame={estimated_pose}\n"

    def _get_indexed_rotation_ranges(self):
        return np.array(
            [config.rotation_range for config in self._indexed_servo_configs]
        )

    def __get_index_by_servo_name(self, name):
        return self._inverse_indexed_servo_names[name]

    # the Sequence_Index would match self._indexed_ variables
    def __fetch_indexed_actuator_positions(self):
        return tuple((obj.payload_position for obj in self._indexed_servo_objs))

    def __fetch_indexed_joint_positions(self):
        return self.__fetch_indexed_actuator_positions()[:-1]

    def __fetch_cached_actuator_position_obj(
        self, refresh=False
    ) -> arm_position.ActuatorPositions:
        if refresh or self.__current_actuator_position_obj is None:
            positions = self.__fetch_indexed_actuator_positions()
            assert isinstance(positions[0], numbers.Number), type(positions[0])
            self.__current_actuator_position_obj = arm_position.ActuatorPositions(
                actuator_positions=positions
            )
        return self.__current_actuator_position_obj

    def __fetch_cached_robot_pose_detail(self, refresh=False):
        if arm_position.ActuatorPositions.is_ready() and (
            refresh or self.__current_robot_pose_detail is None
        ):
            joint_positions = self.__fetch_indexed_joint_positions()
            self.__current_robot_pose_detail = (
                arm_position.ActuatorPositions.forward_kinematics_math_based(
                    joint_positions=joint_positions
                )
            )
        return self.__current_robot_pose_detail

    def __fetch_cached_endeffector_pose(self, use_math_based_impl=False, refresh=False):
        if use_math_based_impl:
            return self.__fetch_cached_robot_pose_detail(
                refresh=refresh
            ).endeffector_pose_intrinsic
        else:
            if refresh or self.__current_endeffector_pose is None:
                self.__current_endeffector_pose = arm_position.ActuatorPositions.forward_kinematics_pinocchio_based(
                    robot=self.robot,
                    joint_config_radians=arm_position.ActuatorPositions.convert_to_pinocchio_configuration(
                        joint_positions=self.__fetch_indexed_joint_positions()
                    ),
                )
            return self.__current_endeffector_pose
