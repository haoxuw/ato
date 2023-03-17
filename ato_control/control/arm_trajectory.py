# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import bisect
import collections
import json
from typing import List, Tuple

import numpy as np
from control.arm_position import ActuatorPositions, ArmPosition, EndeffectorPose


class ArmTrajectory:
    def __init__(self, start_time=None, trajectory=None):
        if trajectory is None:
            self.__trajectory: List[Tuple(float, ArmPosition)] = []
        else:
            self.__trajectory = list(trajectory)
        self.__dim = None

        self.start_time = start_time
        self.pointer_offset = 0

    @property
    def duration(self):
        if not self.__trajectory:
            return 0
        return self.__trajectory[-1][0]

    @property
    def dimension(self):
        return self.__dim

    @property
    def length(self):
        if self.__trajectory is None:
            return 0
        return len(self.__trajectory)

    def __len__(self):
        return self.length

    def _append_vector(self, timestamp_delta: float, vector: collections.abc.Sequence):
        if len(self.__trajectory) > 0 and timestamp_delta <= self.__trajectory[-1][0]:
            return
        if self.__dim is None:
            self.__dim = len(vector)
        else:
            assert self.__dim == len(vector), f"{self.__dim} != {len(vector)}"
        self.__trajectory.append(tuple((timestamp_delta, vector)))

    @property
    def trajectory(self):
        return self.__trajectory

    def __set_trajectory(self, trajectory):
        self.__trajectory = sorted(trajectory)

    def move_pointer(self, offset):
        self.pointer_offset = max(0, self.pointer_offset + offset)

    # may perform linear interpolation
    def get(self, now: float):
        if len(self.__trajectory) <= 1:
            return None
        timestamps = tuple((position[0] for position in self.__trajectory))
        # python 3.9 doesn't support bisect with key

        index = bisect.bisect(timestamps, now)
        if index < 1:
            return self.__trajectory[0][1].to_np()
        elif index == len(self.__trajectory):
            return self.__trajectory[-1][1].to_np()
        else:
            current, towards = np.array(self.__trajectory)[index - 1 : index + 1]
            # current: [timestamp, ActuatorPositions]

            progress = (now - current[0]) / (towards[0] - current[0])

            delta = towards[1].to_np() - current[1].to_np()
            assert delta.shape == (self.dimension,), delta.shape
            assert (
                current[0] <= now <= towards[0] and towards[0] - current[0] > 0
            ), f"{current[0]} <= {now} <= {towards[0]}"

            return current[1].to_np() + delta * progress

    def reset(self):
        self.__set_trajectory([])

    def __str__(self) -> str:
        return "\n".join(
            [f"\t@{t}:{str(position)}" for t, position in self.__trajectory]
        )

    def json_dumps(self):
        return json.dumps(self.__trajectory)

    def json_loads(self, string):
        self.__trajectory = json.loads(string)

    def to_serializable(self):
        return tuple(
            tuple((timestamp, position.to_serializable()))
            for (timestamp, position) in self.__trajectory
        )


class TrajectoryActuatorPositions(ArmTrajectory):
    def __init__(self, timestamps=[], multiple_positions=[], start_time=None):
        super().__init__(start_time=start_time)
        assert len(timestamps) == len(multiple_positions), (
            len(timestamps),
            len(multiple_positions),
        )
        for timestamp, positions in zip(timestamps, multiple_positions):
            self.append(timestamp_delta=timestamp, positions=positions)

    def append(self, timestamp_delta: float, positions: ActuatorPositions):
        assert isinstance(positions, ActuatorPositions)
        super()._append_vector(timestamp_delta=timestamp_delta, vector=positions)

    @property
    def starting_positions(self):
        if self.length > 0:
            return self.trajectory[0][1]
        return None

    @property
    def latest_timestamp(self):
        if self.length > 0:
            return self.trajectory[-1][0]
        return 0

    def forward_kinematics(self):
        endeffector_trajectory = TrajectoryEndeffectorPose(
            starting_positions=self.starting_positions
        )
        for timestamp_delta, positions in self.trajectory:
            positions: ActuatorPositions
            pose = positions.forward_kinematics_ml_based()
            endeffector_trajectory.append(
                timestamp_delta=timestamp_delta,
                pose=EndeffectorPose(
                    pose=pose, gripper_position=positions.gripper_position
                ),
            )
        return endeffector_trajectory

    def append_waypoint(
        self,
        waypoint: ActuatorPositions,
        velocities: Tuple[float],
        least_duration: float = 0,
    ):
        assert len(self) > 0
        last_timestamp, last_position = self.trajectory[-1]
        required_duration = TrajectoryActuatorPositions.calculate_duration_needed(
            start=last_position, destination=waypoint, velocities=velocities
        )
        duration = max(least_duration, required_duration)
        self.append(timestamp_delta=last_timestamp + duration, positions=waypoint)
        logging.info(f"Added waypoint {waypoint}")

    def append_pause(self, pause_sec=1):
        assert len(self) > 0
        last_timestamp, last_position = self.trajectory[-1]
        self.append(timestamp_delta=last_timestamp + pause_sec, positions=last_position)
        logging.info(f"Added pause {pause_sec}")

    @staticmethod
    def calculate_duration_needed(
        start: ActuatorPositions,
        destination: ActuatorPositions,
        velocities: Tuple[float],
    ):
        offsets = destination.actuator_positions - start.actuator_positions
        ms_per_s = 1000
        duration = np.abs(offsets / velocities / ms_per_s)
        max_duration = np.amax(duration)
        return max_duration

    @staticmethod
    def prepend_reposition_to_trajectory(
        target_trajectory,
        current_positions: ActuatorPositions,
        velocities: Tuple[float],
        pause_sec: float = 1,
    ):
        assert isinstance(target_trajectory, TrajectoryActuatorPositions), type(
            target_trajectory
        )
        _, first_position = target_trajectory.trajectory[0]
        max_duration = TrajectoryActuatorPositions.calculate_duration_needed(
            start=current_positions, destination=first_position, velocities=velocities
        )
        new_trajectory = TrajectoryActuatorPositions()
        new_trajectory.append(timestamp_delta=0, positions=current_positions)
        new_trajectory.append(timestamp_delta=max_duration, positions=first_position)
        if pause_sec > 0:
            new_trajectory.append(
                timestamp_delta=max_duration + pause_sec, positions=first_position
            )
        else:
            pause_sec = 0
        for timestamp, actuator_positions in target_trajectory.trajectory[1:]:
            new_trajectory.append(
                timestamp_delta=max_duration + pause_sec + timestamp,
                positions=actuator_positions,
            )
        return new_trajectory


class TrajectoryEndeffectorPose(ArmTrajectory):
    def __init__(self, starting_positions: ActuatorPositions = None, trajectory=None):
        super().__init__(trajectory)
        self.starting_positions = starting_positions

    def append(self, timestamp_delta: float, pose: EndeffectorPose):
        assert isinstance(pose, EndeffectorPose)
        super()._append_vector(timestamp_delta=timestamp_delta, vector=pose)

    def inverse_kinematics(self, starting_positions: ActuatorPositions = None):
        if starting_positions is None:
            # a starting_positions is needed to determine the optimal path
            starting_positions = self.starting_positions
        assert starting_positions is not None

        actuator_positions_trajectory = TrajectoryActuatorPositions()
        current_positions = np.array(starting_positions.joint_positions)
        for index in range(1, len(self.trajectory)):
            timestamp, new_pose = self.trajectory[index]
            new_pose: EndeffectorPose
            new_positions, _ = new_pose.inverse_kinematics_ikpy(
                initial_joint_positions=current_positions[:-1],
            )
            if new_positions is None:
                continue
            actuator_positions_trajectory.append(
                timestamp_delta=timestamp,
                positions=ActuatorPositions(joint_positions=new_positions),
            )
            current_positions = new_positions
        return actuator_positions_trajectory
