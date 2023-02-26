# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import bisect
import collections
import json
import logging
import numbers
import pathlib
import sys
from abc import ABC, abstractmethod
from typing import List, Tuple

import ikpy.chain
import numpy as np
from scipy.spatial.transform import Rotation

try:
    sys.path.append(f"{pathlib.Path(__file__).parent}/../../learning/")
    # pylint: disable=import-error
    from learn_kinematics import forward_kinematics, inverse_kinematics
except Exception as import_e:
    logging.warning(f"Skipped importing the learning package, due to: {import_e}")


class Position(ABC):
    @abstractmethod
    def set(self, sequence):
        pass

    @abstractmethod
    def to_tuple(self):
        pass

    @abstractmethod
    def to_np(self):
        pass

    @abstractmethod
    def to_serializable(self):
        pass

    @abstractmethod
    def __len__(self):
        pass

    def _verify_sequence_valid(self, sequence, shape):
        assert isinstance(
            sequence, (np.ndarray, np.generic, collections.abc.Sequence)
        ), type(sequence)
        assert (
            np.array(sequence).shape == shape
        ), f"{np.array(sequence).shape} != {shape}"
        for val in sequence:
            assert isinstance(val, numbers.Number), (val, type(val))


class Singletons:
    __forward_kinematics_model = None
    __inverse_kinematics_model = None

    @classmethod
    def get_forward_kinematics_model(cls):
        if cls.__forward_kinematics_model is None:
            cls.__forward_kinematics_model = (
                forward_kinematics.ForwardKinematics.load_from_checkpoint()
            )
        return cls.__forward_kinematics_model

    @classmethod
    def get_inverse_kinematics_model(cls):
        if cls.__inverse_kinematics_model is None:
            cls.__inverse_kinematics_model = (
                inverse_kinematics.InverseKinematics.load_from_checkpoint()
            )
        return cls.__inverse_kinematics_model


class ActuatorPositions(Position):
    # e.g. when actuator_indices_mapping = ((0,1), (4,5)), it means:
    # self.__actuator_positions[0] is roll of the 1st segment
    # self.__actuator_positions[1] is pitch of the 1st segment
    # self.__actuator_positions[4] is roll of the 2nd segment
    # self.__actuator_positions[5] is pitch of the 2nd segment
    actuator_indices_mapping = None

    # e.g. when arm_segment_lengths = (30, 60), it means:
    # total length of the 1st segment is 30, 2nd is 60
    arm_segment_lengths = None

    rotation_ranges = None

    def __init__(self, positions) -> None:
        self.set(sequence=positions)

    @classmethod
    def is_ready(cls):
        return not (
            cls.actuator_indices_mapping is None
            or cls.arm_segment_lengths is None
            or cls.rotation_ranges is None
        )

    @staticmethod
    def get_index(segment_id: int, roll_not_pitch: bool):
        return segment_id * 2 + (0 if roll_not_pitch else 1)

    def set_position(self, val: numbers.Number, segment_id: int, roll_not_pitch: bool):
        self.positions[
            self.get_index(segment_id=segment_id, roll_not_pitch=roll_not_pitch)
        ] = val

    @property
    def positions(self):
        return self.__actuator_positions

    @property
    def actuator_positions(self):
        return self.__actuator_positions[:-1]

    @property
    def gripper_position(self):
        return self.__actuator_positions[-1]

    @classmethod
    def set_actuator_indices_mapping(cls, actuator_indices_mapping):
        assert isinstance(
            actuator_indices_mapping, (np.ndarray, np.generic, collections.abc.Sequence)
        )
        cls.actuator_indices_mapping = np.array(actuator_indices_mapping)

    @classmethod
    def set_gripper_index_mapping(cls, gripper_index_mapping):
        assert isinstance(gripper_index_mapping, int)
        cls.gripper_index_mapping = np.array(gripper_index_mapping)

    @classmethod
    def set_arm_segment_lengths(cls, arm_segment_lengths):
        assert isinstance(
            arm_segment_lengths, (np.ndarray, np.generic, collections.abc.Sequence)
        )
        cls.arm_segment_lengths = np.array(arm_segment_lengths)

    @classmethod
    def set_gripper_length(cls, gripper_length):
        assert isinstance(gripper_length, numbers.Number)
        cls.gripper_length = np.array(gripper_length)

    @classmethod
    def set_rotation_ranges(cls, rotation_ranges):
        assert isinstance(
            rotation_ranges, (np.ndarray, np.generic, collections.abc.Sequence)
        )
        cls.rotation_ranges = np.array(rotation_ranges)

    @classmethod
    def denormalize_relu6(cls, sequence):
        denormalized = [
            new_range[0] + (new_range[1] - new_range[0]) * max(min(val, 6.0), 0) / 6.0
            # min + (max-min)*(val_normalized_0_to_1)
            for val, new_range in zip(sequence, cls.rotation_ranges)
        ]
        return np.array(denormalized, dtype=np.float32)

    @classmethod
    def normalize_to_relu6(cls, sequence):
        normalized = []
        for val, current_range in zip(sequence, cls.rotation_ranges):
            new_val = (
                (val - current_range[0]) / (current_range[1] - current_range[0]) * 6.0
            )
            new_val = max(min(new_val, 6.0), 0)
            normalized.append(new_val)
        return np.array(normalized, dtype=np.float32)

    def set(self, sequence):
        self._verify_sequence_valid(
            sequence=sequence, shape=(len(self.__class__.arm_segment_lengths) * 2 + 1,)
        )
        self.__actuator_positions = np.array(sequence)

    def to_tuple(self):
        return tuple(val for val in self.__actuator_positions)

    def to_np(self):
        return self.__actuator_positions

    def to_serializable(self):
        return self.to_tuple()

    def __len__(self):
        return len(self.__actuator_positions)

    def __str__(self) -> str:
        return f"@{self.positions}"

    def info_str(self) -> str:
        return f"actuator @{self.positions}" + (
            f" segment_actuator_indices: { self.__class__.actuator_indices_mapping} gripper_index_mapping: { self.__class__.gripper_index_mapping}"
            if self.__class__.actuator_indices_mapping is not None
            else ""
        )

    def forward_kinematics_ml_based(self):
        actuator_positions = np.array(self.positions[:6], dtype=np.float32)
        features = ActuatorPositions.normalize_to_relu6(sequence=actuator_positions)
        return Singletons.get_forward_kinematics_model().inference_one(
            features=features
        )

    def forward_kinematics(self):
        assert (
            self.__class__.actuator_indices_mapping is not None
        ), self.__class__.actuator_indices_mapping
        assert (
            self.__class__.arm_segment_lengths is not None
        ), self.__class__.arm_segment_lengths
        actuator_positions = np.array(self.__actuator_positions)
        rotation_sequence = np.array(
            [
                (
                    actuator_positions[segment_index[1]],
                    0,
                    actuator_positions[segment_index[0]],
                )
                for segment_index in self.__class__.actuator_indices_mapping
            ]
        )

        assert (
            rotation_sequence.shape[0] == self.__class__.arm_segment_lengths.shape[0]
        ), f"{rotation_sequence.shape[0]} == {self.__class__.arm_segment_lengths.shape[0]}"

        # not supporting yaw
        def rotate(body_frame_ref_rpy, rotation_rpy):
            assert body_frame_ref_rpy.shape == (
                (3, 3)
            ), f"Expected body_frame_ref_rpy of shape {body_frame_ref_rpy.shape} to describe each of x y z axis' orientation in global reference frame."
            assert rotation_rpy.shape == (
                3,
            ), f"Expected rotation_rpy of shape {rotation_rpy.shape} to describe a sequential rotation for roll pitch and yaw, and in that order."
            for axis_index in [2, 0, 1]:  # roll, 0, pitch
                body_frame_ref_rpy = np.array(
                    [
                        Rotation.from_rotvec(
                            rotation_rpy[axis_index] * body_frame_ref_rpy[axis_index],
                            degrees=True,
                        ).apply(axis)
                        for axis in body_frame_ref_rpy
                    ]
                )
            return body_frame_ref_rpy

        # frame_ref describes the frame of reference's axes
        global_frame = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # x y z axes
        body_frame_ref = np.array(global_frame)

        current_frame_ref_origin = np.array((0, 0, 0), dtype="float")
        segments_xyz = [current_frame_ref_origin]  # base mount
        segments_reference_frames = []

        # in each iteration, we first rotate body_frame_ref, which track orientation of the next segment
        # then we project arm_segment vector to body_frame_ref, for now because body_frame_ref is (l,0,0)
        # the projection is always X/roll of body_frame_ref, i.e. arm_segment @ body_frame_ref  == body_frame_ref[0]
        # finally we add this projected vector to current_frame_ref_origin, and move on to the next segment
        for rotation, arm_segment_length in zip(
            rotation_sequence, self.__class__.arm_segment_lengths
        ):
            arm_segment = np.array([0, 0, arm_segment_length])
            # it's important and counterintuitive body_frame_ref is on the right side,
            # i.e. we are calculating dot product of segment to columns of body_frame_ref,
            # not rows of body_frame_ref, which are normalized vectors describing orientation of axes
            # this is because we are mapping arm_segment to the reference frame, not to the global frame
            current_segment = arm_segment @ body_frame_ref

            current_frame_ref_origin = (
                np.copy(current_frame_ref_origin) + current_segment
            )
            segments_reference_frames.append(body_frame_ref)
            segments_xyz.append(current_frame_ref_origin)
            body_frame_ref = rotate(
                body_frame_ref_rpy=body_frame_ref, rotation_rpy=rotation
            )

        endeffector_xyz = (
            np.copy(current_frame_ref_origin)
            + np.array([0, 0, self.__class__.gripper_length]) @ body_frame_ref
        )
        endeffector_reference_frame = body_frame_ref
        endeffector_rpy = Rotation.align_vectors(global_frame, body_frame_ref)[0]
        endeffector_rpy_intrinsic = endeffector_rpy.as_euler("XYZ", degrees=True)
        endeffector_rpy_extrinsic = endeffector_rpy.as_euler("xyz", degrees=True)
        return {
            "segments_xyz": np.array(segments_xyz),
            "frontend_pose_intrinsic": np.array(
                np.concatenate([segments_xyz[-1], endeffector_rpy_intrinsic])
            ),  # frontend_pose_intrinsic would align with the base of gripper, while endeffector_pose aligns with the jaw of the gripper
            "segments_reference_frames": np.array(segments_reference_frames),
            "endeffector_pose_intrinsic": np.array(
                np.concatenate([endeffector_xyz, endeffector_rpy_intrinsic])
            ),
            "endeffector_pose_extrinsic": np.array(
                np.concatenate([endeffector_xyz, endeffector_rpy_extrinsic])
            ),
            "endeffector_reference_frame": np.array(endeffector_reference_frame),
        }


class EndeffectorPose(Position):
    def __init__(self, pose, gripper_position=0) -> None:
        self.set(sequence=pose)
        self.__gripper_position = gripper_position

    @property
    def gripper_position(self):
        return self.__gripper_position

    @property
    def xyz(self):
        return self.__pose[:3]

    @property
    def rpy(self):
        return self.__pose[3:6]

    @property
    def x(self):
        return self.__pose[0]

    @property
    def y(self):
        return self.__pose[1]

    @property
    def z(self):
        return self.__pose[2]

    @property
    def roll(self):
        return self.__pose[3]

    @property
    def pitch(self):
        return self.__pose[4]

    @property
    def yaw(self):
        return self.__pose[5]

    @property
    def pose(self):
        return self.__pose

    def set(self, sequence):
        self._verify_sequence_valid(sequence=sequence, shape=(6,))
        self.__pose = np.array(sequence)

    def set_xyz(self, sequence):
        self._verify_sequence_valid(sequence=sequence, shape=(3,))
        self.__pose[:3] = sequence

    def set_rpy(self, sequence, is_degrees=True):  # not is_degrees == is_radius
        self._verify_sequence_valid(sequence=sequence, shape=(3,))
        if not is_degrees:
            sequence = np.degrees(sequence)
        self.__pose[3:6] = sequence

    def apply_delta(self, pose_delta, gripper_delta=0):
        self._verify_sequence_valid(sequence=pose_delta, shape=(6,))
        self.__pose[:3] += np.array(pose_delta)[:3]
        rot = Rotation.from_euler(
            "XYZ", pose_delta[3:6], degrees=True
        ) * Rotation.from_euler("XYZ", self.rpy, degrees=True)
        self.__pose[3:6] = rot.as_euler("XYZ", degrees=True)
        self.__gripper_position += gripper_delta

    def to_tuple(self):
        return tuple(val for val in np.append(self.__pose, self.__gripper_position))

    def to_np(self):
        return self.__pose

    def to_serializable(self):
        return self.to_tuple()

    def __len__(self):
        return 6

    def __str__(self) -> str:
        return f"XYZ: ({self.x},{self.y},{self.z}); RPY: ({self.roll},{self.pitch},{self.yaw})"

    def inverse_kinematics_math_based(
        self, robot_urdf, current_actuator_positions, align_orientation=False
    ):
        robot = ikpy.chain.Chain.from_urdf_file(
            robot_urdf, active_links_mask=[False] + [True] * 6 + [False]
        )
        initial_position = np.radians(
            np.concatenate([[0], current_actuator_positions, [0]])
        )
        if align_orientation:
            target_orientation = Rotation.from_euler(
                "XYZ", self.rpy, degrees=True
            ).as_matrix()
            orientation_mode = "all"
        else:
            target_orientation = [0, 0, 1]  # unit vector of x axis
            orientation_mode = "X"
        try:
            joint_positions = robot.inverse_kinematics(
                target_position=self.xyz,
                initial_position=initial_position,
                target_orientation=target_orientation,
                orientation_mode=orientation_mode,
            )
        except Exception as e:
            logging.warning(f"Skipped IK, exception: {e}")
            return None
        # logging.info(robot.forward_kinematics(joint_positions))
        joint_positions = joint_positions[1:7] * 180.0 / np.pi
        # logging.error(f"target_pose {target_pose} joint_positions {joint_positions}")
        actuator_positions = np.append(
            joint_positions, self.gripper_position
        )  # [1:] to remove base_link and gripper link
        return actuator_positions

    @staticmethod
    # Credits to ChatGPT. When asked:
    # "use scipy, write code to transform a positional and orientational pose vector of shape (6,), which contains [x,y,z,roll,pitch,yaw], named input_pose, where roll pitch yaw are already in degrees, convert it into an affine matrix of (4,4) named affine_matrix"
    # Aside from mistaken intrinsic to 'xyz', instead of 'XYZ', its response was copy pasted as-is:
    def pose_to_affine(input_pose):
        # Extract the translation and rotation values from the pose vector
        tx, ty, tz, roll, pitch, yaw = input_pose

        # Compute the rotation matrix using the given Euler angles in degrees
        r = Rotation.from_euler(
            "XYZ", [np.radians(roll), np.radians(pitch), np.radians(yaw)], degrees=False
        )
        R = r.as_matrix()

        # Construct the affine matrix using the translation and rotation values
        affine_matrix = np.eye(4)
        affine_matrix[:3, :3] = R
        affine_matrix[:3, 3] = [tx, ty, tz]

        return affine_matrix

    def inverse_kinematics_ml_based(self, current_actuator_positions, current_pose):
        current_actuator_positions = np.array(
            current_actuator_positions, dtype=np.float32
        )
        current_actuator_positions_features = ActuatorPositions.normalize_to_relu6(
            sequence=current_actuator_positions
        )
        current_pose = np.array(current_pose, dtype=np.float32)
        target_pose = np.array(self.pose[:6], dtype=np.float32)
        features = np.concatenate(
            [current_actuator_positions_features, current_pose, target_pose]
        )
        inferenced_actuator_positions = (
            Singletons.get_inverse_kinematics_model().inference_one(features=features)
        )
        inferenced_actuator_positions = ActuatorPositions.denormalize_relu6(
            sequence=inferenced_actuator_positions
        )
        return np.append(inferenced_actuator_positions, self.gripper_position)


class Trajectory:
    def __init__(self, trajectory=None):
        if trajectory is None:
            self.__trajectory: List[Tuple(float, Position)] = []
        else:
            self.__trajectory = list(trajectory)
        self.__dim = None

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

    def _append_vector(self, timestamp: float, vector: collections.abc.Sequence):
        if len(self.__trajectory) > 0 and timestamp <= self.__trajectory[-1][0]:
            return
        if self.__dim is None:
            self.__dim = len(vector)
        else:
            assert self.__dim == len(vector), f"{self.__dim} != {len(vector)}"
        self.__trajectory.append(tuple((timestamp, vector)))

    @property
    def trajectory(self):
        return self.__trajectory

    def __set_trajectory(self, trajectory):
        self.__trajectory = sorted(trajectory)

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


class TrajectoryActuatorPositions(Trajectory):
    def append(self, timestamp: float, positions: ActuatorPositions):
        assert isinstance(positions, ActuatorPositions)
        super()._append_vector(timestamp=timestamp, vector=positions)

    @property
    def starting_positions(self):
        if self.length > 0:
            return self.trajectory[0][1]
        return None

    def forward_kinematics(self):
        endeffector_trajectory = TrajectoryEndeffectorPose(
            starting_positions=self.starting_positions
        )
        for timestamp, positions in self.trajectory:
            positions: ActuatorPositions
            pose = positions.forward_kinematics_ml_based()
            endeffector_trajectory.append(
                timestamp=timestamp,
                pose=EndeffectorPose(
                    pose=pose, gripper_position=positions.gripper_position
                ),
            )
        return endeffector_trajectory


class TrajectoryEndeffectorPose(Trajectory):
    def __init__(self, starting_positions: ActuatorPositions = None, trajectory=None):
        super().__init__(trajectory)
        self.starting_positions = starting_positions

    def append(self, timestamp: float, pose: EndeffectorPose):
        assert isinstance(pose, EndeffectorPose)
        super()._append_vector(timestamp=timestamp, vector=pose)

    def inverse_kinematics(self, starting_positions: ActuatorPositions = None):
        if starting_positions is None:
            # a starting_positions is needed to determine the optimal path
            starting_positions = self.starting_positions
        assert starting_positions is not None

        actuator_positions_trajectory = TrajectoryActuatorPositions()
        current_positions = np.array(starting_positions.positions)
        for index in range(1, len(self.trajectory)):
            timestamp, new_pose = self.trajectory[index]
            _, previous_pose = self.trajectory[index - 1]
            new_pose: EndeffectorPose
            previous_pose: EndeffectorPose
            new_positions = new_pose.inverse_kinematics_math_based(
                current_actuator_positions=current_positions[:-1],
                current_pose=previous_pose.pose,
            )
            if new_positions is None:
                continue
            actuator_positions_trajectory.append(
                timestamp=timestamp,
                positions=ActuatorPositions(positions=new_positions),
            )
            current_positions = new_positions
        return actuator_positions_trajectory
