# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import collections
import logging
import numbers
import pathlib
import sys
from abc import ABC, abstractmethod

import ikpy.chain
import numpy as np
from control.config_and_enums.controller_enums import SolverMode
from scipy.spatial.transform import Rotation

try:
    sys.path.append(f"{pathlib.Path(__file__).parent}/../../ato_learning/")
    # pylint: disable=import-error
    from learn_kinematics import forward_kinematics, inverse_kinematics
except Exception as import_e:
    logging.warning(f"Skipped importing the learning package, due to: {import_e}")


# there will be two ways to define arm position
# either by joint positions, or end effector pose
# those two are mapped by a many-to-one relation
# and converted via forward kinematics or inverse kinematics
class ArmPosition(ABC):
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


class ActuatorPositions(ArmPosition):
    # e.g. when actuator_indices_mapping = ((0,1), (4,5)), it means:
    # self.__joint_positions[0] is roll of the 1st segment
    # self.__joint_positions[1] is pitch of the 1st segment
    # self.__joint_positions[4] is roll of the 2nd segment
    # self.__joint_positions[5] is pitch of the 2nd segment
    actuator_indices_mapping = None

    # e.g. when arm_segment_lengths = (30, 60), it means:
    # total length of the 1st segment is 30, 2nd is 60
    arm_segment_lengths = None

    rotation_ranges = None

    def __init__(
        self, actuator_positions=None, joint_positions=None, gripper_position=None
    ) -> None:
        if actuator_positions is not None:
            assert joint_positions is None
            assert gripper_position is None
            self.set(sequence=actuator_positions[:-1])
            self.__gripper_position = actuator_positions[-1]
        else:
            assert joint_positions is not None
            self.set(sequence=joint_positions)
            self.__gripper_position = gripper_position

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
    def actuator_positions(self):
        return np.append(self.__joint_positions, self.__gripper_position)

    @property
    def joint_positions(self):
        return self.__joint_positions

    @property
    def gripper_position(self):
        return self.__gripper_position

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
            sequence=sequence, shape=(len(self.__class__.arm_segment_lengths) * 2,)
        )
        self.__joint_positions = np.array(sequence)

    def to_tuple(self):
        return tuple(val for val in self.actuator_positions)

    def to_np(self):
        return self.actuator_positions

    def to_serializable(self):
        return self.to_tuple()

    def __len__(self):
        return len(self.actuator_positions)

    def __str__(self) -> str:
        return f"@{self.actuator_positions}"

    def info_str(self) -> str:
        return f"actuator @{self.actuator_positions}" + (
            f" segment_actuator_indices: { self.__class__.actuator_indices_mapping} gripper_index_mapping: { self.__class__.gripper_index_mapping}"
            if self.__class__.actuator_indices_mapping is not None
            else ""
        )

    # we have 3 versions of forward_kinematics, to practice and learn, we implemented one version based on ml
    # and another base on math.
    # finally a feature rich implementation from the ikpy package
    def forward_kinematics_ml_based(self):
        joint_positions = np.array(self.joint_positions, dtype=np.float32)
        features = ActuatorPositions.normalize_to_relu6(sequence=joint_positions)
        return Singletons.get_forward_kinematics_model().inference_one(
            features=features
        )

    def forward_kinematics_math_based(self):
        assert (
            self.__class__.actuator_indices_mapping is not None
        ), self.__class__.actuator_indices_mapping
        assert (
            self.__class__.arm_segment_lengths is not None
        ), self.__class__.arm_segment_lengths
        joint_positions = np.array(self.__joint_positions)
        rotation_sequence = np.array(
            [
                (
                    joint_positions[segment_index[1]],
                    0,
                    joint_positions[segment_index[0]],
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
        body_frame_ref = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        current_frame_ref_origin = np.array((0, 0, 0), dtype="float")
        segments_xyz = [current_frame_ref_origin]  # base mount
        segments_reference_frames = []

        # in each iteration, we first rotate body_frame_ref, which track orientation of the next segment
        # then we rotate arm_segment vector to body_frame_ref by: arm_segment @ body_frame_ref
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
        endeffector_rpy = Rotation.align_vectors(a=body_frame_ref, b=global_frame)[
            0
        ]  # body_frame_ref is body observed in the initial global frame
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

    # returns EndeffectorPose
    def forward_kinematics_ikpy(self, robot_chain):
        estimated_ee_pose_matrix_via_fk = robot_chain.forward_kinematics(
            [
                0,
                *np.radians(self.__joint_positions),
                0,
            ]  # base_link + active_joints + dummy_gripper
        )
        xyz = estimated_ee_pose_matrix_via_fk[:3, 3]
        rpy = Rotation.from_matrix(estimated_ee_pose_matrix_via_fk[:3, :3]).as_euler(
            "XYZ", degrees=True
        )
        actual_pose_vector = np.concatenate([xyz, rpy])
        return EndeffectorPose(
            pose=actual_pose_vector, gripper_position=self.gripper_position
        )


class EndeffectorPose(ArmPosition):
    robot_chain = None

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
        rot = Rotation.from_euler("XYZ", self.rpy, degrees=True) * Rotation.from_euler(
            "XYZ", pose_delta[3:6], degrees=True
        )
        self.__pose[3:6] = rot.as_euler("XYZ", degrees=True)
        self.__gripper_position += gripper_delta
        return self

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

    @classmethod
    def set_robot_chain_filename(cls, urdf_filename):
        cls.urdf_filename = urdf_filename

    @staticmethod
    def get_ikpy_robot_chain():
        assert EndeffectorPose.urdf_filename is not None
        return ikpy.chain.Chain.from_urdf_file(
            urdf_file=EndeffectorPose.urdf_filename,
            active_links_mask=[False] + [True] * 6 + [False],  # todo 6
        )

    @classmethod
    def load_ik_cache(cls, file_path):
        try:
            with open(file_path, "rb") as fin:
                logging.info(f"loading ik_cache to {file_path}")
                cls.ik_caches = np.load(fin, allow_pickle=True).item()
                fin.close()
                return True
        except Exception as exp:
            logging.warning(f"Failed to load ik_cache @{file_path}, due to {exp}")
            return False

    @staticmethod
    def inverse_kinematics_ikpy(
        robot_chain: ikpy.chain.Chain,
        target_pose,
        solver_mode: SolverMode,
        initial_joint_positions=None,  # excludes gripper position
        skip_validation=False,
    ):
        xyz = target_pose.xyz
        rpy = target_pose.rpy
        if initial_joint_positions is None:
            initial_position = None  # unknown initial_position
        else:
            initial_position = np.radians(
                np.concatenate([[0], initial_joint_positions, [0]])
            )
        target_orientation = Rotation.from_euler("XYZ", rpy, degrees=True).as_matrix()

        # if align all, or only z or only pos
        if solver_mode == SolverMode.FORWARD:
            orientation_mode = "Z"
            target_orientation = [0, 1, 0]
        elif solver_mode == SolverMode.ALL:
            orientation_mode = "all"  # convention keyword of ikpy
            # target_orientation = target_orientation
        elif solver_mode == SolverMode.Z:
            # to align with unit vector of z axis -- because gripper was pointing upwards at installation
            orientation_mode = "Z"
            target_orientation = target_orientation[:, 2]
        elif solver_mode == SolverMode.NONE:
            orientation_mode = None
            target_orientation = None
        else:
            raise Exception(f"Unexpected solver_mode == {solver_mode}")

        try:
            joint_positions = robot_chain.inverse_kinematics(
                target_position=xyz,
                initial_position=initial_position,
                target_orientation=target_orientation,
                orientation_mode=orientation_mode,
            )
        except Exception as e:
            logging.warning(f"Skipped IK for {orientation_mode}, exception: {e}")
            return None

        joint_positions = (
            joint_positions[1:7] * 180.0 / np.pi
        )  # [1:7] to remove base_link and gripper link

        if not skip_validation:
            joint_positions, _ = EndeffectorPose.validate_ik_fk(
                robot_chain=robot_chain,
                intended_pose_vector=target_pose.pose,
                new_joint_positions_sent_to_hardware=joint_positions,
            )
        if joint_positions is None:
            return None
        else:
            return ActuatorPositions(
                joint_positions=joint_positions,
                gripper_position=target_pose.gripper_position,
            )

    @staticmethod
    def validate_ik_fk(
        robot_chain,
        intended_pose_vector,
        new_joint_positions_sent_to_hardware: collections.abc.Sequence,
        tolerance={
            "xyz_atol": 10,  # mm
            "rpy_atol": 3,  # degrees
        },
        use_ikpy=True,
    ):
        if use_ikpy:
            estimated_ee_pose_vector = (
                ActuatorPositions(joint_positions=new_joint_positions_sent_to_hardware)
                .forward_kinematics_ikpy(robot_chain=robot_chain)
                .pose
            )
        else:
            estimated_ee_pose = ActuatorPositions(
                joint_positions=new_joint_positions_sent_to_hardware
            ).forward_kinematics_math_based()
            estimated_ee_pose_vector = estimated_ee_pose["endeffector_pose_intrinsic"]

        xyz_validated = np.allclose(
            a=intended_pose_vector[:3],
            b=estimated_ee_pose_vector[:3],
            rtol=tolerance["xyz_atol"],
        )
        rpy_validated = np.allclose(
            a=intended_pose_vector[3:6],
            b=estimated_ee_pose_vector[3:6],
            rtol=tolerance["rpy_atol"],
        )
        if xyz_validated and rpy_validated:
            validated_positions = new_joint_positions_sent_to_hardware
        else:
            validated_positions = None
        return validated_positions, estimated_ee_pose_vector

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

    def inverse_kinematics_ml_based(self, initial_joint_positions, current_pose):
        initial_joint_positions = np.array(initial_joint_positions, dtype=np.float32)
        initial_joint_positions_features = ActuatorPositions.normalize_to_relu6(
            sequence=initial_joint_positions
        )
        current_pose = np.array(current_pose, dtype=np.float32)
        target_pose = np.array(self.pose[:6], dtype=np.float32)
        features = np.concatenate(
            [initial_joint_positions_features, current_pose, target_pose]
        )
        inferenced_actuator_positions = (
            Singletons.get_inverse_kinematics_model().inference_one(features=features)
        )
        inferenced_actuator_positions = ActuatorPositions.denormalize_relu6(
            sequence=inferenced_actuator_positions
        )
        return np.append(inferenced_actuator_positions, self.gripper_position)

    def inverse_kinematics_cached(self, orientation=SolverMode.FORWARD):
        if orientation == SolverMode.FORWARD:
            orientation = (-90, 0, 0)
            evaluation_unit = self.ik_caches["evaluation_unit"]
            initial_xyz = self.ik_caches[orientation]["initial_xyz"]
            initial_positions = self.ik_caches[orientation]["initial_positions"]
            ik_cache = self.ik_caches[orientation]["ik_cache"]
        else:
            raise Exception(f"Not supported yet {orientation}")

        xyz_relative = self.xyz - np.array(
            initial_xyz
        )  # recenter origin to initial_xyz
        rounding_delta = (
            xyz_relative - np.floor(xyz_relative / evaluation_unit) * evaluation_unit
        )
        rounded_down_xyz = self.xyz - rounding_delta
        total_weight = 0
        total_weighted_positions = np.zeros(len(initial_positions))
        for offset in (  # loop over corners of the cube around xyz
            (0, 0, 0),  # left back bottom
            (1, 0, 0),  # right back bottom
            (0, 1, 0),  # left front bottom
            (1, 1, 0),  # right front bottom
            (0, 0, 1),  # left back top
            (1, 0, 1),  # right back top
            (0, 1, 1),  # left front top
            (1, 1, 1),  # right front top
        ):
            xyz_key = EndeffectorPose.to_fix_point(
                rounded_down_xyz + np.array(offset) * evaluation_unit
            )
            if ik_cache.get(xyz_key, None) is None:
                continue
            distance = np.sum(np.square(self.xyz - np.array(xyz_key))) ** 0.5
            total_weighted_positions += ik_cache[xyz_key] * distance
            total_weight += distance

        if total_weight == 0:
            return None
        else:
            target_positions = total_weighted_positions / total_weight
            return ActuatorPositions(
                joint_positions=target_positions, gripper_position=self.gripper_position
            )

    @staticmethod
    def to_fix_point(float_vector):
        return tuple(round(float(val), 2) for val in float_vector)
