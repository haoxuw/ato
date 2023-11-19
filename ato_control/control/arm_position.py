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
from datetime import datetime

import numpy as np
import pinocchio
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
        ), f"{np.array(sequence).shape} != {shape}: {sequence}"
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
        self,
        actuator_positions=None,
        joint_positions=None,
        gripper_position=None,
        expected_pose=None,
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
        self.expected_pose = expected_pose

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

    # todo move to a robot_attributes singleton
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

    def forward_kinematics_ml_based(self):
        joint_positions = np.array(self.joint_positions, dtype=np.float32)
        features = ActuatorPositions.normalize_to_relu6(sequence=joint_positions)
        return Singletons.get_forward_kinematics_model().inference_one(
            features=features
        )

    @staticmethod
    def convert_to_pinocchio_configuration(joint_positions):
        # [0] as the dummy_object_link, because:
        # in the urdf, each link is represented by its anchoring position
        # therefore the endeffector's coordinate is located on the last joint
        # to model the position that holds the object, we added a dummy_object_link
        joint_configuration = np.radians(np.array(list(joint_positions) + [0]))
        return np.matrix(joint_configuration).T

    @staticmethod
    def forward_kinematics_pinocchio_based(
        robot, joint_config_radians, return_se3_object=False, log_debug=False
    ):
        pinocchio.forwardKinematics(
            robot.model, robot.data, joint_config_radians
        )  # updated in robot.data.oMi internally

        # pinocchio's naming convention:
        # oMi is origin to M (i.e. end effector) transformation, (i)nferred from joint_configuration
        # oMdes is origin to M (i.e. end effector) transformation, (des)ired

        oMdes = robot.data.oMi[-1]
        if return_se3_object:
            # return a pinocchio.SE3 object
            solved_pose = oMdes.copy()
        else:
            xyz = oMdes.translation
            rotation_current = Rotation.from_matrix(oMdes.rotation)
            rpy = rotation_current.as_euler("XYZ", degrees=True)
            solved_pose = np.concatenate([xyz, rpy])
        return solved_pose

    # consider: refactor to static method
    # support multiple arm configurations, by
    # creating a arm_configuration object: {actuator_indices_mapping, gripper_index_mapping, arm_segment_lengths}
    @classmethod
    def forward_kinematics_math_based(cls, joint_positions):
        current_time = datetime.now()
        assert cls.actuator_indices_mapping is not None, cls.actuator_indices_mapping
        assert cls.arm_segment_lengths is not None, cls.arm_segment_lengths
        joint_positions = np.array(joint_positions)
        rotation_sequence = np.array(
            [
                (
                    joint_positions[segment_index[1]],
                    0,
                    joint_positions[segment_index[0]],
                )
                for segment_index in cls.actuator_indices_mapping
            ]
        )

        assert (
            rotation_sequence.shape[0] == cls.arm_segment_lengths.shape[0]
        ), f"{rotation_sequence.shape[0]} == {cls.arm_segment_lengths.shape[0]}"

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
            rotation_sequence, cls.arm_segment_lengths
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
            + np.array([0, 0, cls.gripper_length]) @ body_frame_ref
        )
        endeffector_reference_frame = body_frame_ref
        endeffector_rpy = Rotation.align_vectors(a=body_frame_ref, b=global_frame)[
            0
        ]  # body_frame_ref is body observed in the initial global frame
        endeffector_rpy_intrinsic = endeffector_rpy.as_euler("XYZ", degrees=True)
        endeffector_rpy_extrinsic = endeffector_rpy.as_euler("xyz", degrees=True)
        logging.debug(
            f"forward_kinematics_math_based elapsed time: {(datetime.now() - current_time).total_seconds() * 1000.0}ms"
        )
        pose_detail = {
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
        return RobotPoseDetail(**pose_detail)


class RobotPoseDetail:
    def __init__(
        self,
        segments_xyz,
        frontend_pose_intrinsic,
        segments_reference_frames,
        endeffector_pose_intrinsic,
        endeffector_pose_extrinsic,
        endeffector_reference_frame,
    ) -> None:
        self.segments_xyz = segments_xyz
        self.frontend_pose_intrinsic = frontend_pose_intrinsic
        self.segments_reference_frames = segments_reference_frames
        self.endeffector_pose_intrinsic = endeffector_pose_intrinsic
        self.endeffector_pose_extrinsic = endeffector_pose_extrinsic
        self.endeffector_reference_frame = endeffector_reference_frame


class EndeffectorPose(ArmPosition):
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

    # todo maybe try move towards
    # try solve absolute position, rather than delta

    @staticmethod
    def inverse_kinematics_pinocchio_based(
        robot: pinocchio.robot_wrapper.RobotWrapper,
        target_pose: ActuatorPositions,
        time_delta_ms: int,
        initial_joint_positions,  # excludes gripper position
        gripper_position=None,
        # todo: what's the best value for the following parameters?
        damp=1e-2,
        eps=2,
        delta_t=0.1,
        max_try=100,
    ) -> ActuatorPositions:
        prior = ActuatorPositions.convert_to_pinocchio_configuration(
            joint_positions=initial_joint_positions
        )
        ik_solved_positions = None
        current_time = datetime.now()
        joint_id = len(initial_joint_positions) + 1  # id of the dummy_object_link

        for index in range(max_try):
            oMdes = ActuatorPositions.forward_kinematics_pinocchio_based(
                robot=robot,
                joint_config_radians=prior,
                return_se3_object=True,
            )

            oMdes.translation = target_pose.xyz

            rotation_delta = Rotation.from_euler("XYZ", target_pose.rpy, degrees=True)
            rotation_current = Rotation.from_matrix(oMdes.rotation)
            rotation_target: Rotation = rotation_delta * rotation_current
            rotation_matrix = pinocchio.Jlog3(rotation_target.as_matrix())
            oMdes.rotation = rotation_matrix
            current_rpy = rotation_current.as_euler("XYZ", degrees=True)
            logging.debug(
                f"curren_pose.xyz: {oMdes.translation}, current_pose.rpy: {current_rpy}"
            )
            logging.debug(
                f"target_pose.xyz: {target_pose.xyz}, target_pose.rpy: {target_pose.rpy}"
            )
            logging.debug(
                f"rpy_delta: {np.array(target_pose.rpy) - np.array(current_rpy)}"
            )

            dMi = oMdes.actInv(
                robot.data.oMi[-1]
            )  # find transformation between two frames
            err = pinocchio.log(dMi).vector
            weights = (1, 1, 1, 0.05, 0.5, 0.5)  # (wx, wy, wz, wroll, wpitch, wyaw)
            err_norm = np.linalg.norm(np.multiply(err, weights))

            logging.debug(f"Solver error: {err}")
            logging.info(f"Solver err_norm: {err_norm}")
            # todo: a known issue where xyz and rpy has different scale
            # use different eps for xyz and rpy
            if err_norm < eps:
                prior = np.array(prior)
                prior = prior.reshape(-1)
                joint_positions = np.degrees(prior)[:-1]  # remove the dummy_object_link
                ik_solved_positions = ActuatorPositions(
                    joint_positions=joint_positions,
                    gripper_position=gripper_position,
                    expected_pose=target_pose,
                )
                logging.info(
                    f"pinocchio ik_solved_positions: {ik_solved_positions}, tried {index} times"
                )
                break
            else:
                jointJacobian = pinocchio.computeJointJacobian(
                    robot.model, robot.data, prior, joint_id
                )  # calculate jacobian
                velocity = -jointJacobian.T.dot(
                    np.linalg.solve(
                        jointJacobian.dot(jointJacobian.T) + damp * np.eye(6), err
                    )
                )  # calculate velocity in configuration space

                prior = pinocchio.integrate(
                    robot.model,
                    prior,
                    velocity * time_delta_ms * delta_t,
                )

        if ik_solved_positions is None:
            logging.warning(
                f"pinocchio inverse kinematics failed to converge after {max_try} tries"
            )
        logging.debug(
            f"inverse_kinematics_pinocchio_based elapsed time: {(datetime.now() - current_time).total_seconds() * 1000.0}ms"
        )
        print(ik_solved_positions)
        return ik_solved_positions

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

    @staticmethod
    def to_fix_point(float_vector):
        return tuple(round(float(val), 2) for val in float_vector)
