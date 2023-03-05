# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq
from mesh.configuration import arm_configuration, segment_configuration
from mesh.cq_lib import cq_mesh
from mesh.model_structure.mechanical_parts import allocated_motors
from mesh.model_structure.segment import bone_augmented, joint_augmented


class Arm(cq_mesh.CqMesh):
    def __init__(
        self,
        arm_configs: arm_configuration.ArmConfigsBase,
        name: str = "",
        half_joint=False,
        include_motor_mesh=True,
    ):
        super().__init__(segment_configs=None, name=name)
        self.segments_configs = arm_configs.segments_configs
        self.half_joint = half_joint
        self.include_motor_mesh = include_motor_mesh

    def __segment_mesh(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        allocate_pitch: float = None,
    ):
        if self.half_joint:
            joint_mesh = joint_augmented.JointAugmentedPos(
                segment_configs=segment_configs
            ).model_mesh()
        else:
            joint_mesh = joint_augmented.JointAugmented(
                segment_configs=segment_configs
            ).model_mesh()
        if self.include_motor_mesh:
            joint_mesh = joint_mesh.add(
                allocated_motors.AllocatedMotors(
                    segment_configs=segment_configs
                ).model_mesh()
            )
        bone_roll = bone_augmented.BoneAugmented(
            segment_configs=segment_configs,
            allocate_both_bones_as_whole=True,
            allocate_pitch=False,
            allocate_roll=True,
        ).model_mesh()
        segment = joint_mesh.add(bone_roll)
        if allocate_pitch is not None:
            bone_pitch = bone_augmented.BoneAugmented(
                segment_configs=segment_configs,
                allocate_both_bones_as_whole=True,
                allocate_pitch=True,
                allocate_roll=False,
            ).model_mesh()
            bone_pitch = bone_pitch.rotate((0, 0, 0), (1, 0, 0), allocate_pitch)
            segment = segment.add(bone_pitch)
        # we constructed the joint with pitch center at origin, and roll side upwards,
        # so that most of the joint is has positive coordinate
        # to build an arm with pitch side downwards, we relocate
        segment = segment.rotate((0, 0, 0), (1, 0, 0), 180)
        return segment

    def printable_mesh(self):
        arm = cq.Workplane(self.workplane_primary)
        for index, segment_configs in enumerate(reversed(self.segments_configs)):
            if index != 0:
                arm = arm.rotate(
                    (0, 0, 0), (1, 0, 0), segment_configs.allocate_with_pitch_angle
                )
            segment = self.__segment_mesh(
                segment_configs=segment_configs,
                allocate_pitch=segment_configs.allocate_with_pitch_angle
                if index == 0
                else None,
            )
            arm = arm.add(segment)
            arm = arm.translate((0, 0, segment_configs.structural.SegmentLength))
        return arm
