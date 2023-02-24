# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import copy

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import allocated_motors
from mesh.model_structure.segment.abstractions import bone_abstract


class BoneRoll(bone_abstract.BoneRollAbstract):
    def __create_lubricant_opening(self, positive: bool):
        # lets match the exterior sides to be cool
        n_sides = max(self.segment_configs.joint_polygon_sides, 4)
        # the following are rather arbitrary
        scaler = 1.5
        negator = 1 if positive else -1
        diameter = self.segment_configs.structural.RollLubricantOpeningWidth
        lubricant_opening = (
            cq.Workplane(self.workplane_tertiary)
            .pushPoints([(0, 0)])
            .polygon(
                nSides=n_sides,
                diameter=diameter * scaler,
            )
            .workplane(
                offset=self.segment_configs.structural.BoneHolderRadius
                * scaler
                * negator
            )
            .pushPoints([(0, 0)])
            .polygon(
                nSides=n_sides,
                diameter=diameter / scaler,
            )
            .loft()
            .translate(
                (
                    0,
                    0,
                    self.segment_configs.structural.MotorTopLocation
                    + self.segment_configs.structural.RollHolderOffsetZ
                    + diameter,
                )
            )
        )
        return lubricant_opening

    def space_mesh(self):
        roll_bone = self.model_mesh(add_surface_give=True)
        roll_bone = roll_bone.add(self.__create_lubricant_opening(positive=True))
        roll_bone = roll_bone.add(self.__create_lubricant_opening(positive=False))
        return roll_bone


class BonePitch(bone_abstract.BonePitchAbstract):
    pass


class BoneAugmented(bone_abstract.BoneAbstract):
    @property
    def bone_roll_class(self):
        return BoneRoll

    @property
    def bone_pitch_class(self):
        return BonePitch

    def _get_roll_pitch_model_mesh(self, add_surface_give):
        segment_configs = self.segment_configs
        if not self.if_generate_motor_space_and_roll_holder:
            segment_configs = copy.deepcopy(self.segment_configs)
            segment_configs.structural.RollBoneHolderType = (
                segment_configuration.RollBoneHolderType.NONE
            )

        roll = self.bone_roll_class(segment_configs=segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )
        pitch = self.bone_pitch_class(segment_configs=self.segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )

        if self.if_generate_motor_space_and_roll_holder:
            roll = roll.cut(
                allocated_motors.AllocatedMotors(
                    segment_configs=self.segment_configs,
                    allocate_pitch=False,
                    allocate_roll=True,
                ).space_mesh()
            )
            pitch = pitch.cut(
                allocated_motors.AllocatedMotors(
                    segment_configs=self.segment_configs,
                    allocate_pitch=True,
                    allocate_roll=False,
                ).space_mesh()
            )

        return roll, pitch

    def urdf_mesh(self):
        roll, pitch = self._rearranged_connected_roll_pitch_()
        bone = roll.add(pitch)
        bone = bone.rotate((0, 0, 0), (1, 0, 0), 180)
        return bone
