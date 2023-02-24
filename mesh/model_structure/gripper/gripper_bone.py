# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import abstractmethod

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh
from mesh.model_structure.segment import bone_separable
from mesh.model_structure.segment.abstractions import bone_abstract


# base class of a gripper attached to a separable bone, can be connected to a pitch bone
class GripperBone(bone_separable.BoneSeparable):
    @property
    def if_generate_motor_space_and_roll_holder(self):
        return False

    def __separable_bone_mesh__(self):
        bone, _ = self.get_connected_roll_pitch_with_triangle_holder()
        return bone

    @abstractmethod
    def _gripper_(self):
        pass

    @abstractmethod
    def _bone_cut_length_(self):
        # make sure to cut enough so the gripper head is connected
        # but not too far to compromise bone's structure, e.g. the holder socket
        pass

    def model_mesh(self, add_surface_give=False):
        gripper, gripper_rotation_space = self._gripper_()
        bone = self.__separable_bone_mesh__()
        bone = bone.rotate((0, 0, 0), (0, 0, 1), 90)
        bone = bone.translate(
            (
                0,
                0,
                self.segment_configs.structural.PitchBoneMeshNetLength
                - self.segment_configs.structural.PitchCenterLocationZ,
            )
        )
        # now the hinge-side edge of the bone is re-centered to origin,
        # with bone extruding towards -Z

        bone_length = self.segment_configs.structural.RollBoneMeshNetLength
        bone = bone.translate((0, 0, bone_length))
        # now the motor-side edge of the bone is re-centered to origin,
        # with bone extruding towards +Z

        bone = bone.rotate((0, 0, 0), (1, 0, 0), 90)
        # now re-centered to origin with bone extrude towards +Y

        diameter = self.segment_configs.structural.BoneHolderRadius * 2
        cut_length = self._bone_cut_length_()
        cut_box = (
            cq.Workplane(self.workplane_tertiary)
            .rect(diameter, diameter)
            .extrude(cut_length)
        )
        bone = bone.cut(cut_box).translate((0, cut_length, 0))

        mesh = gripper.add(bone).cut(gripper_rotation_space)
        return mesh

    def space_mesh(self):
        return None

    def urdf_mesh(self):
        roll = self.model_mesh(add_surface_give=True)
        # affine transform to print up straight, for easier removal of supporting materials
        roll = roll.translate(
            (
                0,
                self.segment_configs.structural.RollBoneMeshNetLength
                + self.segment_configs.structural.PitchBoneMeshNetLength
                - self._bone_cut_length_(),
                0,
            )
        ).rotate((0, 0, 0), (1, 0, 0), -90)
        _, pitch = self._rearranged_connected_roll_pitch_()
        bone = roll.add(pitch)
        bone = bone.rotate((0, 0, 0), (1, 0, 0), 180)
        return bone
