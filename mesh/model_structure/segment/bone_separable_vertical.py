# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


from mesh.configuration import segment_configuration

from . import bone_separable


# Place two sections of a bone vertically, side by side
# Only used for creating figures
class BoneSeparableVertical(bone_separable.BoneSeparable):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        separable_bone_pitch_only: bool = False,
    ):
        self.separable_bone_pitch_only = separable_bone_pitch_only
        # when printing vertically, we don't need side opening
        super().__init__(segment_configs=segment_configs, name=name)

    def printable_mesh(self, place_pitch_horizontally=True):
        roll, pitch = self.get_connected_roll_pitch_with_triangle_holder()
        roll = roll.translate(
            (0, 0, -self.segment_configs.structural.PitchCenterLocationZ)
        )
        pitch = pitch.translate(
            (0, 0, -self.segment_configs.structural.PitchCenterLocationZ)
        )
        roll = roll.rotate((0, 0, 0), (1, 0, 0), 180)

        offset_y = (
            -self.segment_configs.structural.JointLength
            + self.segment_configs.structural.BoneHolderRadius
        )
        offset_z = -self.segment_configs.structural.PitchBoneMeshNetLength
        roll = roll.translate((0, offset_y, offset_z))
        if place_pitch_horizontally:
            pitch = pitch.rotate((0, 0, 0), (0, 0, 1), -90)
            # for now, slicer would add support material to nut socket+space if nut facing downwards
            # so we are rotating nut side up
            # although preferably we hope nut side down, so it's easier to remove material in horn socket
            pitch = pitch.rotate((0, 0, 0), (1, 0, 0), -90)
            offset_z = self.segment_configs.structural.BoneHolderRadius
        else:
            pitch = pitch.rotate((0, 0, 0), (0, 0, 1), 90)
            pitch = pitch.rotate((0, 0, 0), (1, 0, 0), 180)
            offset_z = self.segment_configs.structural.BoneRadius
        pitch = pitch.translate((0, 0, offset_z))
        if (
            self.separable_bone_pitch_only
            or self.segment_configs.separable_bone_pitch_only
        ):
            bone = pitch
        else:
            bone = roll.add(pitch)
        return bone
