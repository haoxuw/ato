# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh


class BonePitchAbstract(cq_mesh.CqMesh):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def model_mesh(self, add_surface_give=False):
        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0

        pitch_wheel = (
            cq.Workplane(self.workplane_secondary)
            .circle(self.segment_configs.structural.BoneRadius + surface_give / 2)
            .extrude(self.segment_configs.structural.BoneHolderRadius + surface_give)
        )
        pitch_wheel = pitch_wheel.add(pitch_wheel.mirror(self.workplane_secondary))

        pitch_bone = (
            cq.Workplane(self.workplane_primary)
            .circle(self.segment_configs.structural.BoneRadius + surface_give / 2)
            .extrude(
                -(self.segment_configs.structural.PitchBoneMeshNetLength + surface_give)
            )
        )
        pitch_bone = pitch_bone.add(pitch_wheel)

        pitch_offset = (0, 0, self.segment_configs.structural.PitchCenterLocationZ)
        pitch_bone = pitch_bone.translate(pitch_offset)

        return pitch_bone

    def space_mesh(self):
        mesh = self.model_mesh(add_surface_give=True)
        pitch_wheel_space_radius = (
            self.segment_configs.structural.BoneRadius
            + self.segment_configs.structural.SurfaceGive
        )
        bone_pitch_space = (
            cq.Workplane(self.workplane_secondary)
            .polyline(
                [
                    (0, pitch_wheel_space_radius),
                    (
                        self.segment_configs.structural.BoneLength,
                        self.segment_configs.structural.BoneLength
                        + pitch_wheel_space_radius,
                    ),
                    (
                        self.segment_configs.structural.BoneLength,
                        -self.segment_configs.structural.BoneLength,
                    ),
                    (0, -self.segment_configs.structural.BoneLength),
                ]
            )
            .mirrorY()
            .extrude(pitch_wheel_space_radius * 2)
            .translate(
                (
                    -pitch_wheel_space_radius,
                    0,
                    0,
                )
            )
        )
        pitch_offset = (0, 0, self.segment_configs.structural.PitchCenterLocationZ)
        bone_pitch_space = bone_pitch_space.translate(pitch_offset)

        return mesh.add(bone_pitch_space)


class BoneRollAbstract(cq_mesh.CqMesh):
    def model_mesh(self, add_surface_give=False):
        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0

        roll_bone = (
            cq.Workplane(self.workplane_primary)
            .circle(self.segment_configs.structural.BoneRadius + surface_give / 2)
            .extrude(self.segment_configs.structural.RollBoneMeshNetLength)
            .translate((0, 0, self.segment_configs.structural.MotorTopLocation))
        )

        if (
            self.segment_configs.structural.RollBoneHolderType
            == segment_configuration.RollBoneHolderType.SPHERE
        ):
            roll_bone_holder = (
                cq.Workplane(self.workplane_primary)
                .sphere(
                    radius=self.segment_configs.structural.BoneHolderRadius
                    + surface_give
                )
                .translate((0, 0, self.segment_configs.structural.BoneHolderRadius))
            )
        elif (
            self.segment_configs.structural.RollBoneHolderType
            == segment_configuration.RollBoneHolderType.CYLINDER
        ):
            roll_bone_holder = (
                cq.Workplane(self.workplane_primary)
                .circle(
                    self.segment_configs.structural.RollCylinderHolderDepth
                    + self.segment_configs.structural.BoneRadius
                    # special case, extra space for rotation -- the holder is difficult to print perfectly
                    + surface_give * 1.5
                )
                .extrude(
                    self.segment_configs.structural.RollCylinderHolderBreadth
                    + surface_give
                )
            )
        else:
            roll_bone_holder = None

        if roll_bone_holder is not None:
            roll_bone_holder = roll_bone_holder.translate(
                (
                    0,
                    0,
                    self.segment_configs.structural.MotorTopLocation
                    + self.segment_configs.structural.RollHolderOffsetZ,
                )
            )
            roll_bone = roll_bone.add(roll_bone_holder)

        return roll_bone

    def space_mesh(self):
        return self.model_mesh(add_surface_give=True)


class BoneAbstract(cq_mesh.CqMesh):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        allocate_both_bones_as_whole=False,
        allocate_pitch=True,
        allocate_roll=True,
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        # given each joint would be connected to 2 bones
        # if not allocate_both_bones_as_whole, we only allocate roll half of the bone on roll side,
        # and pitch half of the bone on pitch side. If True, then both bone would be allocated as whole
        # this is mainly to facilitate cutting for sockets
        self.allocate_both_bones_as_whole = allocate_both_bones_as_whole

        self.allocate_pitch = allocate_pitch
        self.allocate_roll = allocate_roll

    @property
    def if_generate_motor_space_and_roll_holder(self):
        return True

    @property
    def bone_roll_class(self):
        return BoneRollAbstract

    @property
    def bone_pitch_class(self):
        return BonePitchAbstract

    def __merge_roll_with_pitch(self, roll, pitch):
        bone_whole = cq.Workplane(self.workplane_primary)
        if self.allocate_pitch:
            bone_whole = bone_whole.add(pitch)
        if self.allocate_roll:
            bone_whole = bone_whole.add(roll)
        return bone_whole

    def _get_roll_pitch_z_distance(self):
        return (
            self.segment_configs.structural.BoneLength
            + self.segment_configs.structural.JointRadius * 2
            + self.segment_configs.structural.MotorTopLocation
            - self.segment_configs.structural.PitchCenterLocationZ
        )

    def __add_complementary(self, roll, pitch):
        offset_z = self._get_roll_pitch_z_distance()
        roll_whole = roll.add(pitch.translate((0, 0, offset_z)))
        pitch_whole = pitch.add(roll.translate((0, 0, -offset_z)))
        return roll_whole, pitch_whole

    def _get_roll_pitch_model_mesh(self, add_surface_give):
        roll = self.bone_roll_class(segment_configs=self.segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )
        pitch = self.bone_pitch_class(segment_configs=self.segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )

        return roll, pitch

    def model_mesh(self, add_surface_give=False):
        roll, pitch = self._get_roll_pitch_model_mesh(add_surface_give=add_surface_give)
        if self.allocate_both_bones_as_whole:
            roll, pitch = self.__add_complementary(roll=roll, pitch=pitch)
        return self.__merge_roll_with_pitch(roll=roll, pitch=pitch)

    def space_mesh(self):
        roll = self.bone_roll_class(segment_configs=self.segment_configs).space_mesh()
        pitch = self.bone_pitch_class(segment_configs=self.segment_configs).space_mesh()

        if self.allocate_both_bones_as_whole:
            roll, pitch = self.__add_complementary(roll=roll, pitch=pitch)

        return self.__merge_roll_with_pitch(roll=roll, pitch=pitch)

    def _rearranged_connected_roll_pitch_(
        self, horn_side_downwards=False, add_surface_give=False
    ):
        roll, pitch = self._get_roll_pitch_model_mesh(add_surface_give=add_surface_give)
        # rotate so that when horn side downwards, nut sockets are on top
        if horn_side_downwards:
            pitch = pitch.rotate((0, 0, 0), (0, 0, 1), 180)
        offset_z = self._get_roll_pitch_z_distance()
        roll = roll.translate((0, 0, -offset_z))
        return roll, pitch

    def _rearrange_to_primary_plane(self, roll, pitch):
        bone = roll.add(pitch)
        bone = bone.rotate((0, 0, 0), (1, 0, 0), 90)
        bone = bone.rotate((0, 0, 0), (0, 1, 0), -90)
        bone = bone.rotate((0, 0, 0), (0, 0, 1), 180)
        bone = bone.translate(
            (
                0,
                0,
                self.segment_configs.structural.BoneHolderRadius,
            )
        )
        return bone

    def printable_mesh(self):
        roll, pitch = self._rearranged_connected_roll_pitch_()
        bone = self._rearrange_to_primary_plane(roll=roll, pitch=pitch)
        return bone

    def urdf_mesh(self):
        roll, pitch = self._rearranged_connected_roll_pitch_()
        bone = roll.add(pitch)
        bone = bone.rotate((0, 0, 0), (1, 0, 0), 180)
        return bone
