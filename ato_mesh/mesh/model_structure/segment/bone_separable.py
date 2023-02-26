# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import bolt
from mesh.model_structure.segment import bone_augmented


class BoneSeparable(bone_augmented.BoneAugmented):
    def __get_height_radius__(self):
        radius = self.segment_configs.structural.BoneRadius / 2
        height = radius * 4
        distance = radius / 2
        return radius, height, distance

    def __triangle_holder__(self, add_surface_give):
        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive / 2
        else:
            surface_give = 0

        radius, height, distance = self.__get_height_radius__()
        mesh = cq.Workplane(self.workplane_primary)
        offset_z = (
            self.segment_configs.structural.PitchBoneMeshNetLength
            - self.segment_configs.structural.PitchCenterLocationZ
        )
        for offsets in [
            (distance, 0, -offset_z),
            (-distance / 2**0.5, distance / 2**0.5, -offset_z),
            (-distance / 2**0.5, -distance / 2**0.5, -offset_z),
        ]:
            mesh = mesh.add(
                cq.Workplane(self.workplane_primary)
                .circle(radius + surface_give)
                .extrude(-height)
                .translate(offsets)
            )
        return mesh

    def get_connected_roll_pitch_with_triangle_holder(self):
        roll, pitch = self._rearranged_connected_roll_pitch_()
        holder = self.__triangle_holder__(add_surface_give=False)
        pitch = pitch.add(holder)
        holder_socket = self.__triangle_holder__(add_surface_give=True)
        roll = roll.cut(holder_socket)

        # a bolt to latch the two sections
        bone_bolt = bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=False,
            nut_offset_factor=0.9,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=2, BoltLength=40
            ),
        )
        bolt_socket = bone_bolt.model_mesh(add_surface_give=True)
        _, height, _ = self.__get_height_radius__()
        offset_z = (
            self.segment_configs.structural.PitchBoneMeshNetLength
            - self.segment_configs.structural.PitchCenterLocationZ
            + height / 2
        )
        bolt_socket = (
            bolt_socket.rotate((0, 0, 0), (1, 0, 0), 90)
            .rotate((0, 0, 0), (0, 0, 1), 90)
            .translate((self.segment_configs.structural.BoneRadius, 0, -offset_z))
        )
        roll = roll.cut(bolt_socket)
        pitch = pitch.cut(bolt_socket)
        return roll, pitch

    def model_mesh(self, add_surface_give=False):
        roll, pitch = self.get_connected_roll_pitch_with_triangle_holder()
        bone = roll.add(pitch)
        return bone

    def printable_mesh(self):
        roll, pitch = self.get_connected_roll_pitch_with_triangle_holder()

        _, height, _ = self.__get_height_radius__()
        pitch = pitch.translate((0, 0, height))
        bone = self._rearrange_to_primary_plane(roll=roll, pitch=pitch)
        return bone

    def urdf_mesh(self):
        return None
