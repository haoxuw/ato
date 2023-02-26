# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import math

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import bolt, horn_base


class HornDiskDs5160(horn_base.HornBase):
    def __init__(self, segment_configs: segment_configuration.SegmentConfigs, **kwards):
        # it seems vs code tracing only works after adding the type hint
        self.horn_configs: segment_configuration.HornDiskConfigsDs5160 = (
            segment_configuration.HornDiskConfigsDs5160()
        )
        super().__init__(segment_configs=segment_configs, **kwards)

    @property
    def horn_bolt(self):
        return bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=self.install_bolt_from_side_instead_along,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=2, BoltLength=self.segment_configs.motor_horn.HornBoltLength
            ),
            side_print_opening_length_ratio=0.318 if self.side_print_opening else 0,
        )

    def __get_bolt_distances__(self):
        # the disk horn has bolt socket like this
        # --XX--
        # -X--X-
        # -X--X-
        # --XX--
        radius = self.horn_configs.HornBoltRadius
        sides = 8
        angle = math.pi * 2 / sides
        long = math.cos(angle / 2) * radius
        short = math.sin(angle / 2) * radius
        # e.g. the top two XX would be located at (-short, long), (short, long)
        return long, short

    def __mesh_template__(self, add_surface_give=False, inclose_from_motor=True):

        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0
        if inclose_from_motor:
            horn_body = (
                cq.Workplane(self.workplane_primary)
                .circle(self.horn_configs.HornRadius + surface_give / 2)
                .extrude(
                    self.horn_configs.HornHeight + self.horn_configs.HornInnerOffset
                )
            )
        else:
            horn_body = (
                cq.Workplane(self.workplane_primary)
                .circle(self.horn_configs.HornRadius + surface_give / 2)
                .extrude(self.horn_configs.HornHeight)
                .translate((0, 0, self.horn_configs.HornInnerOffset))
            )
            horn_bottom_plane = (
                cq.Workplane(self.workplane_primary)
                .circle(self.horn_configs.HornBottomPlaneRadius + surface_give / 2)
                .extrude(self.horn_configs.HornBottomPlaneHeight)
            )
            horn_body = horn_body.add(horn_bottom_plane)
        horn_shaft = (
            cq.Workplane(self.workplane_primary)
            .circle(self.motor_configs.ShaftRadius + surface_give / 2)
            .extrude(self.horn_configs.HornInnerOffset)
        )
        horn_cap = (
            cq.Workplane(self.workplane_primary)
            .circle(self.horn_configs.HornCapRadius + surface_give / 2)
            .extrude(self.horn_configs.HornHeight + self.horn_configs.HornCapHeight)
            .translate((0, 0, self.horn_configs.HornInnerOffset))
        )

        # fine modeling the DS5160 servo
        horn = horn_body.add(horn_shaft).add(horn_cap)
        return horn

    def __horn_bolt_space__(self, less_bolts=False):
        space = cq.Workplane(self.workplane_primary)
        long, short = self.__get_bolt_distances__()
        if less_bolts:
            four_bolts_without_the_middle_four = [
                # (-long, -short),
                # (-long, short),
                (-short, -long),
                (-short, long),
                # (long, -short),
                # (long, short),
                (short, -long),
                (short, long),
            ]
            bolt_offsets = four_bolts_without_the_middle_four
        else:
            eight_bolts = [
                (-long, -short),
                (-long, short),
                (-short, -long),
                (-short, long),
                (long, -short),
                (long, short),
                (short, -long),
                (short, long),
            ]
            bolt_offsets = eight_bolts

        for offset_x, offset_y in bolt_offsets:
            bolt_space = self.__relocate_bolt_on_horn__(
                self.horn_bolt.space_mesh(),
                (offset_x, offset_y, self.horn_configs.HornInnerOffset),
            )
            space = space.add(bolt_space)
        return space
