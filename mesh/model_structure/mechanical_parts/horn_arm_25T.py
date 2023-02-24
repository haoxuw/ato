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


class HornArm25T(horn_base.HornBase):
    def __init__(self, segment_configs: segment_configuration.SegmentConfigs, **kwards):
        self.horn_configs: segment_configuration.HornArmConfigs25T = (
            segment_configuration.HornArmConfigs25T()
        )
        super().__init__(segment_configs=segment_configs, **kwards)
        self.arm_rotation_range = self.horn_configs.ArmRotationRange
        if self.arm_rotation_range is None:
            self.arm_rotation_range = (0, 180)

    @property
    def horn_bolt(self):
        return bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=True,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=3, BoltLength=32
            ),
            bolt_head_insert_space_length_override=0,
            bolt_length_override=30,
            nut_offset_factor=0.57,
            side_print_opening_length_ratio=0.33,
        )

    def __get_offset_height__(self, inclose_from_motor):
        if inclose_from_motor:
            horn_inner_offset = 0
            horn_height = (
                self.horn_configs.HornHeight + self.horn_configs.HornInnerOffset
            )
        else:
            horn_inner_offset = self.horn_configs.HornInnerOffset
            horn_height = self.horn_configs.HornHeight
        return horn_inner_offset, horn_height

    def __mesh_template__(self, add_surface_give, inclose_from_motor):

        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
            # for socket, we enclose the space between horn and motor
            horn_inner_offset = 0
        else:
            surface_give = 0
        horn_inner_offset, horn_height = self.__get_offset_height__(
            inclose_from_motor=inclose_from_motor
        )
        horn_shaft = (
            cq.Workplane(self.workplane_primary)
            .circle(self.motor_configs.ShaftRadius + surface_give / 2)
            .extrude(self.horn_configs.HornInnerOffset)
        )
        horn_mounting_circle = (
            cq.Workplane(self.workplane_primary)
            .circle(self.horn_configs.HornRadius + surface_give / 2)
            .extrude(horn_height)
            .translate((0, 0, horn_inner_offset))
        )
        horn_arm_trapezoid = (
            cq.Workplane(self.workplane_primary)
            .polyline(
                [
                    (0, 0),
                    (0, self.horn_configs.HornArmBottomWidth / 2 + surface_give / 2),
                    (
                        -(self.horn_configs.HornArmReach + surface_give / 2),
                        self.horn_configs.HornArmTopWidth / 2 + surface_give / 2,
                    ),
                    (-(self.horn_configs.HornArmReach + surface_give / 2), 0),
                ]
            )
            .mirrorX()
            .extrude(horn_height)
            .translate((0, 0, horn_inner_offset))
            .rotate(
                (0, 0, 0), (0, 0, 1), self.horn_configs.ArmRotationInitialAngle - 90
            )
        )
        horn_body = horn_arm_trapezoid.add(horn_mounting_circle)

        horn = horn_body.add(horn_shaft)
        return horn

    def __horn_bolt_space__(self):
        all_bolt_space = cq.Workplane(self.workplane_primary)
        for bolt_location_x in self.horn_configs.BoltLocations:
            bolt_obj = self.horn_bolt
            space_mesh = (
                bolt_obj.space_mesh()
                .rotate((0, 0, 0), (1, 0, 0), 180)
                .translate((-bolt_location_x, 0, self.horn_configs.HornInnerOffset))
                .rotate(
                    (0, 0, 0), (0, 0, 1), self.horn_configs.ArmRotationInitialAngle - 90
                )
            )
            all_bolt_space.add(space_mesh)

        return all_bolt_space

    def __fan_mesh_(self, radius, height, min_angle, max_angle):
        assert 0 <= min_angle < max_angle <= 180
        min_angle = min_angle * math.pi / 180
        max_angle = max_angle * math.pi / 180
        mid_angle = (min_angle + max_angle) / 2
        return (
            cq.Workplane(self.workplane_primary)
            .lineTo(-math.sin(max_angle) * radius, math.cos(max_angle) * radius)
            .threePointArc(
                (-math.sin(mid_angle) * radius, math.cos(mid_angle) * radius),
                (-math.sin(min_angle) * radius, math.cos(min_angle) * radius),
            )
            .close()
            .extrude(height)
        )

    def __horn_rotation_space__(
        self,
    ):
        horn_inner_offset, horn_height = self.__get_offset_height__(
            inclose_from_motor=True
        )
        surface_give = self.segment_configs.structural.SurfaceGive
        bolt_head_diam = self.horn_bolt.bolt_configs.BoltHeadDiam
        bolt_head_height = self.horn_bolt.bolt_configs.BoltHeadHeight
        radius = self.horn_configs.HornArmReach + surface_give / 2
        min_angle, max_angle = self.horn_configs.ArmRotationRange
        rotation_space = self.__fan_mesh_(
            radius=radius, height=horn_height, min_angle=min_angle, max_angle=max_angle
        ).translate((0, 0, horn_inner_offset))

        # bolt head is exposed between the horn arm and motor
        # when rotating the horn/shaft space is need to fit the head
        for bolt_location_x in self.horn_configs.BoltLocations:
            bolt_head_rotation_space = (
                cq.Workplane(self.workplane_primary)
                .add(
                    self.__fan_mesh_(
                        radius=bolt_location_x + (bolt_head_diam / 2 + surface_give),
                        height=-(bolt_head_height + surface_give),
                        min_angle=min_angle,
                        max_angle=max_angle,
                    )
                )
                .cut(
                    self.__fan_mesh_(
                        radius=bolt_location_x - (bolt_head_diam / 2 + surface_give),
                        height=-(bolt_head_height + surface_give),
                        min_angle=min_angle,
                        max_angle=max_angle,
                    )
                )
                .translate((0, 0, self.horn_configs.HornInnerOffset))
            )
            rotation_space = rotation_space.add(bolt_head_rotation_space)
        return rotation_space
