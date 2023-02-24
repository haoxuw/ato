# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import motor_base


class StepperMotor(motor_base.MotorBase):
    def __init__(self, segment_configs: segment_configuration.SegmentConfigs, **kwards):
        logging.warning(
            f"No longer maintaining {self.__class__}, only used for demonstration purposes"
        )
        super().__init__(segment_configs=segment_configs, **kwards)

    def __holder_space__(self):
        diameter = (
            self.motor_configs.Radius + self.motor_configs.MotorFastenerGirth
        ) * 2
        holder = (
            cq.Workplane(self.workplane_primary)
            .rect(diameter, diameter)
            .extrude(-self.motor_configs.MotorFastenerHeight)
        )
        return holder

    def __mesh_template__(self, add_surface_give):
        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0

        motor_hex_points_half = [
            (0, 0),
            (0, self.motor_configs.Radius + surface_give),
            (
                self.motor_configs.HexRadius + surface_give,
                self.motor_configs.Radius + surface_give,
            ),
            (
                self.motor_configs.Radius + surface_give,
                self.motor_configs.HexRadius + surface_give,
            ),
            (
                self.motor_configs.Radius + surface_give,
                -(self.motor_configs.HexRadius + surface_give),
            ),
            (
                self.motor_configs.HexRadius + surface_give,
                -(self.motor_configs.Radius + surface_give),
            ),
            (0, -(self.motor_configs.Radius + surface_give)),
        ]
        motor_plane = (
            cq.Workplane(self.workplane_primary)
            .polyline(motor_hex_points_half)
            .mirrorY()
        )
        motor_mesh = motor_plane.extrude(-(self.motor_configs.Height + surface_give))

        if add_surface_give is True and not self.install_from_side_instead_along:
            motor_mesh = motor_mesh.add(self.__motor_side_opening_space__())

        return motor_mesh

    def __motor_side_opening_space__(self):
        motor_socket_opening = cq.Workplane(self.workplane_primary).box(
            self.motor_configs.HexRadius,
            self.segment_configs.structural.Far,
            self.motor_configs.Height,
        )
        motor_socket_opening = motor_socket_opening.translate(
            (0, 0, -self.motor_configs.Height / 2)
        )

        return motor_socket_opening

    def optional_space(self):
        return cq.Workplane(self.workplane_primary)

    def __shaft_bolt_driver_opening_space__(self):
        horn_bolt_driver_space = (
            cq.Workplane(self.workplane_secondary)
            .circle(self.segment_configs.motor_horn.HornShaftBoltDriverSpaceRadius)
            .extrude(-self.segment_configs.structural.Far)
            .translate(
                (
                    -self.segment_configs.motor_horn.HornShaftBoltDriverSpaceCenterOffset,
                    0,
                    self.segment_configs.structural.PitchCenterLocationZ,
                )
            )
        )
        return horn_bolt_driver_space
