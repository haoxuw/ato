# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass

import cadquery as cq

from mesh.cq_lib import cq_mesh


@dataclass
class WheelConfigs:
    SurfaceGive: float = 1
    WheelRadius: float = 12
    WheelWidth: float = 3
    WheelDistanceX: float = 45
    WheelDistanceY: float = 60
    ShaftRadis: float = 5
    BedPadding: float = ShaftRadis + 2
    BedPilarHeight: float = 40
    BedPilarRadius: float = 10


class Wheels(cq_mesh.CqMesh):
    def __init__(
        self,
        wheel_configs: WheelConfigs = None,
    ):
        super().__init__(segment_configs=None)
        if wheel_configs is None:
            self.wheel_configs = WheelConfigs()
        else:
            self.wheel_configs = wheel_configs
        self.num_wheel_pairs = 2

    def model_mesh(self, add_surface_give=False):
        if add_surface_give:
            surface_give = self.wheel_configs.SurfaceGive
        else:
            surface_give = 0
        mesh = cq.Workplane(self.workplane_primary)
        for i in range(self.num_wheel_pairs):
            pair = self.__wheel_pair__(surface_give=surface_give)
            distance_y = self.wheel_configs.WheelDistanceY * i
            mesh = mesh.add(pair.translate((0, distance_y, 0)))
        return mesh.add(self.__bed__())

    def __wheel_pair__(self, surface_give):
        shaft = (
            cq.Workplane(self.workplane_secondary)
            .circle(self.wheel_configs.ShaftRadis + surface_give / 2)
            .extrude(self.wheel_configs.WheelDistanceX)
            .translate((-self.wheel_configs.WheelDistanceX / 2, 0, 0))
        )
        one_wheel = (
            cq.Workplane(self.workplane_secondary)
            .circle(self.wheel_configs.WheelRadius + surface_give / 2)
            .extrude(self.wheel_configs.WheelWidth * 2 + surface_give)
            .translate((-self.wheel_configs.WheelWidth - surface_give / 2, 0, 0))
        )
        pair = shaft.add(
            one_wheel.translate((-self.wheel_configs.WheelDistanceX / 2, 0, 0))
        ).add(one_wheel.translate((self.wheel_configs.WheelDistanceX / 2, 0, 0)))
        return pair

    def __bed__(self):
        padding_diameter = self.wheel_configs.BedPadding * 2
        bed_length = self.wheel_configs.WheelDistanceY * (self.num_wheel_pairs - 1)
        bed = (
            cq.Workplane(self.workplane_tertiary)
            .rect(
                self.wheel_configs.WheelDistanceX
                - self.wheel_configs.WheelRadius * 2
                + padding_diameter,
                padding_diameter,
            )
            .extrude(-(bed_length + padding_diameter))
            .translate((0, -padding_diameter / 2, 0))
        )
        pillar = (
            cq.Workplane(self.workplane_primary)
            .circle(self.wheel_configs.BedPilarRadius)
            .extrude(self.wheel_configs.BedPilarHeight)
            .translate((0, bed_length / 2, 0))
        )
        bed = bed.add(pillar)
        return bed
