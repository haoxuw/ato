# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import bolt, horn_base


class HornDiskNema17(horn_base.HornBase):
    def __init__(self, segment_configs: segment_configuration.SegmentConfigs, **kwards):
        self.horn_configs: segment_configuration.HornDiskConfigsNema17 = (
            segment_configuration.HornDiskConfigsNema17()
        )
        super().__init__(segment_configs=segment_configs, **kwards)

    @property
    def horn_bolt(self):
        return bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=self.install_bolt_from_side_instead_along,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=3, BoltLength=16
            ),
        )

    def __mesh_template__(self, add_surface_give=False, inclose_from_motor=None):
        if add_surface_give:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0
        horn_space = (
            cq.Workplane(self.workplane_primary)
            .circle(self.horn_configs.HornRadius + surface_give / 2)
            .extrude(self.horn_configs.HornHeightBoundary)
        )
        return horn_space

    def __horn_bolt_space__(self):
        space = cq.Workplane(self.workplane_primary)

        for offset_x in [
            -self.horn_configs.HornBoltRadius,
            self.horn_configs.HornBoltRadius,
        ]:
            for offset_y in [
                -self.horn_configs.HornBoltRadius,
                self.horn_configs.HornBoltRadius,
            ]:
                bolt_space = self.__relocate_bolt_on_horn__(
                    self.horn_bolt.space_mesh(),
                    (
                        offset_x,
                        offset_y,
                        self.horn_configs.HornInnerOffset,
                    ),
                )
                space = space.add(bolt_space)
        return space
