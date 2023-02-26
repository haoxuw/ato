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


class Halves(cq_mesh.CqMesh):
    def __init__(
        self,
        positive: bool,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        self.positive = positive

    def model_mesh(self, add_surface_give=False):
        mesh = cq.Workplane(self.workplane_primary).box(
            self.segment_configs.structural.VeryFar,
            self.segment_configs.structural.VeryFar,
            self.segment_configs.structural.VeryFar,
        )

        if self.positive:
            return mesh.translate((0, -self.segment_configs.structural.VeryFar / 2, 0))
        else:
            return mesh.translate((0, self.segment_configs.structural.VeryFar / 2, 0))
