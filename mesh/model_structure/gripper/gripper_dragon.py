# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import os
import pathlib

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.model_structure.gripper import gripper_decorative


class GripperDragon(gripper_decorative.GripperDecorative):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def _bone_cut_length_(self):
        # depends how large is the dragon_head
        return 38.5

    def _rotation_params_(self):
        radius = self.segment_configs.structural.BoneRadius * 0.72
        # center, rotation_radius, holder_radius, holder_length, mouth_angle, mouth_opening_factor
        # mouth_angle is hard coded to align with the dragon's mouth
        return (0, 4.5, -9), radius, radius / 2.1, radius * 2, -30, 0.89

    def _import_decoration_mesh_(self, stl_file="dragon_head.step", **kwargs):
        current_dir = pathlib.Path(__file__).parent
        mesh = cq.importers.importStep(os.path.join(current_dir, stl_file))
        # this is a hard-coded offset to recenter the stl file
        offset = (0, 0, -5)
        mesh = mesh.translate(offset)
        # turn the head towards +Y
        mesh = mesh.rotate((0, 0, 0), (0, 0, 1), 180)
        return mesh
