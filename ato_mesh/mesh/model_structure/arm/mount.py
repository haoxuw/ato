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
from mesh.model_structure.segment import bone_augmented
from mesh.model_structure.utilities import half_masks


class Mount(cq_mesh.CqMesh):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def model_mesh(self, add_surface_give=False):
        factor = self.segment_configs.structural.BaseSizeFactor
        base = cq.Workplane(self.workplane_primary)
        length = 3 * factor
        width = 1 * factor
        height = 1 * factor
        base = base.box(length=length, width=width, height=height)
        pitch = bone_augmented.BonePitch(
            segment_configs=self.segment_configs
        ).model_mesh()
        offset_z = height / 3 - self.segment_configs.structural.BoneRadius
        pitch = pitch.translate((0, 0, offset_z))
        base = base.cut(pitch)
        return base


class MountPos(Mount):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def model_mesh(self, add_surface_give=False):
        neg_half = half_masks.Halves(
            segment_configs=self.segment_configs, positive=False
        ).model_mesh(add_surface_give=add_surface_give)
        pos_mesh = super().model_mesh().cut(neg_half)
        return pos_mesh

    def printable_mesh(self):
        mesh = self.model_mesh(add_surface_give=True)
        mesh = mesh.rotate((0, 0, 0), (1, 0, 0), 180)
        return mesh


class MountNeg(Mount):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def model_mesh(self, add_surface_give=False):
        pos_half = half_masks.Halves(
            segment_configs=self.segment_configs, positive=True
        ).model_mesh(add_surface_give=add_surface_give)
        neg_mesh = super().model_mesh().cut(pos_half)
        return neg_mesh

    def printable_mesh(self):
        mesh = self.model_mesh(add_surface_give=True)
        mesh = mesh.rotate((0, 0, 0), (1, 0, 0), -180)
        return mesh
