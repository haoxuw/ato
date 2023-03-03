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
from mesh.model_structure.segment.abstractions import bone_abstract
from mesh.model_structure.utilities import half_masks


class Mount(cq_mesh.CqMesh):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def _get_dimensions(self):
        factor = self.segment_configs.structural.MountUnitSize
        length = 2 * factor
        width = 2 * factor
        height = 0.8 * factor
        return length, width, height

    def model_mesh(self, add_surface_give=False):
        length, width, height = self._get_dimensions()
        base = cq.Workplane(self.workplane_primary)
        base = base.box(length=length, width=width, height=height)
        pitch = bone_abstract.BoneAbstract(
            segment_configs=self.segment_configs,
            allocate_both_bones_as_whole=True,
            allocate_pitch=True,
            allocate_roll=True,
        ).model_mesh(
            add_surface_give=False
        )  # add_surface_give=False to create tight fit
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


class MountCombined(Mount):
    def __get_halves(self, add_surface_give=False):
        pos_half = MountNeg(segment_configs=self.segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )
        neg_half = MountPos(segment_configs=self.segment_configs).model_mesh(
            add_surface_give=add_surface_give
        )
        return pos_half, neg_half

    def model_mesh(self, add_surface_give=False):
        pos_half, neg_half = self.__get_halves(add_surface_give=add_surface_give)
        mesh = pos_half.add(neg_half)
        mesh = mesh.rotate((0, 0, 0), (1, 0, 0), -180)
        return mesh

    def printable_mesh(self):
        pos_half, neg_half = self.__get_halves()
        _, width, _ = self._get_dimensions()
        pos_half = pos_half.translate(
            (0, -width / 2 - self.segment_configs.structural.SurfaceGive, 0)
        )
        neg_half = neg_half.translate(
            (0, width / 2 + self.segment_configs.structural.SurfaceGive, 0)
        )
        mesh = pos_half.add(neg_half)
        mesh = mesh.rotate((0, 0, 0), (1, 0, 0), -180)
        return mesh
