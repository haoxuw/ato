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
from mesh.cq_lib import cq_mesh
from mesh.model_structure.utilities import half_masks

from . import bone_abstract


class JointAbstract(cq_mesh.CqMesh):
    ROUND = -1

    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        assert self.segment_configs.joint_polygon_sides == self.ROUND or (
            self.segment_configs.joint_polygon_sides >= 6
            and self.segment_configs.joint_polygon_sides % 2 == 0
        ), f"Expect sides == {self.segment_configs.joint_polygon_sides} of polygon to be -1 (round), or 6, 8, 12 ..."

    @property
    def bone_class(self):
        return bone_abstract.BoneAbstract

    def model_mesh(self, add_surface_give=False):
        if self.segment_configs.joint_polygon_sides == self.ROUND:
            joint = self.__round_joint()
        else:
            joint = self.__joint_polygon_sides(self.segment_configs.joint_polygon_sides)

        joint = joint.cut(
            self.bone_class(
                segment_configs=self.segment_configs, allocate_both_bones_as_whole=True
            ).space_mesh()
        )

        return joint

    def __round_joint(self):
        joint_mid = (
            cq.Workplane(self.workplane_primary)
            .circle(self.segment_configs.structural.JointRadius)
            .extrude(self.segment_configs.structural.JointLength)
        )
        joint_pitch = cq.Workplane(self.workplane_primary).sphere(
            self.segment_configs.structural.JointRadius
        )
        joint_roll = (
            cq.Workplane(self.workplane_tertiary)
            .sphere(self.segment_configs.structural.JointRadius)
            .translate((0, 0, self.segment_configs.structural.JointLength))
        )
        return joint_mid.add(joint_pitch).add(joint_roll)

    @property
    # self.segment_configs.structural.JointRadius = radius of the inner circle of the polygon
    # this function return the radius of outer circle
    def outer_radius(self):
        if self.segment_configs.joint_polygon_sides == self.ROUND:  # round
            return self.segment_configs.structural.JointRadius
        return self.segment_configs.structural.JointRadius / math.cos(
            math.pi * 2 / self.segment_configs.joint_polygon_sides / 2
        )

    @property
    # for polygons outer_radius if sides%4 == 0 then an edge would be pointing downwards
    # if sides%4 != 0 then a surface would be aligned on the bottom
    # this matters when printing joint along bone with a 3D printer in one go
    # max_distance_from_surface describes the vertical center position of the joint, when laid on a plane
    def max_distance_from_surface(self):
        if (
            self.segment_configs.joint_polygon_sides == self.ROUND
            or self.segment_configs.joint_polygon_sides % 4 == 0
        ):
            return self.outer_radius
        else:
            return self.segment_configs.structural.JointRadius

    def __joint_polygon_sides(self, sides):
        radius = self.segment_configs.structural.JointRadius
        joint_mid = (
            cq.Workplane(self.workplane_primary)
            .pushPoints([(0, 0)])
            .polygon(nSides=sides, diameter=self.outer_radius * 2)
            .extrude(self.segment_configs.structural.JointLength)
        )

        joint_pitch = (
            cq.Workplane(self.workplane_primary)
            .pushPoints([(0, 0)])
            .polygon(nSides=sides, diameter=self.outer_radius * 2)
            .workplane(offset=-radius * 2)
            .rect(0.001, 0.001)
            .loft()
        )

        joint_roll = (
            cq.Workplane(self.workplane_primary)
            .pushPoints([(0, 0)])
            .polygon(nSides=sides, diameter=self.outer_radius * 2)
            .workplane(offset=radius * 2)
            .rect(0.001, 0.001)
            .loft()
            .translate((0, 0, self.segment_configs.structural.JointLength))
        )
        return joint_mid.add(joint_pitch).add(joint_roll)


class JointAbstractPos(cq_mesh.CqMesh):
    @property
    def joint_class(self):
        return JointAbstract

    def model_mesh(self, add_surface_give=False):
        neg_half = half_masks.Halves(
            segment_configs=self.segment_configs, positive=False
        ).model_mesh(add_surface_give=True)
        pos_mesh = (
            self.joint_class(segment_configs=self.segment_configs)
            .model_mesh(add_surface_give=add_surface_give)
            .cut(neg_half)
        )
        return pos_mesh

    def printable_mesh(self):
        pos_mesh = self.model_mesh(add_surface_give=True)
        pos_mesh = pos_mesh.rotate((0, 0, 0), (1, 0, 0), 90)
        return pos_mesh


class JointAbstractNeg(cq_mesh.CqMesh):
    @property
    def joint_class(self):
        return JointAbstract

    def model_mesh(self, add_surface_give=False):
        pos_half = half_masks.Halves(
            segment_configs=self.segment_configs, positive=True
        ).model_mesh(add_surface_give=True)
        neg_mesh = (
            self.joint_class(segment_configs=self.segment_configs)
            .model_mesh(add_surface_give=add_surface_give)
            .cut(pos_half)
        )
        return neg_mesh

    def printable_mesh(self):
        neg_mesh = self.model_mesh(add_surface_give=True)
        neg_mesh = neg_mesh.rotate((0, 0, 0), (0, 0, 1), 180)
        neg_mesh = neg_mesh.rotate((0, 0, 0), (1, 0, 0), 90)
        return neg_mesh
