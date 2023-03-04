# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import math
from abc import abstractmethod

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh
from mesh.model_structure.gripper import gripper_bone


class GripperUsingJaw(gripper_bone.GripperBone):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def _relocate(self, mesh: cq_mesh.CqMesh):
        offsets = (-19.5, 0, 0)
        return (
            mesh.rotate((0, 0, 0), (0, 0, 1), -90)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate(self._rotation_params_()[0])
            .translate(offsets)
        )

    @abstractmethod
    def _import_decoration_mesh_(self, **kwargs):
        pass

    # tunable base on the mesh
    @abstractmethod
    def _rotation_params_(self):
        """
        return center, rotation_radius, holder_radius, holder_length, mouth_angle, mouth_opening_factor
        """
        # we draw a circle to cut out jaw line, so the jaw can rotate
        # meanwhile this cut cannot cut through the jaw itself, mouth_opening_factor determines wide

    # use rectangle_x_delta to remove +Y boarder of carver
    def __rotation_carver_template__(
        self, rotation_radius, mouth_angle, mouth_opening_factor, rectangle_x_delta=0
    ):
        far = self.segment_configs.structural.BoneRadius * 3

        circle = (
            cq.Workplane(self.workplane_secondary)
            .circle(rotation_radius)
            .extrude(far)
            .translate((-far / 2, 0, 0))
        )
        rectangle = (
            cq.Workplane(self.workplane_secondary)
            .polyline(
                [
                    (0, 0),
                    (0, rotation_radius * mouth_opening_factor),
                    (
                        rotation_radius + rectangle_x_delta,
                        rotation_radius * mouth_opening_factor,
                    ),
                    (rotation_radius + rectangle_x_delta, 0),
                ]
            )
            .mirrorX()
            .extrude(far)
            .translate((-far / 2, 0, 0))
        )
        rectangle = rectangle.rotate((0, 0, 0), (1, 0, 0), mouth_angle)
        return circle.add(rectangle)

    def __rotation_holder_angle__(self):
        return 60

    def __rotation_holder_template__(
        self, holder_radius, holder_length, connector_z_delta=0
    ):
        holder = (
            cq.Workplane(self.workplane_secondary)
            .circle(holder_radius)
            .extrude(holder_length)
            .translate((-holder_length / 2, 0, 0))
        )
        connector = (
            cq.Workplane(self.workplane_tertiary)
            .circle(holder_radius)
            .extrude(holder_length + connector_z_delta)
        ).rotate((0, 0, 0), (1, 0, 0), -self.__rotation_holder_angle__())
        return holder.add(connector)

    def __rotation_holder_space_template__(
        self, rotation_radius, holder_radius, overshot=True
    ):
        """
        overshot: if allow jaw rotate backwards, i.e. towards close
        """
        assert 0 < self.__rotation_holder_angle__() < 90
        if overshot:
            # rectangle
            space = (
                cq.Workplane(self.workplane_primary)
                .polyline(
                    [
                        (0, -rotation_radius),
                        (holder_radius, -rotation_radius),
                        (holder_radius, holder_radius),
                        (0, holder_radius),
                    ]
                )
                .mirrorY()
                .extrude(-rotation_radius * 4)
                .translate((0, 0, rotation_radius * 2))
            )
        else:
            bottom_space = (
                cq.Workplane(self.workplane_primary)
                .polyline(
                    [
                        (0, -rotation_radius),
                        (holder_radius, -rotation_radius),
                        (holder_radius, holder_radius),
                        (0, holder_radius),
                    ]
                )
                .mirrorY()
                .extrude(-rotation_radius * 2)
            )
            polyline = [
                (0, 0),
                (-rotation_radius, 0),
                (
                    -rotation_radius,
                    rotation_radius
                    * math.tan(self.__rotation_holder_angle__() * math.pi / 180.0),
                ),
            ]
            top_space = (
                cq.Workplane(self.workplane_secondary)
                .polyline(polyline)
                .close()
                .extrude(holder_radius * 2)
                .translate((-holder_radius, 0, 0))
            )
            space = bottom_space.add(top_space)
        return space

    # e.g. for the dragon head gripper:
    # make cuts at jawline so that the chin could rotate
    def __rotation_space__(self, gap_size=0.25):
        (
            center,
            rotation_radius,
            holder_radius,
            holder_length,
            mouth_angle,
            mouth_opening_factor,
        ) = self._rotation_params_()
        assert gap_size >= self.segment_configs.nozzle_diameter / 2

        holder_space = self.__rotation_holder_template__(
            holder_radius=holder_radius + gap_size,
            holder_length=holder_length + gap_size,
        )
        rotation_holder_space = self.__rotation_holder_space_template__(
            rotation_radius=rotation_radius + gap_size,
            holder_radius=holder_radius + gap_size,
        )
        holder_space = holder_space.add(rotation_holder_space)

        holder_mesh = self.__rotation_holder_template__(
            holder_radius=holder_radius,
            holder_length=holder_length,
            connector_z_delta=gap_size * 2,
        )
        # consider refactor with cq.shell
        holder_hollow_space = holder_space.cut(holder_mesh)

        # to carve rotation structures of the jaw
        carver_space = self.__rotation_carver_template__(
            rotation_radius=rotation_radius + gap_size,
            mouth_angle=mouth_angle,
            mouth_opening_factor=mouth_opening_factor,
        )
        carver_hollow_space = carver_space.cut(
            self.__rotation_carver_template__(
                rotation_radius=rotation_radius,
                rectangle_x_delta=gap_size * 2,
                mouth_angle=mouth_angle,
                mouth_opening_factor=mouth_opening_factor,
            )
        )
        carver_hollow_space = carver_hollow_space.cut(holder_mesh)
        space = holder_hollow_space.add(carver_hollow_space)

        space = self.__trim_top_back_corner__(
            rotation_space=space,
            rotation_radius=rotation_radius + gap_size,
            carver_space=carver_space,
        )
        space = space.translate(center)
        return space

    def __trim_top_back_corner__(self, rotation_space, rotation_radius, carver_space):
        trimmer_radius = rotation_radius / math.cos(
            self.__rotation_holder_angle__() * math.pi / 180.0
        )

        # rectangle when viewed from 2d
        rectangle = (
            cq.Workplane(self.workplane_secondary)
            .polyline(
                [
                    (trimmer_radius, 0),
                    (-trimmer_radius, 0),
                    (-trimmer_radius, trimmer_radius),
                    (trimmer_radius, trimmer_radius),
                ]
            )
            .close()
            .extrude(rotation_radius * 2)
            .translate((-rotation_radius, 0, 0))
        )

        trimmer = (rectangle).cut(carver_space)
        return rotation_space.cut(trimmer)

    def _gripper_(self):
        rotation_space = self.__rotation_space__()
        decoration_mesh = self._import_decoration_mesh_()
        return decoration_mesh, rotation_space

    def printable_mesh(self, face_upwards=True):
        mesh = self.model_mesh(add_surface_give=True)
        # affine transform to print up straight, for easier removal of supporting materials
        mesh = mesh.translate(
            (
                0,
                self.segment_configs.structural.RollBoneMeshNetLength
                - self._bone_cut_length_(),
                0,
            )
        )
        if face_upwards:
            mesh = mesh.rotate((0, 0, 0), (1, 0, 0), 90)
        else:
            mesh = mesh.rotate((0, 0, 0), (0, 1, 0), 90)
        return mesh
