# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import copy

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh


class Bolt(cq_mesh.CqMesh):
    def __init__(
        self,
        bolt_configs: segment_configuration.BoltConfigsBase,
        install_from_side_instead_along: bool,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        bolt_head_insert_space_length_override: float = None,  # leave space for bolt driver if true, defaults to Far
        nut_side_insert_space_length_override: float = None,  # leave space to slide in nut from the side, defaults to Far
        nut_bottom_insert_space_length_override: float = None,  # used when install_from_side_instead_along==False, defaults to Far
        bolt_socket_diam_tighten_factor=0.85,  # make bolt dim smaller so socket could hug the helix
        nut_offset_factor=0.95,  # when set to 1, nut socket would be at bottom, when 0.5, nut would be on the middle
        side_print_opening_length_ratio=0.382,  # for easier 3d printing, we can give a portion of the bolt socket an open ceiling, so that there's no need to dig out supporting material, 0.618 means tp print 38.2% of ceiling
        bolt_length_override: float = None,
        print_opening_space_on_both_sides: bool = False,
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        self.bolt_configs = copy.deepcopy(bolt_configs)
        self.install_from_side_instead_along = install_from_side_instead_along
        self.nut_offset = self.bolt_configs.BoltLength * nut_offset_factor

        if bolt_length_override is not None:
            self.bolt_length_override = bolt_length_override
        else:
            self.bolt_length_override = self.bolt_configs.BoltLength
        if segment_configs.BoltSocketDiamTightenFactor is not None:
            bolt_socket_diam_tighten_factor = (
                segment_configs.BoltSocketDiamTightenFactor
            )
        self.tightened_bolt_diam = (
            self.bolt_configs.BoltDiam * bolt_socket_diam_tighten_factor
        )
        self.side_print_opening_length_ratio = side_print_opening_length_ratio
        self.print_opening_space_on_both_sides = print_opening_space_on_both_sides

        self.nut_side_insert_space_length = self.__may_do_override__(
            nut_side_insert_space_length_override
        )
        self.nut_bottom_insert_space_length = self.__may_do_override__(
            nut_bottom_insert_space_length_override
        )
        self.bolt_head_insert_space_length = self.__may_do_override__(
            bolt_head_insert_space_length_override
        )

    def __may_do_override__(self, override_value, default_value=None):
        if override_value is None:
            if default_value is None:
                return self.segment_configs.structural.Far
            else:
                return default_value
        else:
            return override_value

    def model_mesh(self, add_surface_give=False):
        return self.__mesh_template__(add_surface_give=add_surface_give)

    def space_mesh(self):
        return self.__mesh_template__(is_space=True)

    def __mesh_template__(self, add_surface_give=False, is_space=False):
        assert not (add_surface_give and is_space)
        if add_surface_give or is_space:
            surface_give = self.segment_configs.structural.SurfaceGive
        else:
            surface_give = 0
        nut_side_insert_length = 0
        nut_bottom_insert_space_length = 0
        if is_space:
            if self.install_from_side_instead_along:
                nut_side_insert_length = self.nut_side_insert_space_length
            else:
                nut_bottom_insert_space_length = self.nut_bottom_insert_space_length

        # origin is set at where bolt head connects body
        bolt_head = (
            cq.Workplane(self.workplane_primary)
            .circle((self.bolt_configs.BoltHeadDiam) / 2 + surface_give / 2)
            .extrude(
                self.bolt_configs.BoltHeadHeight
                + surface_give
                + self.bolt_head_insert_space_length
            )
        )
        bolt_body = (
            cq.Workplane(self.workplane_primary)
            .circle(
                (self.tightened_bolt_diam + self.segment_configs.structural.SurfaceGive)
                / 2
            )
            .extrude(
                -(
                    self.bolt_length_override
                    + self.segment_configs.structural.SurfaceGive / 2
                )
            )
        )

        nut_hex_short_radius = self.bolt_configs.NutHexDiam / 2
        nut_hex_long_radius = nut_hex_short_radius / (3**0.5) * 2

        nut_hex_points_quarter = [
            (nut_hex_short_radius + surface_give / 2, 0),
            (
                nut_hex_short_radius + surface_give / 2,
                nut_hex_long_radius / 2 + surface_give / 2,
            ),
            (0, nut_hex_long_radius + surface_give / 2),
        ]
        nut = (
            cq.Workplane(self.workplane_primary)
            .polyline(nut_hex_points_quarter)
            .mirrorY()
            .mirrorX()
            .extrude(
                -self.bolt_configs.NutHeight
                - surface_give
                - nut_bottom_insert_space_length
            )
            .translate(
                (
                    0,
                    0,
                    -self.nut_offset + self.bolt_configs.NutHeight + surface_give / 2,
                )
            )
        )

        bolt = bolt_head.add(bolt_body).add(nut)

        if (
            is_space and self.install_from_side_instead_along
        ):  # add additional space from the side
            bolt_side_print_opening_space = self.__side_print_opening_space__(
                nut_hex_short_radius=nut_hex_short_radius,
                nut_side_insert_length=nut_side_insert_length,
                surface_give=surface_give,
            )
            bolt = bolt.add(bolt_side_print_opening_space)

        return bolt

    def __bolt_side_print_opening_space__(self, nut_side_insert_length, surface_give):
        assert 0 <= self.side_print_opening_length_ratio <= 1
        if self.side_print_opening_length_ratio <= 0:
            return cq.Workplane(self.workplane_tertiary)
        else:
            opening_length = (
                self.bolt_configs.BoltLength * self.side_print_opening_length_ratio
            )
            # when installing a bolt from the side, the container model has to leave a side opening for the nut, as its larger than bolt body, and cannot be slide along bolt socket
            # meanwhile it is also difficult to remove the 3D printing supporting structure inside of the bolt socket so
            # when side_print_opening_length_ratio == 1, we leave a side opening for the entire bolt's head+body+nut, so that no supports needs to be printed
            # when 0<=side_print_opening_length_ratio<1, we leave a side opening the bolt's body+nut, so that some supports are printed, but can be easily pushed in, and removed from the sides
            if self.side_print_opening_length_ratio == 1:
                install_space_include_bolt_head = True
            else:
                install_space_include_bolt_head = False

            bolt_opening_points_half = []  # origin is where bolt head connects body
            if install_space_include_bolt_head:
                bolt_opening_points_half += [
                    (0, self.bolt_configs.BoltHeadHeight),
                    (
                        self.bolt_configs.BoltHeadDiam / 2 + surface_give / 2,
                        self.bolt_configs.BoltHeadHeight,
                    ),
                    (self.bolt_configs.BoltHeadDiam / 2 + surface_give / 2, 0),
                ]
            else:
                bolt_opening_points_half += [
                    (0, -(self.bolt_configs.BoltLength - opening_length)),
                    (
                        self.tightened_bolt_diam / 2 + surface_give / 2,
                        -(self.bolt_configs.BoltLength - opening_length),
                    ),
                ]
            bolt_opening_points_half += [
                (
                    self.tightened_bolt_diam / 2 + surface_give / 2,
                    -self.bolt_configs.BoltLength - surface_give / 2,
                ),
                (0, -self.bolt_configs.BoltLength - surface_give / 2),
            ]
            bolt_side_print_opening_space = (
                cq.Workplane(self.workplane_tertiary)
                .polyline(bolt_opening_points_half)
                .mirrorY()  # this makes a box -- to be consistent with code pattern of nut
                .extrude(nut_side_insert_length)
            )
            return bolt_side_print_opening_space

    def __side_print_opening_space__(
        self, nut_hex_short_radius, nut_side_insert_length, surface_give
    ):
        nut_opening_points_half = [
            (0, -self.nut_offset + self.bolt_configs.NutHeight + surface_give / 2),
            (
                nut_hex_short_radius + surface_give / 2,
                -self.nut_offset + self.bolt_configs.NutHeight + surface_give / 2,
            ),
            (
                nut_hex_short_radius + surface_give / 2,
                -self.nut_offset - surface_give / 2,
            ),
            (0, -self.nut_offset - surface_give / 2),
        ]
        nut_side_print_opening_space = (
            cq.Workplane(self.workplane_tertiary)
            .polyline(nut_opening_points_half)
            .mirrorY()
            .extrude(nut_side_insert_length)
        )
        bolt_side_print_opening_space = self.__bolt_side_print_opening_space__(
            nut_side_insert_length=nut_side_insert_length, surface_give=surface_give
        )
        print_opening_space = bolt_side_print_opening_space.add(
            nut_side_print_opening_space
        )
        if self.print_opening_space_on_both_sides:
            print_opening_space = print_opening_space.translate(
                (0, nut_side_insert_length / 2, 0)
            )
        return print_opening_space
