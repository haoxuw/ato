# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import bolt, motor_base


class ServoMotor(motor_base.MotorBase):
    motor_configs: segment_configuration.ServoConfigsDs5160

    def __holder_space__(self):
        length = (
            self.motor_configs.Length
            + self.motor_configs.MotorFastenerBoltSocketXExtra * 2
            + self.motor_configs.MotorFastenerGirth * 2
        )
        width = self.motor_configs.Width + self.motor_configs.MotorFastenerGirth * 2
        holder = (
            cq.Workplane(self.workplane_primary)
            .rect(length, width)
            .extrude(-self.motor_configs.MotorFastenerHeight)
        )
        holder = holder.translate((self.motor_configs.ShaftCenterOffsetX, 0, 0))
        return holder

    def __mesh_template__(self, add_surface_give):
        if add_surface_give:
            surface_give = (
                self.segment_configs.structural.SurfaceGive / 2
            )  # tight -- the motor exteriors are usually true to the advertised size
        else:
            surface_give = 0

        motor_points_half = [
            (0, 0),
            (0, self.motor_configs.Width / 2 + surface_give),
            (
                self.motor_configs.Length / 2 + surface_give,
                self.motor_configs.Width / 2 + surface_give,
            ),
            (
                self.motor_configs.Length / 2 + surface_give,
                -(self.motor_configs.Width / 2 + surface_give),
            ),
            (0, -(self.motor_configs.Width / 2 + surface_give)),
        ]
        motor_plane = (
            cq.Workplane(self.workplane_primary).polyline(motor_points_half).mirrorY()
        )
        motor_mesh = motor_plane.extrude(-(self.motor_configs.Height + surface_give))

        holder_points_half = [
            (0, 0),
            (0, self.motor_configs.Width / 2 + surface_give),
            (
                self.motor_configs.Length / 2
                + self.motor_configs.MotorFastenerBoltSocketXExtra
                + surface_give,
                self.motor_configs.Width / 2 + surface_give,
            ),
            (
                self.motor_configs.Length / 2
                + self.motor_configs.MotorFastenerBoltSocketXExtra
                + surface_give,
                -(self.motor_configs.Width / 2 + surface_give),
            ),
            (0, -(self.motor_configs.Width / 2 + surface_give)),
        ]
        holder_plane = (
            cq.Workplane(self.workplane_primary).polyline(holder_points_half).mirrorY()
        )
        holder = holder_plane.extrude(
            -(
                self.motor_configs.MotorFastenerBoltSocketZBot
                - self.motor_configs.MotorFastenerBoltSocketZTop
                + surface_give
            )
        )
        holder = holder.translate(
            (0, 0, -self.motor_configs.MotorFastenerBoltSocketZTop)
        )
        motor_mesh = motor_mesh.add(holder)
        if self.motor_configs.ExtrusionPresents:
            extrusion = (
                cq.Workplane(self.workplane_primary)
                .circle(
                    self.motor_configs.ExtrusionRadius
                    + self.segment_configs.structural.SurfaceGive / 2
                )
                .extrude(self.motor_configs.ExtrusionHeight)
                .translate((0, self.motor_configs.ExtrusionOffsetY, 0))
            )
            motor_mesh = motor_mesh.add(extrusion)
        if add_surface_give is True:
            motor_mesh = motor_mesh.add(self.__wire_tunnel__())
        motor_mesh = motor_mesh.translate((self.motor_configs.ShaftCenterOffsetX, 0, 0))
        # mesh = mesh.add(horn.mirror(self.workplane_primary))
        return motor_mesh

    def __wire_tunnel__(self):
        surface_give = self.segment_configs.structural.SurfaceGive
        wire_tunnel_points_half = [
            (0, 0),
            (0, self.motor_configs.WireTunnelWidth / 2 + surface_give),
            (
                self.motor_configs.Length / 2
                + self.motor_configs.MotorFastenerBoltSocketXExtra
                + surface_give,
                self.motor_configs.WireTunnelWidth / 2 + surface_give,
            ),
            (
                self.motor_configs.Length / 2
                + self.motor_configs.MotorFastenerBoltSocketXExtra
                + surface_give,
                -(self.motor_configs.WireTunnelWidth / 2 + surface_give),
            ),
            (0, -(self.motor_configs.WireTunnelWidth / 2 + surface_give)),
        ]

        wire_tunnel_plane = (
            cq.Workplane(self.workplane_primary)
            .polyline(wire_tunnel_points_half)
            .mirrorY()
        )
        wire_tunnel = wire_tunnel_plane.extrude(
            -(
                self.motor_configs.Height
                - self.motor_configs.MotorFastenerBoltSocketZTop
                + self.motor_configs.WireTunnelHeight
                + surface_give
            )
        )
        wire_tunnel = wire_tunnel.translate(
            (0, 0, -self.motor_configs.MotorFastenerBoltSocketZTop)
        )
        return wire_tunnel

    def optional_space(self):
        if self.generate_motor_fastening_bolt_space is not None:
            return self.motor_fastening_bolt_space()
        else:
            return None

    def motor_fastening_bolt(self):
        return bolt.Bolt(
            segment_configs=self.segment_configs,
            # when motor is inserted from side, servo motor bolt would be also inserted from along
            install_from_side_instead_along=self.install_from_side_instead_along,
            nut_side_insert_space_length_override=self.motor_configs.MotorFastenerBoltY
            * 2,
            nut_offset_factor=0.95,
            nut_bottom_insert_space_length_override=0,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=3, BoltLength=16
            ),
        )

    def motor_fastening_bolt_space(self):
        bolt_space = cq.Workplane(self.workplane_primary)
        fastener_bolt = self.motor_fastening_bolt()
        offset_z = -self.motor_configs.MotorFastenerBoltZ
        dist_x = self.motor_configs.Length / 2 + self.motor_configs.MotorFastenerBoltX

        bolt_id = -1
        for offset_x in [
            self.motor_configs.ShaftCenterOffsetX + dist_x,
            self.motor_configs.ShaftCenterOffsetX - dist_x,
        ]:
            for offset_y in [
                self.motor_configs.MotorFastenerBoltY,
                -self.motor_configs.MotorFastenerBoltY,
            ]:
                bolt_id += 1
                if isinstance(self.generate_motor_fastening_bolt_space, set):
                    if bolt_id not in self.generate_motor_fastening_bolt_space:
                        continue
                space = fastener_bolt.space_mesh()
                space = space.rotate((0, 0, 0), (1, 0, 0), 180)
                if offset_y > 0:
                    # so that opening space is from above
                    space = space.rotate((0, 0, 0), (0, 0, 1), 180)
                space = space.translate((offset_x, offset_y, offset_z))
                bolt_space.add(space)
        return bolt_space

    def __shaft_bolt_driver_opening_space__(self):
        if self.use_shaft_bolt:
            return self._shaft_bolt_driver_opening_space_template_()
        else:
            return None

    def _shaft_bolt_driver_opening_space_template_(self):
        m3x16_bolt = bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=False,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=3, BoltLength=16
            ),
        )
        horn_bolt_driver_space = (
            cq.Workplane(self.workplane_primary)
            .circle(
                m3x16_bolt.bolt_configs.BoltHeadDiam / 2
                + self.segment_configs.structural.SurfaceGive / 2
            )
            .extrude(
                self.motor_configs.ShaftBoltDriverInsertOpeningLength
                + self.motor_configs.ShaftBoltDriverInsertTunnelLengthZ
            )
        )
        return horn_bolt_driver_space
