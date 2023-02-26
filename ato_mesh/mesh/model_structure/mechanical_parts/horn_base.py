# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import abstractmethod

import cadquery as cq
from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh


class HornBase(cq_mesh.CqMesh):
    def __init__(
        self,
        install_from_side_instead_along: bool,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        motor_configs: segment_configuration.MotorConfigsBase = None,
        generate_rotation_space=True,
    ):
        self.horn_configs: segment_configuration.HornConfigsBase
        super().__init__(segment_configs=segment_configs, name=name)
        if motor_configs is None:
            self.motor_configs = segment_configs.actuator
        else:
            self.motor_configs = motor_configs
        self.motor_configs = motor_configs
        self.install_from_side_instead_along = install_from_side_instead_along
        self.install_bolt_from_side_instead_along = not install_from_side_instead_along
        self.generate_rotation_space = generate_rotation_space
        self.side_print_opening = segment_configs.horn_bolt_side_print_opening

    def model_mesh(self, add_surface_give=False):
        mesh = self.__mesh_template__(
            add_surface_give=add_surface_give, inclose_from_motor=add_surface_give
        )
        return mesh

    def space_mesh(self):
        space_mesh = self.model_mesh(add_surface_give=True).add(
            self.__horn_bolt_space__()
        )
        if self.generate_rotation_space:
            space_mesh = space_mesh.add(self.__horn_rotation_space__())

        return space_mesh

    def printable_mesh(self):
        return self.model_mesh(add_surface_give=True)

    def __relocate_bolt_on_horn__(self, bolt, offset):
        bolt = bolt.rotate(
            (0, 0, 0), (0, 1, 0), 180
        )  # rotate to have nut on the coup side
        if self.segment_configs.actuator_type == segment_configuration.MotorType.SERVO:
            roll_rotation = 90
        elif (
            self.segment_configs.actuator_type
            == segment_configuration.MotorType.STEPPER
        ):
            roll_rotation = -90
        bolt = bolt.rotate(
            (0, 0, 0), (0, 0, 1), roll_rotation
        )  # rotate to align insert
        bolt = bolt.translate(offset)
        return bolt

    @property
    @abstractmethod
    def horn_bolt(self):
        pass

    @abstractmethod
    def __mesh_template__(self, add_surface_give, inclose_from_motor):
        pass

    @abstractmethod
    def __horn_bolt_space__(self):
        pass

    # disk horns rotate in place
    # only applicable to arm horns
    def __horn_rotation_space__(self):
        return cq.Workplane(self.workplane_primary)
