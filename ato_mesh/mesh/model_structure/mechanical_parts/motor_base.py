# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import abstractmethod

from mesh.configuration import segment_configuration
from mesh.cq_lib import cq_mesh
from mesh.model_structure.mechanical_parts import horn_base


class MotorBase(cq_mesh.CqMesh):
    def __init__(
        self,
        horn_mesh_class: horn_base.HornBase,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        motor_configs: segment_configuration.MotorConfigsBase = None,
        install_from_side_instead_along=True,
        generate_motor_fastening_bolt_space=None,
        use_shaft_bolt=True,
        add_wire_tunnel=False,
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        self.motor_configs: segment_configuration.MotorConfigsBase
        if motor_configs is None:
            self.motor_configs = segment_configs.actuator
        else:
            self.motor_configs = motor_configs

        # install_from_side_instead_along == is_pitch
        self.install_from_side_instead_along = install_from_side_instead_along
        # when motor is inserted from side, bolt would be inserted from along, vice versa
        self.install_bolt_from_side_instead_along = not install_from_side_instead_along

        if generate_motor_fastening_bolt_space is None:
            # by default, for arm actuators, generate if install_from_side (i.e. for pitch)
            # for roll, the motor is boxed inside the joint
            # for gripper
            self.generate_motor_fastening_bolt_space = install_from_side_instead_along
        else:
            self.generate_motor_fastening_bolt_space = (
                generate_motor_fastening_bolt_space
            )

        self.use_shaft_bolt = use_shaft_bolt
        self.horn_mesh_class = horn_mesh_class
        self.horn_mesh_obj: horn_base.HornBase = self.horn_mesh_class(
            segment_configs=segment_configs,
            install_from_side_instead_along=self.install_from_side_instead_along,
            motor_configs=self.motor_configs,
        )
        self.add_wire_tunnel = add_wire_tunnel

    @property
    def name(self):
        return f"{self.motor_configs.__class__.__name__}_{self.horn_mesh_obj.horn_configs.__class__.__name__}"

    def supporting_mesh(self):
        return self.__holder_space__()

    def model_mesh(self, add_surface_give=False):
        mesh = self.__mesh_template__(add_surface_give=add_surface_give)
        mesh = mesh.add(
            self.horn_mesh_obj.model_mesh(add_surface_give=add_surface_give)
        )
        return mesh

    def space_mesh(self):
        mesh = self.model_mesh(add_surface_give=True).add(self.optional_space())
        mesh = mesh.add(self.horn_mesh_obj.space_mesh())
        shaft_space_mesh = self.__shaft_bolt_driver_opening_space__()
        if shaft_space_mesh is not None:
            mesh = mesh.add(shaft_space_mesh)
        return mesh

    def printable_mesh(self):
        return self.model_mesh()

    @abstractmethod
    def optional_space(self):
        pass

    @abstractmethod
    def __holder_space__(self):
        pass

    @abstractmethod
    def __mesh_template__(self, add_surface_give):
        pass

    @abstractmethod
    def __shaft_bolt_driver_opening_space__(self):
        pass
