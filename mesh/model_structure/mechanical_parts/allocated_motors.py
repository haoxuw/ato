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
from mesh.model_structure.mechanical_parts import motor_base
from mesh.model_structure.utilities import specification_selector


class AllocatedMotors(cq_mesh.CqMesh):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
        allocate_pitch=True,
        allocate_roll=True,
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        self.allocate_pitch = allocate_pitch
        self.allocate_roll = allocate_roll
        motor_mesh_class: motor_base.MotorBase = specification_selector.get_mesh_class(
            config=segment_configs.actuator
        )
        horn_mesh_class = specification_selector.get_mesh_class(
            config=segment_configs.motor_horn
        )
        self.pitch_motor_obj: motor_base.MotorBase = motor_mesh_class(
            segment_configs=segment_configs,
            horn_mesh_class=horn_mesh_class,
            install_from_side_instead_along=True,
            generate_motor_fastening_bolt_space=True,
            use_shaft_bolt=True,
        )
        self.roll_motor_obj: motor_base.MotorBase = motor_mesh_class(
            segment_configs=segment_configs,
            horn_mesh_class=horn_mesh_class,
            install_from_side_instead_along=False,
            generate_motor_fastening_bolt_space=False,
            use_shaft_bolt=False,
        )

    def supporting_mesh(self):
        return self.__allocate__(
            pitch=self.pitch_motor_obj.supporting_mesh(),
            roll=None,
        )

    def model_mesh(self, add_surface_give=False):
        return self.__allocate__(
            pitch=self.pitch_motor_obj.model_mesh(add_surface_give=add_surface_give),
            roll=self.roll_motor_obj.model_mesh(add_surface_give=add_surface_give),
        )

    def motor_fastening_bolt_space_mesh(self):
        return self.__allocate__(
            pitch=self.pitch_motor_obj.motor_fastening_bolt_space(),
            roll=None,
        )

    # for simplicity we consider the two motor as one object, and do BoneAugmented.cut(AllocatedMotors)
    # this runs the risk of unwanted interaction between combinations e.g. roll motor with pitch arm
    # refactor  might needed in the future to address this limitation by:
    # BonePitch.cut(MotorPitch)
    # BoneRoll.cut(MotorRoll)
    def space_mesh(self):
        return self.__allocate__(
            pitch=self.pitch_motor_obj.space_mesh(),
            roll=self.roll_motor_obj.space_mesh(),
        )

    def __allocate__(self, pitch, roll):
        mesh = cq.Workplane(self.workplane_primary)
        if self.allocate_pitch and pitch is not None:
            pitch = pitch.rotate((0, 0, 0), (0, 1, 0), 90)
            pitch = pitch.rotate((0, 0, 0), (1, 0, 0), 180)
            pitch = pitch.translate(
                (
                    -self.segment_configs.structural.PitchMotorOffsetX,
                    0,
                    self.segment_configs.structural.PitchCenterLocationZ,
                )
            )
            mesh.add(pitch)

        if self.allocate_roll and roll is not None:
            roll = roll.translate(
                (0, 0, self.segment_configs.structural.MotorTopLocation)
            )
            mesh.add(roll)

        return mesh
