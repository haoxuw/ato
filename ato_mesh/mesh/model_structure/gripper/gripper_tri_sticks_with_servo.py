# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.configuration import segment_configuration
from mesh.model_structure.gripper import gripper_tri_sticks
from mesh.model_structure.mechanical_parts import horn_arm_25T, motor_servo


class GripperTriSticksWithServo(gripper_tri_sticks.GripperTriSticks):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        self.motor_obj = motor_servo.ServoMotor(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=False,
            horn_mesh_class=horn_arm_25T.HornArm25T,
            motor_configs=segment_configuration.ServoConfigsDs3218(),
            generate_motor_fastening_bolt_space=set([0, 3]),
            use_shaft_bolt=segment_configs.gripper_servo_use_shaft_bolt,
        )

    def model_mesh(self, add_surface_give=False):
        dragon_mesh = super().model_mesh(add_surface_give=add_surface_give)
        # # todo 
        # return dragon_mesh

        servo_support = self._relocate(self.motor_obj.supporting_mesh())
        # servo_mesh = self._relocate(self.motor_obj.model_mesh())
        servo_space = self._relocate(self.motor_obj.space_mesh())
        return dragon_mesh.add(servo_support).cut(servo_space)
