# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import (
    horn_arm_25T,
    horn_disk_ds5160,
    horn_disk_nema17,
    motor_servo,
    motor_stepper,
)


def get_mesh_class(config):
    """
    Helper class to map config dataclass into mess class
    """
    if isinstance(
        config,
        (
            segment_configuration.ServoConfigsDs3218,
            segment_configuration.ServoConfigsDs5160,
        ),
    ):
        # ServoMotor supports both Ds3218 and Ds5160 which are similar enough
        return motor_servo.ServoMotor
    elif isinstance(config, segment_configuration.StepperConfigsNema17x38):
        return motor_stepper.StepperMotor
    elif isinstance(config, segment_configuration.HornDiskConfigsDs5160):
        return horn_disk_ds5160.HornDiskDs5160
    elif isinstance(config, segment_configuration.HornArmConfigs25T):
        return horn_arm_25T.HornArm25T
    elif isinstance(config, segment_configuration.HornDiskConfigsNema17):
        return horn_disk_nema17.HornDiskNema17
    raise Exception(
        f"Expecting a known configuration which can be mapped to a mesh class, but got {type(config)}"
    )
