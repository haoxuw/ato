# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass


@dataclass
# matches raspberry pi header id to bcm number
# e.g. pin 29 is GPIO05, pin 30 is ground
# there seems to be no pattern to the mapping, but this is how raspberry pi hardware is built
class PiHeaderIdToBCM:
    TWENTY_NINE = 5
    THIRTY = -1  # ground
    THIRTY_ONE = 6
    THIRTY_TWO = 12
    THIRTY_THREE = 13
    THIRTY_FOUR = -1  # ground
    THIRTY_FIVE = 19
    THIRTY_SIX = 16
    THIRTY_SEVEN = 26
    THIRTY_EIGHT = 20
    THIRTY_NINE = -1  # ground
    FORTY = 21


@dataclass
class ArmConnectionAttributes:
    ARM_NAME = "ARM_NAME"
    URDF_FILENAME = "URDF_FILENAME"
    L2R2 = "L2R2"
    PHYSICAL_LENGTH = "PHYSICAL_LENGTH"


@dataclass
class ActuatorPurpose:
    ROLL = "ROLL"
    PITCH = "PITCH"
    GRIPPER = "GRIPPER"


@dataclass
class ServoConnectionConfig:
    def __init__(self, connection_def) -> None:
        (
            self.actuator_purpose,
            self.header_id,
            self.servo_class,
            self.servo_max_position,  # There are two versions of DS5160, rotates 0~180 and 0~270 respectively, with former supporting finer control but less range.
            self.velocity_magnifier,
            self.rotation_range,
        ) = connection_def
        self.installation_angle = -self.rotation_range[0]  # realign at logical zero
