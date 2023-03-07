# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass

from control import servo_ds3218, servo_ds5160


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
class SegmentConfigTypes:
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
            self.installation_angle,
            self.rotation_range,
        ) = connection_def


# use (-130, 130) instead of (0, 180) on rotation_range to prevent damages from overshooting rotation

# this object defines the particular arm config that I happen to be using now
# Copied from the StructuralConfigsSi's configuration
SegmentLength = 210
GripperLength = 147
# servo_max_position should be configured according to which flavor of the arm was assembled
arm_segments_config = {
    0: {
        SegmentConfigTypes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.TWENTY_NINE,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-90, 180),
            ),
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_ONE,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-130, 130),
            )
        ),
    },
    1: {
        SegmentConfigTypes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.THIRTY_TWO,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-90, 180),
            )
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_THREE,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-130, 130),
            )
        ),
    },
    2: {
        SegmentConfigTypes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.THIRTY_FIVE,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-90, 180),
            )
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_SIX,
                servo_ds5160.ServoDs5160,
                270,
                1,
                90,
                (-130, 130),
            )
        ),
    },
    -1: {
        SegmentConfigTypes.PHYSICAL_LENGTH: GripperLength,
        ActuatorPurpose.GRIPPER: ServoConnectionConfig(
            (
                ActuatorPurpose.GRIPPER,
                PiHeaderIdToBCM.FORTY,
                servo_ds3218.ServoDs3218,
                180,
                8,
                80,
                (-80, 0),
            )
        ),
    },
}
