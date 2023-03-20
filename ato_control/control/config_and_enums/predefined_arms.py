# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


from control.config_and_enums.arm_connection_config import (
    ActuatorPurpose,
    ArmConnectionAttributes,
    PiHeaderIdToBCM,
    ServoConnectionConfig,
)
from control.servo_ds3218 import ServoDs3218
from control.servo_ds5160 import ServoDs5160

# this object defines the particular arm config that I happen to be using now

# Copied from the StructuralConfigsSi's configuration when generating mesh
SegmentLength = 220
GripperLength = 147
# servo_max_position should be configured according to which flavor of the arm was assembled
# use (-130, 130) instead of (0, 270) on rotation_range to prevent damages from overshooting rotation
arm_6_axis = {
    ArmConnectionAttributes.ARM_NAME: "ARM_6_AXIS_270_DEG",
    ArmConnectionAttributes.URDF_FILENAME: "ato_3_seg_270_deg.urdf",
    0: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.TWENTY_NINE,
                ServoDs5160,
                270,
                1,
                (-135, 135),
            ),
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_ONE,
                ServoDs5160,
                270,
                1,
                (-130, 130),
            )
        ),
    },
    1: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.THIRTY_TWO,
                ServoDs5160,
                270,
                1,
                (-135, 135),
            )
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_THREE,
                ServoDs5160,
                270,
                1,
                (-130, 130),
            )
        ),
    },
    2: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.THIRTY_FIVE,
                ServoDs5160,
                270,
                1,
                (-135, 135),
            )
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_SIX,
                ServoDs5160,
                270,
                1,
                (-130, 130),
            )
        ),
    },
    -1: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: GripperLength,
        ActuatorPurpose.GRIPPER: ServoConnectionConfig(
            (
                ActuatorPurpose.GRIPPER,
                PiHeaderIdToBCM.FORTY,
                ServoDs3218,
                180,
                8,
                (-80, 0),
            )
        ),
    },
}


# Copied from the StructuralConfigsSi's configuration when generating mesh
SegmentLength = 200
GripperLength = 147
# servo_max_position should be configured according to which flavor of the arm was assembled
# use (-85, 85) instead of (0, 180) on rotation_range to prevent damages from overshooting rotation
arm_4_axis = {
    ArmConnectionAttributes.ARM_NAME: "ARM_6_AXIS_180_DEG",
    ArmConnectionAttributes.URDF_FILENAME: "ato_2_seg_180_deg.urdf",
    0: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.TWENTY_NINE,
                ServoDs5160,
                180,
                1,
                (-90, 90),
            ),
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_ONE,
                ServoDs5160,
                180,
                1,
                (-85, 85),
            )
        ),
    },
    1: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: SegmentLength,
        ActuatorPurpose.ROLL: ServoConnectionConfig(
            (
                ActuatorPurpose.ROLL,
                PiHeaderIdToBCM.THIRTY_TWO,
                ServoDs5160,
                180,
                1,
                (-90, 90),
            )
        ),
        ActuatorPurpose.PITCH: ServoConnectionConfig(
            (
                ActuatorPurpose.PITCH,
                PiHeaderIdToBCM.THIRTY_THREE,
                ServoDs5160,
                180,
                1,
                (-85, 85),
            )
        ),
    },
    -1: {
        ArmConnectionAttributes.PHYSICAL_LENGTH: GripperLength,
        ActuatorPurpose.GRIPPER: ServoConnectionConfig(
            (
                ActuatorPurpose.GRIPPER,
                PiHeaderIdToBCM.FORTY,
                ServoDs3218,
                180,
                8,
                (-80, 0),
            )
        ),
    },
}
