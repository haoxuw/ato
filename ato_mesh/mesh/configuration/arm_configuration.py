# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass
from typing import Tuple

from mesh.configuration.segment_configuration import MotorType, SegmentConfigs


@dataclass
class ArmConfigsBase:
    NumSegments: int
    BoneLengths: Tuple[float]
    # no roll supported
    PitchAngles: Tuple[int]
    MotorTypes: Tuple[str]
    JointPolygonSides: Tuple[int]

    @property
    def segments_configs(self):
        assert len(self.BoneLengths) == self.NumSegments
        assert len(self.PitchAngles) == self.NumSegments
        assert len(self.MotorTypes) == self.NumSegments
        assert len(self.JointPolygonSides) == self.NumSegments
        return [
            SegmentConfigs(
                bone_length=self.BoneLengths[index],
                pitch_angle=self.PitchAngles[index],
                actuator_type=self.MotorTypes[index],
                joint_polygon_sides=self.JointPolygonSides[index],
            )
            for index in range(self.NumSegments)
        ]


@dataclass
class ArmConfigsBase10Segments(ArmConfigsBase):
    NumSegments: int = 10
    BoneLengths: Tuple[float] = (80, 80, 80, 80, 80, 80, 80, 80, 80, 80)
    # no roll supported
    PitchAngles: Tuple[int] = (-45, 45, 45, 45, 45, -45, -45, 45, 45, -45)
    MotorTypes: Tuple[str] = (
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
    )
    JointPolygonSides: Tuple[int] = (-1, 6, 8, 10, 12, 14, 16, 18, 20, 32)


@dataclass
class ArmConfigsBase4Segments(ArmConfigsBase):
    NumSegments: int = 4
    BoneLengths: Tuple[float] = (60, 90, 90, 180)
    # no roll supported
    PitchAngles: Tuple[int] = (-30, 80, 60, 70)
    MotorTypes: Tuple[str] = (
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
        MotorType.SERVO,
    )
    JointPolygonSides: Tuple[int] = (-1, 18, 12, 6)


@dataclass
class ArmConfigsBase3Segments(ArmConfigsBase):
    NumSegments: int = 3
    BoneLengths: Tuple[float] = (60, 90, 90)
    # no roll supported
    PitchAngles: Tuple[int] = (60, 80, -50)
    MotorTypes: Tuple[str] = (MotorType.SERVO, MotorType.SERVO, MotorType.SERVO)
    JointPolygonSides: Tuple[int] = (-1, 12, 6)


@dataclass
class ArmConfigsBase2Segment(ArmConfigsBase):
    NumSegments: int = 2
    BoneLengths: Tuple[float] = (
        90,
        90,
    )
    # no roll supported
    PitchAngles: Tuple[int] = (
        0,
        0,
    )
    MotorTypes: Tuple[str] = (
        MotorType.SERVO,
        MotorType.SERVO,
    )
    JointPolygonSides: Tuple[int] = (-1, -1)


@dataclass
class ArmConfigsBase1Segment(ArmConfigsBase):
    NumSegments: int = 1
    BoneLengths: Tuple[float] = (60,)
    # no roll supported
    PitchAngles: Tuple[int] = (0,)
    MotorTypes: Tuple[str] = (MotorType.SERVO,)
    JointPolygonSides: Tuple[int] = (-1,)
