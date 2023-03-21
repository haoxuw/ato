# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
from dataclasses import dataclass
from typing import Tuple


@dataclass
class MotorType:
    STEPPER = "Stepper Motor"
    SERVO = "Servo Motor"


@dataclass
class MotorHornType:
    DISK = "Disk Horn"
    ARM = "Arm Horn"


@dataclass
class RollBoneHolderType:
    SPHERE = "Sphere"
    CYLINDER = "Cylinder"
    NONE = "Do Not Generate"


# since this project is named ATO(mic), let's name the first build (Silicon)
# The Si180 version supports 180 degree pitch range, the 270 version supports 270 degrees, while requiring a longer joint
@dataclass
class StructuralConfigsSi180:
    # to understand the what those variables represent, please refer to
    # https://github.com/haoxuw/ato/blob/master-resources/images/illustrations/segment_config_diagram.png
    # length units are in mm:

    ##########################################
    ## values derived, to be assigned in __init__
    BoneLength: float
    PitchBoneMeshNetLength: float
    RollBoneMeshNetLength: float
    SegmentLength: float
    ##########################################

    ##########################################
    ## values hard-coded to best fit the Ds5160 Servo and its HornDisk geometry:
    JointRadius: float = 33
    BoneRadius: float = 16.5

    # Used as value to represent "far away coordinates", but not too far, so we can still visualize
    Far: float = 142
    VeryFar: float = Far * 10
    ##########################################

    ##########################################
    ## values that are configurable:
    JointLength: float = 110
    PitchRange: float = 180
    MotorTopLocation: float = 70

    PitchMotorOffsetX: float = 25  # larger is outwards
    PitchCenterLocationZ: float = (
        0  # 0: pitch axis is aligned with joint sphere's center, z: move inwards z mm
    )
    ##########################################

    BoneHolderRadius: float = BoneRadius * 1.4

    # RotateHolder is what locks roll bone in the joint, which allows rotation
    # When set to sphere, breath = radius
    # When set to cylinder, depth measures furthest point of holder to bone, i.e. depth = CylinderRadius - BoneRadius
    # meanwhile breath measures height of the cylinder
    # those values/ratio are set arbitrarily
    RollCylinderHolderDepth: float = BoneHolderRadius - BoneRadius
    RollCylinderHolderBreadth: float = BoneHolderRadius

    # when RollHolderOffsetZ == 0 the holder's edge would be touching roll motor
    # this offset move the holder outwards
    RollHolderOffsetZ: float = 15

    RollLubricantOpeningWidth: float = BoneHolderRadius

    # coefficients are empirical, and was used to calculate offsets of the bolts binding two pats of the joints
    BoltOffsetCoefficientX: float = 1.58  # less is outwards
    BoltOffsetCoefficientZTop: float = 1.74  # less is upwards
    BoltOffsetCoefficientZDown: float = 1.74  # less is downwards

    SurfaceGive: float = 0.3
    MountUnitSize: float = 90

    JointSphereHolders = [
        (
            (
                BoneRadius + RollCylinderHolderDepth * 1.4,
                0,
                MotorTopLocation + RollHolderOffsetZ / 2,
            ),
            5,
        ),
        (
            (
                -BoneRadius - RollCylinderHolderDepth * 1.4,
                0,
                MotorTopLocation + RollHolderOffsetZ / 2,
            ),
            5,
        ),
        (
            (
                (BoneRadius + RollCylinderHolderDepth + JointRadius) / 2,
                0,
                BoneRadius * 4 / 5,
            ),
            3,
        ),
        (((BoneRadius + RollCylinderHolderDepth + JointRadius) / 2, 0, 0), 3),
    ]  # offset_x, offset_y, radius

    def __init__(
        self, bone_length: float, roll_bone_holder_type: RollBoneHolderType
    ) -> None:
        # <BoneLength> measures the net length of bones visible and functional
        # this is because a section of roll bone is always inside of the joint
        # length of that portion depends on length of joint and location of motor
        # so the printed bone is longer than bone_length
        self.BoneLength = bone_length
        self.PitchBoneMeshNetLength = self.JointRadius
        self.RollBoneMeshNetLength = (
            self.BoneLength + self.JointRadius * 2 - self.PitchBoneMeshNetLength
        )

        self.RollBoneHolderType = roll_bone_holder_type
        self.SegmentLength = (
            self.BoneLength
            + self.JointRadius
            + self.JointLength
            - self.PitchCenterLocationZ
        )  # == RollBoneEdgeToPitchCenterLocation

    def __post_init__(self):
        assert self.BoneRadius < self.JointRadius
        assert (
            self.PitchBoneMeshNetLength + self.RollBoneMeshNetLength
            == self.BoneLength + self.JointRadius * 2
        )
        self._minimal_dimension_requirements()

    def _minimal_dimension_requirements(self):
        assert (
            self.JointLength >= 110
        ), "Joint length too short will results in insufficient space to house the DS1560 servo motors"


# The 270 version requires a longer joint length than 180 to house pitch space
@dataclass
class StructuralConfigsSi270(StructuralConfigsSi180):
    def __init__(
        self, bone_length: float, roll_bone_holder_type: RollBoneHolderType
    ) -> None:
        super().__init__(bone_length, roll_bone_holder_type)

    JointLength: float = 130
    PitchRange: float = 270
    MotorTopLocation: float = 90
    BoltOffsetCoefficientX: float = 1.58  # less is outwards
    BoltOffsetCoefficientZTop: float = 1.64  # less is upwards
    BoltOffsetCoefficientZDown: float = 1.74  # less is downwards

    JointRadius: float = 33
    BoneRadius: float = 16.5

    BoneHolderRadius: float = BoneRadius * 1.4
    RollCylinderHolderDepth: float = BoneHolderRadius - BoneRadius
    RollHolderOffsetZ: float = 15

    JointSphereHolders = [
        (
            (
                BoneRadius + RollCylinderHolderDepth * 1.4,
                0,
                MotorTopLocation + RollHolderOffsetZ / 2,
            ),
            5,
        ),
        (
            (
                -BoneRadius - RollCylinderHolderDepth * 1.4,
                0,
                MotorTopLocation + RollHolderOffsetZ / 2,
            ),
            5,
        ),
        (
            (
                (BoneRadius + RollCylinderHolderDepth + JointRadius) / 2,
                0,
                BoneRadius * 4 / 5,
            ),
            3,
        ),
        (((BoneRadius + RollCylinderHolderDepth + JointRadius) / 2, 0, 0), 3),
    ]  # offset_x, offset_y, radius


@dataclass
class BoltConfigsBase:
    BoltLength: float
    BoltDiam: float
    BoltInsertSpaceLength: float
    BoltHeadHeight: float
    BoltHeadDiam: float
    NutHexDiam: float
    NutHeight: float

    def __init__(self, BoltDiam, BoltLength):
        self.BoltLength = BoltLength
        self.BoltDiam = BoltDiam
        if BoltDiam == 2:
            self.BoltHeadHeight = 1.6
            self.BoltHeadDiam = 3.2
            self.NutHexDiam = 4.05
            self.NutHeight = 1.3
        elif BoltDiam == 3:
            self.BoltHeadHeight = 3.2  # over shoot a bit
            self.BoltHeadDiam = 5.5
            self.NutHexDiam = 5.5
            self.NutHeight = 2
        else:
            raise Exception(
                f"Not support bolts with M{BoltDiam}X{BoltLength}, to add a new spec measure the appropriate BoltHeadHeight etc."
            )
        self.BoltInsertSpaceLength = BoltLength * 2
        # make BoltDiam socket tighter


@dataclass
class HornConfigsBase:
    pass


@dataclass
class HornDiskConfigsNema17(HornConfigsBase):
    Type: MotorHornType = MotorHornType.DISK
    # used to calculate both Motor location and roll horn bolt location
    HornHeight: float = 2
    # HornInnerOffset == the offset where bolt should be
    HornInnerOffset: float = 22
    HornCapHeight: float = 0
    HornHeightBoundary: float = HornInnerOffset + HornHeight + HornCapHeight

    HornRadius: float = 11.25

    # measured using opposite pairs, when horns set as "+"" not 2 rows
    HornBoltDist: float = 11
    HornBoltRadius: float = HornBoltDist / 2

    # Nema horn disk has shaft bolt on the side
    HornShaftBoltDriverSpaceRadius: float = 3
    # measured
    HornShaftBoltDriverSpaceCenterOffset: float = 8


@dataclass
class HornDiskConfigsDs5160(HornConfigsBase):
    Type: MotorHornType = MotorHornType.DISK
    HornHeight: float = 5
    HornRadius: float = 15.2
    # HornInnerOffset measures distance the motor and inner side of the horn, where shaft is exposed
    # HornInnerOffset == the offset where the horn bolts should be
    HornInnerOffset: float = 2

    # on the horn I own, there is an extra cylinder acting as the cap to cover the shaft gears, we need to leave a socket for it
    HornCapHeight: float = 2
    HornCapRadius: float = 6 + 0.1
    HornHeightBoundary: float = HornInnerOffset + HornHeight + HornCapHeight

    HornBoltLength: float = 40

    HornBottomPlaneRadius: float = 9
    HornBottomPlaneHeight: float = 1

    # measured using opposite pairs, when horns set as "*" not 4 rows
    HornBoltDist: float = 24
    HornBoltRadius: float = HornBoltDist / 2


@dataclass
class HornArmConfigs25T(HornConfigsBase):
    Type: MotorHornType = MotorHornType.ARM
    HornInnerOffset: float = 2
    HornHeight: float = 6
    HornHeightBoundary: float = HornInnerOffset + HornHeight

    # the horn arm looks somewhere between O= to OD
    # meaning a circular base plane i.e. "O" around the shaft, which transition into an arm reached out i.e. "=" or "D"
    # "0" radius: HornRadius
    # ">" vertical: (HornArmBottomWidth, HornArmWidth) horizontal: HornArmLength
    HornRadius: float = 7.5
    HornArmReach: float = 28
    # modeling the arm as circle O plus a trapezoid D
    HornArmBottomWidth: float = HornRadius * 2  # width of the left edge from D
    HornArmTopWidth: float = 8  # width of the right edge of D

    # [shaft_bolt, mounting_bolt_1, mounting_bolt_2]
    BoltLocations: Tuple[float] = (
        24,
        # 20, # no space to install the second nut
    )

    # with 0 and 180 pointing sidewards, when viewing from bottom towards up, 0 towards left, 90 along the motor front (wire side)
    ArmRotationInitialAngle: float = 68
    ArmRotationRange: Tuple[float] = (
        0,
        ArmRotationInitialAngle,
    )  # (min, max) in degrees or None, which means (0, 180)


@dataclass
class MotorConfigsBase:
    pass


@dataclass
class StepperConfigsNema17x38(MotorConfigsBase):
    type: MotorType = MotorType.STEPPER

    Radius: float = 21  # object size + surface space
    HexRadius: float = 17.5
    Height: float = 38

    MotorFastenerGirth: float = 3
    MotorFastenerHeight: float = 10


@dataclass
class ServoConfigsDs3218(MotorConfigsBase):
    type: MotorType = MotorType.SERVO

    ### from datasheet:
    Length: float = 40
    Width: float = 20
    Height: float = 40
    MotorFastenerEdgeDistX: float = 54.5 + 0.5
    MotorFastenerCenterDistX: float = 49.5
    MotorFastenerBoltSocketHeight: float = 40.4 - 27.7
    MotorFastenerBoltYDist: float = 10.0
    ### end of datasheet

    MotorFastenerBoltXDist: float = MotorFastenerCenterDistX

    MotorFastenerBoltSocketZBot: float = MotorFastenerBoltSocketHeight + 0.5
    MotorFastenerBoltSocketZTop: float = MotorFastenerBoltSocketZBot - 6
    MotorFastenerBoltSocketXExtra: float = (MotorFastenerEdgeDistX - Length) / 2

    MotorFastenerGirth: float = 3
    MotorFastenerHeight: float = Height / 3

    MotorFastenerBoltX: float = (MotorFastenerBoltXDist - Length) / 2
    MotorFastenerBoltY: float = MotorFastenerBoltYDist / 2
    MotorFastenerBoltZ: float = MotorFastenerBoltSocketZBot

    WireTunnelExitExtraX: float = MotorFastenerBoltSocketXExtra
    WireTunnelHeight: float = 2
    WireTunnelWidth: float = WireTunnelHeight * 3  # 3 wires: Pos Neg Sig

    ShaftRadius: float = 4
    # furthest point where shaft touches the bone

    # measured, not in datasheet
    ShaftCenterOffsetX: float = 10

    # to install the shaft bolt on roll bone, we have to first install horn on bone before installing horn on servo
    # so a driver is needed to be lowed inside of the bone
    # 57 was measured from a allen key driver for M2
    ShaftBoltDriverLength: float = 57
    ShaftBoltDriverDim: float = 3.2
    ShaftBoltHeadRadius: float = 5.5
    # ShaftBoltDriverInsertOpeningLength >= length of the bolt driver
    # With driver known, we need an opening space on bone to insert the driver
    ShaftBoltDriverInsertOpeningLength: float = 14
    ShaftBoltDriverInsertTunnelLengthZ: float = ShaftBoltDriverLength - 2

    ExtrusionPresents: bool = False

    def __post_init__(self):
        assert self.ShaftBoltDriverInsertTunnelLengthZ <= self.ShaftBoltDriverLength


@dataclass
class ServoConfigsDs5160(MotorConfigsBase):
    # 5160 5180 51150 seems to be the same
    type: MotorType = MotorType.SERVO

    ### from datasheet:
    Length: float = 65
    Width: float = 30
    Height: float = 48
    MotorFastenerEdgeDistX: float = 81.5
    MotorFastenerCenterDistX: float = 75
    MotorFastenerBoltSocketHeight: float = 48 - 31
    MotorFastenerBoltYDist: float = 17.0
    ### end of datasheet

    MotorFastenerBoltXDist: float = MotorFastenerCenterDistX

    MotorFastenerBoltSocketZBot: float = MotorFastenerBoltSocketHeight + 0.5
    MotorFastenerBoltSocketZTop: float = MotorFastenerBoltSocketZBot - 6
    MotorFastenerBoltSocketXExtra: float = (MotorFastenerEdgeDistX - Length) / 2

    MotorFastenerGirth: float = 4
    MotorFastenerHeight: float = Height / 2

    MotorFastenerBoltX: float = (MotorFastenerBoltXDist - Length) / 2
    MotorFastenerBoltY: float = MotorFastenerBoltYDist / 2
    MotorFastenerBoltZ: float = MotorFastenerBoltSocketZBot

    WireTunnelExitExtraX: float = MotorFastenerBoltSocketXExtra
    WireTunnelHeight: float = 2
    WireTunnelWidth: float = WireTunnelHeight * 3  # 3 wires: Pos Neg Sig

    ShaftRadius: float = 4
    # furthest point where shaft touches the bone

    # measured, not in datasheet
    ShaftCenterOffsetX: float = 20

    # to install the shaft bolt on roll bone, we have to first install horn on bone before installing horn on servo
    # so a driver is needed to be lowed inside of the bone
    # 57 was measured from a allen key driver for M2
    ShaftBoltDriverLength: float = 57
    ShaftBoltDriverDim: float = 3.2
    ShaftBoltHeadRadius: float = 5.5
    # ShaftBoltDriverInsertOpeningLength >= length of the bolt driver
    # With driver known, we need an opening space on bone to insert the driver
    ShaftBoltDriverInsertOpeningLength: float = 14
    ShaftBoltDriverInsertTunnelLengthZ: float = ShaftBoltDriverLength - 2

    ExtrusionPresents: bool = True
    # for some reason there is an extrusion on the bottom of servo
    ExtrusionHeight: float = 1
    ExtrusionRadius: float = 3
    ExtrusionOffsetY: float = -6

    def __post_init__(self):
        assert self.ShaftBoltDriverInsertTunnelLengthZ <= self.ShaftBoltDriverLength


# to be refactored, separate segment config outside of arm config
class SegmentConfigs:
    def __init__(
        self,
        actuator_type,
        joint_polygon_sides,
        roll_bone_holder_type=RollBoneHolderType.SPHERE,
        bone_length=47,
        pitch_range_180=False,
        allocate_with_pitch_angle=0,  # only for demonstrating the arm assembly
        nozzle_diameter=0.4,  # 0.4mm
        horn_bolt_side_print_opening=True,
        separable_bone=True,
        separable_bone_pitch_only=False,
        BoltSocketDiamTightenFactor=0.85,
        HornBoltLengthOverride=None,
        gripper_servo_use_shaft_bolt=False,
    ):
        self.nozzle_diameter = nozzle_diameter
        self.horn_bolt_side_print_opening = horn_bolt_side_print_opening
        self.joint_polygon_sides = joint_polygon_sides
        self.allocate_with_pitch_angle = allocate_with_pitch_angle
        self.separable_bone = separable_bone
        self.separable_bone_pitch_only = separable_bone_pitch_only

        if pitch_range_180:
            self.structural = StructuralConfigsSi180(
                bone_length=bone_length,
                roll_bone_holder_type=roll_bone_holder_type,
            )
        else:
            self.structural = StructuralConfigsSi270(
                bone_length=bone_length,
                roll_bone_holder_type=roll_bone_holder_type,
            )

        self.BoltSocketDiamTightenFactor: float = BoltSocketDiamTightenFactor

        self.actuator_type = actuator_type
        if actuator_type == MotorType.STEPPER:
            logging.warning(
                f"Code for building a stepper-based arm is deprecated, due to its practical limitations."
            )
            self.actuator = StepperConfigsNema17x38()
            self.motor_horn = HornDiskConfigsNema17()
        elif actuator_type == MotorType.SERVO:
            self.actuator = ServoConfigsDs5160()
            self.motor_horn = HornDiskConfigsDs5160()
        else:
            raise Exception(f"Unsupported actuator_type == {actuator_type}")

        if HornBoltLengthOverride is not None:
            self.motor_horn.HornBoltLength = HornBoltLengthOverride
        self.gripper_servo_use_shaft_bolt = gripper_servo_use_shaft_bolt
        logging.info(self.__repr__())

    def __repr__(self) -> str:
        return f"actuator_type: {self.actuator_type}, configs:\n\n{self.structural}\n\n{self.actuator}\n"
