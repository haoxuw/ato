# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import argparse
import logging

from mesh.configuration import arm_configuration, segment_configuration
from mesh.cq_lib import cq_mesh_shelf
from mesh.model_structure.arm import arm, mount
from mesh.model_structure.gripper import gripper_dragon, gripper_dragon_with_servo
from mesh.model_structure.mechanical_parts import (
    allocated_motors,
    bolt,
    horn_arm_25T,
    horn_disk_ds5160,
    horn_disk_nema17,
    motor_servo,
    motor_stepper,
)
from mesh.model_structure.segment import (
    bone_augmented,
    bone_separable,
    bone_separable_vertical,
    joint_augmented,
    segment_augmented,
)
from mesh.model_structure.segment.abstractions import (
    bone_abstract,
    joint_abstract,
    segment_abstract,
)
from mesh.model_structure.toys import gripper_dragon_on_wheels, wheels


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--abstract",
        action="store_true",
        help="If only print the abstract objects, this would remove functional components such as bolt sockets, motor holder etc.",
    )
    parser.add_argument(
        "--joint_polygon_sides",
        type=int,
        default=-1,
        help="If generate polygon joint, round if -1",
    )
    parser.add_argument(
        "--all", action="store_true", help="If export all models on shelf"
    )
    parser.add_argument(
        "--no_default",
        action="store_true",
        help="If not to print the default printable models, which includes a complete segment, separable bone, and gripper",
    )
    parser.add_argument(
        "--toys", action="store_true", help="If print the random selection of odd toys"
    )
    parser.add_argument("--bolt", action="store_true", help="If print bolt")
    parser.add_argument("--gripper", action="store_true", help="If print gripper")
    parser.add_argument("--motor", action="store_true", help="If print motor")
    parser.add_argument("--parts", action="store_true", help="If print whole")
    parser.add_argument("--bone", action="store_true", help="If print bone")
    parser.add_argument("--mount", action="store_true", help="If print mount")
    parser.add_argument(
        "--arm_include_motor_mesh",
        action="store_true",
        help="If print an assembled arm with motor_mesh",
    )
    parser.add_argument("--stepper", action="store_true")
    parser.add_argument("--servo", action="store_true")
    parser.add_argument("--arm", action="store_true", help="If print an assembled arm")

    parser.add_argument(
        "--arm10", action="store_true", help="If print an assembled 10-segment arm"
    )
    parser.add_argument(
        "--cylinder", action="store_true", help="If use cylinder roll holder"
    )
    parser.add_argument(
        "--separable_bone", action="store_true", help="If generate separable bone"
    )
    parser.add_argument(
        "--separable_bone_pitch_only",
        action="store_true",
        help="If only generate pitch separable bone",
    )
    parser.add_argument(
        "--gripper_servo_use_shaft_bolt",
        action="store_true",
        help="If generate installation space for a shaft bolt for the gripper",
    )
    parser.add_argument(
        "--tighten_bolt_socket",
        action="store_true",
        help="If print bolt socket that are smaller than bolt diameter, so that it the helix screws into the model.",
    )
    parser.add_argument(
        "--nozzle_diameter",
        type=float,
        default=0.4,
        help="The nozzle diameter used for 3D printing. This value will be used in asserts, to make sure intentional gaps are appropriate in relations to nozzle diameter.",
    )
    parser.add_argument(
        "--horn_bolt_length_override",
        type=int,
        default=None,
        help="If use a specified horn_bolt_length",
    )
    parser.add_argument(
        "--bone_length",
        type=int,
        default=67,
        help="Defaults bone length to 67, which is to be combined with joint radius == 33. We do not recommend to change the latter, which was defined to work with motor geometry.",
    )
    parser.add_argument(
        "--export", action="store_true", help="If export printable models on shelf"
    )
    parser.add_argument(
        "--export_all", action="store_true", help="If export all models on shelf"
    )
    parser.add_argument(
        "-o",
        "--out_dir",
        dest="out_dir",
        type=str,
        default="./generated",
        help="Where to put the generated resources",
    )
    parser.add_argument(
        "--no_axes",
        action="store_true",
        help="If not visualize axes (distinguished by length X:1 Y:2 Z:3)",
    )
    parser.add_argument(
        "--no_shelf",
        action="store_true",
        help="If not visualize shelf (i.e. collection of all the mesh)",
    )
    return parser.parse_args()


def add_models(cq_meshes, args):
    logging.info("Adding models to shelf for display and export...")
    if not args.stepper or args.servo:
        actuator_type = segment_configuration.MotorType.SERVO
    else:
        actuator_type = segment_configuration.MotorType.STEPPER

    segment_configs = segment_configuration.SegmentConfigs(
        bone_length=args.bone_length,
        actuator_type=actuator_type,
        joint_polygon_sides=args.joint_polygon_sides,
        nozzle_diameter=args.nozzle_diameter,
        roll_bone_holder_type=segment_configuration.RollBoneHolderType.CYLINDER
        if args.cylinder
        else segment_configuration.RollBoneHolderType.SPHERE,
        separable_bone=args.separable_bone,
        separable_bone_pitch_only=args.separable_bone_pitch_only,
        BoltSocketDiamTightenFactor=0.90 if args.tighten_bolt_socket else 1,
        HornBoltLengthOverride=args.horn_bolt_length_override,
        gripper_servo_use_shaft_bolt=args.gripper_servo_use_shaft_bolt,
    )
    if args.all or args.bolt:
        cq_meshes += [
            bolt.Bolt(
                segment_configs=segment_configs,
                install_from_side_instead_along=True,
                bolt_configs=segment_configuration.BoltConfigsBase(
                    BoltDiam=3, BoltLength=16
                ),
            )
        ]
    if args.all or args.motor:
        if actuator_type == segment_configuration.MotorType.SERVO:
            cq_meshes += [
                # pitch
                motor_servo.ServoMotor(
                    segment_configs=segment_configs,
                    install_from_side_instead_along=True,
                    horn_mesh_class=horn_disk_ds5160.HornDiskDs5160,
                    motor_configs=segment_configuration.ServoConfigsDs5160(),
                ),
                # roll
                motor_servo.ServoMotor(
                    segment_configs=segment_configs,
                    install_from_side_instead_along=False,
                    horn_mesh_class=horn_disk_ds5160.HornDiskDs5160,
                    motor_configs=segment_configuration.ServoConfigsDs5160(),
                ),
                # gripper
                motor_servo.ServoMotor(
                    segment_configs=segment_configs,
                    install_from_side_instead_along=False,
                    horn_mesh_class=horn_arm_25T.HornArm25T,
                    motor_configs=segment_configuration.ServoConfigsDs3218(),
                    generate_motor_fastening_bolt_space=True,
                ),
                allocated_motors.AllocatedMotors(segment_configs=segment_configs),
            ]
        elif actuator_type == segment_configuration.MotorType.STEPPER:
            cq_meshes += [
                motor_stepper.StepperMotor(
                    segment_configs=segment_configs,
                    install_from_side_instead_along=True,
                    horn_mesh_class=horn_disk_nema17.HornDiskNema17,
                ),
                motor_stepper.StepperMotor(
                    segment_configs=segment_configs,
                    install_from_side_instead_along=False,
                    horn_mesh_class=horn_disk_nema17.HornDiskNema17,
                ),
            ]
    if args.all or args.gripper:
        cq_meshes += [
            gripper_dragon_with_servo.GripperDragonWithServo(
                segment_configs=segment_configs
            )
        ]
    if args.all or args.parts:
        cq_meshes += [
            bone_augmented.BoneAugmented(segment_configs=segment_configs),
            bone_separable.BoneSeparable(segment_configs=segment_configs),
            joint_augmented.JointAugmented(segment_configs=segment_configs),
            joint_augmented.JointAugmentedPos(segment_configs=segment_configs),
            joint_augmented.JointAugmentedNeg(segment_configs=segment_configs),
        ]
    if args.all or args.bone:
        cq_meshes += [
            bone_augmented.BoneAugmented(segment_configs=segment_configs),
            bone_separable.BoneSeparable(segment_configs=segment_configs),
            bone_separable_vertical.BoneSeparableVertical(
                segment_configs=segment_configs
            ),
        ]
    if args.all or args.mount:
        cq_meshes += [
            mount.MountPos(segment_configs=segment_configs),
            mount.MountNeg(segment_configs=segment_configs),
        ]
    if args.all or args.abstract:
        cq_meshes += [
            bone_abstract.BoneAbstract(segment_configs=segment_configs),
            joint_abstract.JointAbstract(segment_configs=segment_configs),
            joint_abstract.JointAbstractPos(segment_configs=segment_configs),
            joint_abstract.JointAbstractNeg(segment_configs=segment_configs),
            segment_abstract.SegmentAbstract(segment_configs=segment_configs),
        ]
    if args.all or args.toys:
        cq_meshes += [
            wheels.Wheels(),
            gripper_dragon_on_wheels.GripperDragonOnWheels(
                segment_configs=segment_configs
            ),
        ]
    if args.all or args.arm:
        cq_meshes += [
            arm.Arm(
                name="4Segments",
                arm_configs=arm_configuration.ArmConfigsBase4Segments(),
                include_motor_mesh=args.arm_include_motor_mesh,
            ),
            arm.Arm(
                name="3Segments",
                arm_configs=arm_configuration.ArmConfigsBase3Segments(),
                include_motor_mesh=args.arm_include_motor_mesh,
            ),
            arm.Arm(
                name="2SegmentHalfJoint",
                arm_configs=arm_configuration.ArmConfigsBase2Segment(),
                half_joint=True,
                include_motor_mesh=True,
            ),
            arm.Arm(
                name="1Segment",
                arm_configs=arm_configuration.ArmConfigsBase1Segment(),
                half_joint=False,
                include_motor_mesh=True,
            ),
            arm.Arm(
                name="1SegmentHalfJoint",
                arm_configs=arm_configuration.ArmConfigsBase1Segment(),
                half_joint=True,
                include_motor_mesh=True,
            ),
        ]
    if args.arm10:
        cq_meshes += [
            arm.Arm(
                name="10Segments",
                arm_configs=arm_configuration.ArmConfigsBase10Segments(),
                include_motor_mesh=args.arm_include_motor_mesh,
            )
        ]
    if args.all or not args.no_default:
        cq_meshes += [
            segment_augmented.SegmentAugmented(segment_configs=segment_configs),
            bone_separable_vertical.BoneSeparableVertical(
                segment_configs=segment_configs
            ),
            gripper_dragon_with_servo.GripperDragonWithServo(
                segment_configs=segment_configs
            ),
            bone_separable_vertical.BoneSeparableVertical(
                segment_configs=segment_configs,
                separable_bone_pitch_only=True,
                name="pitch_only",
            ),
        ]
    return cq_meshes


def generate_mesh_shelf():
    args = get_args()

    cq_meshes = add_models(cq_meshes=[], args=args)

    shelf = cq_mesh_shelf.CqMeshShelf(
        cq_meshes=reversed(cq_meshes),
        distance=400,
        show_axes=not args.no_axes,
        out_dir=args.out_dir,
    )

    if args.export:
        shelf.export_each(only_printable_or_urdf=not args.export_all)
    if not args.no_shelf:
        shelf.export()


if __name__ == "__main__":
    generate_mesh_shelf()
