# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import argparse
import glob
import logging
import time

from control import (
    arm_controller_joystick,
    arm_controller_ml_training,
    ps4_joystick,
    raspberry_pi,
)
from control.config_and_enums.predefined_arms import arm_4_axis, arm_6_axis


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sanity_test",
        action="store_true",
        help="If immediately exit after initialization.",
    )
    parser.add_argument(
        "-j",
        "--input_js",
        type=int,
        dest="input_js",
        default=None,
        help="Specify the integer id of the js# device. e.g. 3 maps into /dev/input/js3",
    )
    parser.add_argument(
        "-v",
        "--disable_visualization",
        dest="visualize",
        action="store_false",
        help="If use simple matlab plots to visualize the arm.",
    )
    parser.add_argument(
        "--generate_training_data",
        type=int,
        default=0,
        help="Do not run the arm, but generate training data.",
    )
    parser.add_argument(
        "--frame_rate",
        type=int,
        default=100,
    )
    parser.add_argument(
        "--evaluation_unit",
        type=int,
        default=128,
    )
    parser.add_argument(
        "--training_data_filepath_prefix",
        type=str,
        default="~/.ato/training_data",
        help="Add a suffix to the filename of generated training data.",
    )
    arm_configs = ["arm_4_axis", "arm_6_axis"]
    parser.add_argument(
        "--arm_segments_config",
        type=str,
        choices=arm_configs,
        default="arm_6_axis",
        help=f"Select a valid arm config from {arm_configs}",
    )
    return parser.parse_args()


def resolve_input_js(args):
    try:
        if args.input_js is None:
            devices = glob.glob("/dev/input/js*")
            devices.sort()
            return devices[-1]
        else:
            return f"/dev/input/js{str(args.input_js)}"
    except Exception as e:
        logging.warning(
            f"Failed to resolve the joystick device /dev/input/js*, exception: {e}"
        )


def create_arm_controller_obj(args, for_training=False):
    if args.arm_segments_config == "arm_4_axis":
        arm_segments_config = arm_4_axis
    elif args.arm_segments_config == "arm_6_axis":
        arm_segments_config = arm_6_axis
    else:
        raise Exception(
            f"Unsupported arm_segments_config == {args.arm_segments_config}"
        )

    pi = raspberry_pi.RaspberryPi()
    input_js = resolve_input_js(args=args)
    if input_js is None:
        raise Exception("No joystick controller device found.")
    joystick = ps4_joystick.Ps4Joystick(interface=input_js)
    if for_training:
        arm_ctl = arm_controller_ml_training.ArmControllerMlTraining(
            frame_rate=args.frame_rate,
            pi_obj=pi,
            joystick_obj=joystick,
            arm_segments_config=arm_segments_config,
        )
    else:
        arm_ctl = arm_controller_joystick.ArmControllerJoystick(
            frame_rate=args.frame_rate,
            pi_obj=pi,
            joystick_obj=joystick,
            arm_segments_config=arm_segments_config,
        )
    return arm_ctl


def main():
    args = get_args()

    if args.sanity_test:
        arm_ctl = create_arm_controller_obj(
            args=args,
        )
        arm_ctl.start_threads(start_joystick_thread=False)
        arm_ctl.joystick_obj.on_L3_right(value=10000)
        time.sleep(0.1)  # 0.1s
        arm_ctl.joystick_obj.on_L3_x_at_rest()
    elif args.generate_training_data > 0:
        arm_ctl = create_arm_controller_obj(args=args, for_training=True)
        arm_ctl.generate_training_data(
            size=args.generate_training_data,
            training_data_filepath_prefix=args.training_data_filepath_prefix,
        )
    else:
        arm_ctl = create_arm_controller_obj(
            args=args,
        )
        arm_ctl.start_threads()
        if args.visualize:
            arm_ctl.visualize_joints_pibullet()
            arm_ctl.visualize_joints_matplotlib()
    return


if __name__ == "__main__":
    main()
