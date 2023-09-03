# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging

import numpy as np
from control import (
    arm_controller_joystick,
    ps4_joystick,
    raspberry_pi,
)
from control.config_and_enums.predefined_arms import arm_6_axis


def test_component_obj_creation():
    pi = raspberry_pi.RaspberryPi()
    joystick = ps4_joystick.Ps4Joystick(interface=f"/dev/input/js0")

    arm_ctl = arm_controller_joystick.ArmControllerJoystick(
        frame_rate=20,
        pi_obj=pi,
        joystick_obj=joystick,
        arm_segments_config=arm_6_axis,
    )
    assert arm_ctl.ready is True


def test_move_L3():
    pi = raspberry_pi.RaspberryPi()
    joystick = ps4_joystick.Ps4Joystick(interface=f"/dev/input/js0")

    arm_ctl = arm_controller_joystick.ArmControllerJoystick(
        frame_rate=20,
        pi_obj=pi,
        joystick_obj=joystick,
        arm_segments_config=arm_6_axis,
        auto_save_controller_states_to_file=False,
    )

    arm_ctl.start_threads(start_joystick_thread=False)

    # simulate joystick inputs
    seed = np.random.randint(0, 1000)
    logging.info(f"Using seed {seed}")
    np.random.seed(seed)
    for _ in range(1000):
        arm_ctl.joystick_obj.on_L3_right(
            value=np.random.random() * joystick.axis_max_value
        )
        arm_ctl.joystick_obj.on_L3_x_at_rest()
        arm_ctl.joystick_obj.on_L3_left(
            value=np.random.random() * joystick.axis_max_value
        )
        arm_ctl.joystick_obj.on_L3_up(
            value=np.random.random() * joystick.axis_max_value
        )
        arm_ctl.joystick_obj.on_L3_down(
            value=np.random.random() * joystick.axis_max_value
        )
    arm_ctl.stop_threads()
    return
