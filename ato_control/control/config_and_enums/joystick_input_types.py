# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass


@dataclass
class Button:
    CROSS = "CROSS"
    TRIANGLE = "TRIANGLE"
    CIRCLE = "CIRCLE"
    SQUARE = "SQUARE"
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    L1 = "L1"
    R1 = "R1"
    L3 = "L3"
    R3 = "R3"


@dataclass
# each of those Joystick Axis gets input within [-limit,limit], instead of binary (press/release)
class JoystickAxis:
    LEFT_HORIZONTAL = "LEFT_HORIZONTAL"
    LEFT_VERTICAL = "LEFT_VERTICAL"
    RIGHT_HORIZONTAL = "RIGHT_HORIZONTAL"
    RIGHT_VERTICAL = "RIGHT_VERTICAL"
    L2R2 = "L2R2"
