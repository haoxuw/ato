# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import ABC

import control


class InputDeviceInterface(ABC):
    def __init__(self):
        self._arm_controller_obj: control.arm_controller.ArmController = None

    # to handle new button release inputs (on opposed to persistent button holding)
    # we would call hooks to arm_controller_obj
    # an alternative is to gather state changes in the dict object, and expect arm to handle
    # but without semaphores, there could be issues handling new inputs in quick successions
    def connect_arm_controller(self, arm_controller_obj):
        self._arm_controller_obj = arm_controller_obj
