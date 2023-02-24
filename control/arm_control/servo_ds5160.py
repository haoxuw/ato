# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging

from control.arm_control.interface_classes import servo_interface


class ServoDs5160(servo_interface.ServoInterface):
    @staticmethod
    def _position_to_pulsewidth(position, servo_max_position):
        if servo_max_position == 270:
            position_0_pulsewidth = 500  # from DS5160-SSG spec
            position_max_pulsewidth = 2500  # from spec
            position_max = 270
        elif servo_max_position == 180:
            position_0_pulsewidth = 500
            position_max_pulsewidth = 2500
            position_max = 180
        else:
            assert (
                False
            ), f"Unexpected self.__servo_max_position == {self.__servo_max_position}"
        # assert position>=0 and position<=position_max
        dc_per_position = (
            position_max_pulsewidth - position_0_pulsewidth
        ) / position_max
        pulsewidth = position * dc_per_position + position_0_pulsewidth
        logging.debug(f"Translated {position} degree into {pulsewidth}%")
        if pulsewidth < 500 or pulsewidth > 2500:
            cropped_pulsewidth = max(min(pulsewidth, 2500), 500)
            logging.warning(
                f"Translating {position} degree into {pulsewidth} out of bound, cropped into {cropped_pulsewidth}"
            )
            pulsewidth = cropped_pulsewidth
        return pulsewidth
