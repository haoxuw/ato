# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import numbers
from abc import abstractmethod

from control.arm_control import raspberry_pi


class ServoInterface:
    def __init__(
        self,
        unique_name: str,
        header_id,
        pi_obj: raspberry_pi.RaspberryPi,
        installation_angle: float,
        servo_max_position=270,  # physically allowed
        rotation_range=(5, 175),  # logically allowed
    ):
        self.unique_name = unique_name

        self.header_id = header_id
        self.pi_obj: raspberry_pi.RaspberryPi = pi_obj

        # try to setup the arm as installation position to avoid sudden fast motion at startup
        self.__shaft_position = installation_angle
        self.__installation_angle = installation_angle
        self.__calibration_position = 0
        self.__calibration_delta = 0

        self.__servo_max_position = servo_max_position
        self.__rotation_range = rotation_range

    @property
    def calibration_delta(self):
        return self.__calibration_delta

    @property
    def shaft_position(self):
        return self.__shaft_position

    @property
    def payload_position(self):
        return (
            self.__shaft_position - self.__installation_angle + self.__calibration_delta
        )

    def set_calibration_delta(self, calibration_delta):
        self.__calibration_delta = calibration_delta

    def recalibrate(self):
        self.__calibration_delta = self.__installation_angle - self.__shaft_position

    @property
    def calibration_position(self):
        return self.__calibration_position

    def move_to_position_delta(
        self, position_delta, no_validation=False, show_info=False
    ):
        target_position = self.payload_position + position_delta
        self.move_to_position(
            position=target_position, no_validation=no_validation, show_info=show_info
        )
        return target_position

    @staticmethod
    @abstractmethod
    def _position_to_pulsewidth(position, servo_max_position):
        pass

    def move_to_position(
        self, position, no_validation=False, apply_calibration=True, show_info=False
    ):
        assert isinstance(position, numbers.Number), f"position={type(position)}"
        if not no_validation:
            position = self.get_valid_position(position)
        assert self.pi_obj is not None
        if show_info:
            logging.debug(
                f"moving {self.unique_name} of header {self.header_id} to position {position}"
            )
        if apply_calibration:
            position = position - self.calibration_delta
        self.__shaft_position = position + self.__installation_angle
        pulse = self._position_to_pulsewidth(
            position=self.__shaft_position, servo_max_position=self.__servo_max_position
        )
        if show_info:
            logging.debug(
                f"setting header # {self.header_id} @ logical_pos={position} shaft_pos={self.__shaft_position} dc={pulse}"
            )
        self.pi_obj.update_pulsewidth(
            pulsewidth=pulse,
            header_id=self.header_id,
        )
        return position

    def get_valid_position(self, position):
        position = min(position, self.__rotation_range[1], self.__servo_max_position)
        position = max(position, self.__rotation_range[0])
        return position

    def __str__(self) -> str:
        return f"{self.unique_name}+{self.calibration_delta:.2f}\n\t\t-- payload @{self.payload_position:.2f}"
