# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging

try:
    import pigpio

    UsingMock = False
except Exception:
    logging.warning(
        "Failed to import GPIO, assuming running outside of raspberry pi, use mock instead"
    )
    from unittest.mock import MagicMock

    pigpio = MagicMock()
    UsingMock = True

from control.interface_classes import control_board_interface


class RaspberryPi(control_board_interface.ControlBoardInterface):
    def __init__(self, frequency=50):
        self.pi_pwm = pigpio.pi()
        self.active_header_ids: set[int] = set()
        self.frequency = frequency

    def __config_pwm(self, gpio_id, frequency, dutycycle=None):
        try:
            self.pi_pwm.set_mode(gpio=gpio_id, mode=pigpio.OUTPUT)
            if frequency is not None:
                self.pi_pwm.set_PWM_frequency(user_gpio=gpio_id, frequency=frequency)
            if dutycycle is not None:
                self.pi_pwm.set_PWM_dutycycle(user_gpio=gpio_id, dutycycle=dutycycle)
        except AttributeError as e:
            logging.debug(f"Failed to communicate with GPIO, with exception: {e}")

    def __set_pulsewidth(self, user_gpio, pulsewidth):
        try:
            self.pi_pwm.set_servo_pulsewidth(user_gpio=user_gpio, pulsewidth=pulsewidth)
        except AttributeError as e:
            logging.debug(f"Failed to communicate with GPIO, with exception: {e}")
            logging.debug(
                "Assuming testing, otherwise, run `sudo pigpio` to start pigpio daemon. "
            )

    def __lazy_init_pwm(self, header_id) -> pigpio.pi:
        if header_id not in self.active_header_ids:
            gpio_id = self.__to_gpio_id(header_id)
            self.__config_pwm(gpio_id=gpio_id, frequency=self.frequency)
            self.active_header_ids.add(header_id)

    @staticmethod
    def __to_gpio_id(header_id):
        # by convention header_id < 0 means none GPIO pins, e.g. -1 is for ground
        assert isinstance(header_id, int) and header_id > 0
        return header_id

    def update_pulsewidth(self, pulsewidth, header_id):
        self.__lazy_init_pwm(header_id=header_id)
        gpio_id = self.__to_gpio_id(header_id)
        self.__set_pulsewidth(user_gpio=gpio_id, pulsewidth=pulsewidth)

    def turn_off(self, header_id, show_info=False):
        gpio_id = self.__to_gpio_id(header_id)
        self.__config_pwm(gpio_id=gpio_id, frequency=0, dutycycle=0)
        self.active_header_ids.remove(header_id)
        if show_info:
            logging.debug(f"Turned off {header_id}")
