# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import ABC, abstractmethod


# shows the API of ArmController
class ArmControllerInterface(ABC):
    @abstractmethod
    def get_joystick_input_states(self):
        pass

    @abstractmethod
    def reset_joystick_input_states(self):
        pass

    @abstractmethod
    def is_thread_running(self):
        pass

    @abstractmethod
    def save_controller_states(self, folder, filename):
        pass

    @abstractmethod
    def load_controller_states(self, folder, filename):
        pass

    @abstractmethod
    def recalibrate_servos(self):
        pass

    @abstractmethod
    def replay_trajectory(self):
        pass

    @abstractmethod
    def change_segment_pointer(self, delta):
        pass

    @abstractmethod
    def change_velocity_level(self, delta):
        pass

    @abstractmethod
    def move_servos_to_calibration_position(self):
        pass

    @abstractmethod
    def start_saving_trajectory(self):
        pass

    @abstractmethod
    def save_trajectory(self):
        pass

    @abstractmethod
    def start_threads(self, start_joystick_thread=True):
        pass

    @abstractmethod
    def stop_threads(self):
        pass

    @abstractmethod
    def start_arm_controller_thread(self):
        pass

    @abstractmethod
    def stop_arm_controller_thread(self):
        pass
