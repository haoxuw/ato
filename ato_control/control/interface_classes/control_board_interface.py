# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from abc import ABC, abstractmethod


class ControlBoardInterface(ABC):
    @abstractmethod
    def update_pulsewidth(self, pulsewidth, header_id):
        pass

    @abstractmethod
    def turn_off(self, header_id, show_info=False):
        pass
