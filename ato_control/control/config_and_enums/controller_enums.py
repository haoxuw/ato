# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from dataclasses import dataclass


@dataclass
class ControllerStates:
    LOG_INFO_EACH_TENTHS_SECOND = "__DEBUG_STATE__LOG_INFO_EACH_TENTHS_SECOND"

    CURRENT_MODE = "CURRENT_MODE"  # value has to be one of those:

    DEFAULT = "IN_CARTESIAN_MODE"
    IN_CARTESIAN_MODE = "IN_CARTESIAN_MODE"

    IN_JOINT_SPACE_MODE = "IN_JOINT_SPACE_MODE"

    IN_SETTING_MODE = "IN_SETTING_MODE"

    IN_TRAJECTORY_RECORDING_MODE = "IN_TRAJECTORY_RECORDING_MODE"
    RECORDING_ON = "RECORDING_ON"


@dataclass
class SolverMode:
    FORWARD = "Forward"
    ALL = "All"
    Y = "Y"
    Z = "Z"
    NONE = "None"
