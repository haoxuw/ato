# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.configuration import segment_configuration
from mesh.model_structure.gripper import gripper_bone


class GripperUsingCog(gripper_bone.GripperBone):
    def __init__(
        self,
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
    ):
        super().__init__(segment_configs=segment_configs, name=name)
        # use this class to build a complex gripper
