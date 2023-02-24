# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.configuration import segment_configuration
from mesh.model_structure.gripper import gripper_dragon
from mesh.model_structure.toys import wheels


class GripperDragonOnWheels(gripper_dragon.GripperDragon):
    def __init__(
        self, segment_configs: segment_configuration.SegmentConfigs, name: str = ""
    ):
        super().__init__(segment_configs=segment_configs, name=name)

    def model_mesh(self, add_surface_give=False):
        offsets = (0, -50, -40)
        dragon_mesh = super().model_mesh()
        wheel_mesh = wheels.Wheels().model_mesh().translate(offsets)
        wheel_socket = (
            wheels.Wheels().model_mesh(add_surface_give=True).translate(offsets)
        )
        return dragon_mesh.cut(wheel_socket).add(wheel_mesh)

    def printable_mesh(self):
        mesh = self.model_mesh()
        # affine transform to print up straight, for easier removal of supporting materials
        mesh = mesh.translate(
            (
                0,
                self.segment_configs.structural.RollBoneMeshNetLength
                - self._bone_cut_length_(),
                0,
            )
        ).rotate((0, 0, 0), (1, 0, 0), 90)
        return mesh
