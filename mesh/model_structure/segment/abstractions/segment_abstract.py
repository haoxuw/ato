# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.cq_lib import cq_mesh
from mesh.model_structure.segment.abstractions import bone_abstract, joint_abstract


class SegmentAbstract(cq_mesh.CqMesh):
    @property
    def joint_class(self):
        return joint_abstract.JointAbstract

    @property
    def joint_pos_class(self):
        return joint_abstract.JointAbstractPos

    @property
    def joint_neg_class(self):
        return joint_abstract.JointAbstractNeg

    @property
    def bone_class(self):
        return bone_abstract.BoneAbstract

    def printable_mesh(self):
        joint_whole = self.joint_class(segment_configs=self.segment_configs)
        distance = (
            max(
                joint_whole.outer_radius
                + self.segment_configs.structural.BoneRadius
                + self.segment_configs.structural.RollCylinderHolderDepth,  # roll side distance
                self.segment_configs.structural.BoneRadius
                + self.segment_configs.structural.PitchMotorOffsetX
                + self.segment_configs.actuator.MotorFastenerHeight,  # pitch side distance
            )
            + self.segment_configs.structural.SurfaceGive
        )
        joint_pos = (
            self.joint_pos_class(segment_configs=self.segment_configs)
            .printable_mesh()
            .translate((distance, 0, joint_whole.max_distance_from_surface))
        )
        joint_neg = (
            self.joint_neg_class(segment_configs=self.segment_configs)
            .printable_mesh()
            .translate((-distance, 0, joint_whole.max_distance_from_surface))
        )

        bone_mesh = self.bone_class(
            segment_configs=self.segment_configs
        ).printable_mesh()
        arm_segment = bone_mesh.add(joint_pos).add(joint_neg)
        return arm_segment
