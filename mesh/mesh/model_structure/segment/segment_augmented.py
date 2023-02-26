# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

from mesh.model_structure.segment import (
    bone_augmented,
    bone_separable_vertical,
    joint_augmented,
)
from mesh.model_structure.segment.abstractions import segment_abstract


class SegmentAugmented(segment_abstract.SegmentAbstract):
    @property
    def joint_class(self):
        return joint_augmented.JointAugmented

    @property
    def joint_pos_class(self):
        return joint_augmented.JointAugmentedPos

    @property
    def joint_neg_class(self):
        return joint_augmented.JointAugmentedNeg

    @property
    def bone_class(self):
        if self.segment_configs.separable_bone:
            return bone_separable_vertical.BoneSeparableVertical
        else:
            return bone_augmented.BoneAugmented
