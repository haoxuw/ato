# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import cadquery as cq

from mesh.configuration import segment_configuration
from mesh.model_structure.mechanical_parts import allocated_motors, bolt
from mesh.model_structure.segment import bone_augmented
from mesh.model_structure.segment.abstractions import joint_abstract


class JointAugmented(joint_abstract.JointAbstract):
    @property
    def bone_class(self):
        return bone_augmented.BoneAugmented

    def model_mesh(self, add_surface_give=False):
        joint = super().model_mesh(add_surface_give=add_surface_give)
        joint = self.__put_bolts_on_joint__(joint)

        joint = joint.add(
            allocated_motors.AllocatedMotors(
                segment_configs=self.segment_configs
            ).supporting_mesh()
        )

        joint = joint.cut(
            allocated_motors.AllocatedMotors(
                segment_configs=self.segment_configs
            ).model_mesh(add_surface_give=True)
        )

        joint = joint.cut(
            allocated_motors.AllocatedMotors(
                segment_configs=self.segment_configs
            ).motor_fastening_bolt_space_mesh()
        )

        return joint

    def __put_bolts_on_joint__(self, base):
        def relocate_for_base(obj, offset):
            obj = obj.rotate((0, 0, 0), (1, 0, 0), 90)
            obj = obj.translate(offset)
            return obj

        joint_bolt = bolt.Bolt(
            segment_configs=self.segment_configs,
            install_from_side_instead_along=False,
            nut_offset_factor=0.9,
            bolt_configs=segment_configuration.BoltConfigsBase(
                BoltDiam=3, BoltLength=16
            ),
        )

        #####################
        # the following determines where to put the bolts, aside from dodging other components,
        # those positions are rather arbitrary
        # meanwhile, some math were used to determine the rough location of the bolts
        dist_x = (
            self.segment_configs.structural.JointRadius
            / self.segment_configs.structural.BoltOffsetCoefficientX
        )  # wide but avoid motor wall
        start_offset_z = (
            self.segment_configs.structural.JointRadius
            / self.segment_configs.structural.BoltOffsetCoefficientZ
        )
        offset_z_factor = 1.5
        #####################

        for offset_x in [dist_x, -dist_x]:
            for offset_z in [
                -start_offset_z,
                start_offset_z
                + self.segment_configs.structural.JointLength
                - self.segment_configs.structural.BoneHolderRadius * offset_z_factor,
            ]:
                space = relocate_for_base(
                    joint_bolt.space_mesh(),
                    (
                        offset_x,
                        -(joint_bolt.bolt_configs.BoltLength) / 2,
                        offset_z,
                    ),
                )
                base = base.cut(space)
        return base

    def urdf_mesh(self):
        model_mesh = self.model_mesh(add_surface_give=True)
        model_mesh = model_mesh.rotate((0, 0, 0), (1, 0, 0), 180)
        model_mesh = model_mesh.translate(
            (0, 0, self.segment_configs.structural.SegmentLength)
        )
        return model_mesh


class JointAugmentedPos(joint_abstract.JointAbstractPos):
    @property
    def joint_class(self):
        return JointAugmented

    def model_mesh(self, add_surface_give=False):
        pos_mesh = super().model_mesh(add_surface_give=add_surface_give)
        pos_mesh = self.__add_joint_sphere_holders(pos_mesh)
        return pos_mesh

    def __add_joint_sphere_holders(self, mesh):
        for holder_config in self.segment_configs.structural.JointSphereHolders:
            offset, radius = holder_config
            sphere_holder = (
                cq.Workplane(self.workplane_primary).sphere(radius).translate(offset)
            )
            mesh = mesh.add(sphere_holder)
        return mesh


class JointAugmentedNeg(joint_abstract.JointAbstractNeg):
    @property
    def joint_class(self):
        return JointAugmented

    def model_mesh(self, add_surface_give=False):
        neg_mesh = super().model_mesh(add_surface_give=add_surface_give)
        neg_mesh = self.__add_joint_sphere_holders(neg_mesh)
        return neg_mesh

    def __add_joint_sphere_holders(self, mesh):
        for holder_config in self.segment_configs.structural.JointSphereHolders:
            offset, radius = holder_config
            radius += self.segment_configs.structural.SurfaceGive / 2
            sphere_holder = (
                cq.Workplane(self.workplane_primary).sphere(radius).translate(offset)
            )
            mesh = mesh.cut(sphere_holder)
        return mesh
