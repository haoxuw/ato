# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import os
from abc import ABC
from collections import Counter

import cadquery as cq

from mesh.configuration import segment_configuration


class CqMesh(ABC):
    def __init__(
        self,
        # an assembled segment is the minimal component of an arm, i.e. they are atomic
        # segment configs is needed in each models (i.e. bone, joint, bolts, motor),
        # which all have physical dependency and limitation imposed on each other
        # so we pass segment_configs to all the models
        segment_configs: segment_configuration.SegmentConfigs,
        name: str = "",
    ):
        self.segment_configs = segment_configs
        self._name = name
        # to understand coordinate system of each mesh, check out the no_axes flag
        self.workplane_primary = "XY"
        self.workplane_secondary = "YZ"
        self.workplane_tertiary = "XZ"

        assert Counter(
            self.workplane_primary + self.workplane_secondary + self.workplane_tertiary
        ) == {"X": 2, "Y": 2, "Z": 2}

    @property
    def name(self):
        return self._name

    # the mesh of this physical object
    # # pylint: disable=unused-argument
    def model_mesh(self, add_surface_give=False):
        return None

    # the mesh that this object spatially occupy, including for installation (e.g. allow inserting screw driver) and runtime (e.g. allow rotation/movement),
    # this also include any hollow space may be inside of the model, so that larger_model.cut(space_mesh) would yield meaningful enclosing mesh
    def space_mesh(self):
        return None

    # the mesh that this object needs as support, e.g. extension structure to fasten motors
    def supporting_mesh(self):
        return None

    # the mesh that this object should be printed
    # i.e. the combining the two sections as one object, and translated to the preferred orientation suitable for 3D printing
    def printable_mesh(self):
        return self.model_mesh(add_surface_give=False)

    # translated the rotational center to origin to make importing to urdf easier
    def urdf_mesh(self):
        return None

    def export(
        self, designated_filename=None, only_printable_or_urdf=True, out_dir="."
    ):
        if only_printable_or_urdf:
            mesh_funcs = [self.printable_mesh, self.urdf_mesh]
        else:
            mesh_funcs = [
                self.printable_mesh,
                self.model_mesh,
                self.space_mesh,
                self.supporting_mesh,
            ]

        for mesh_func in mesh_funcs:
            # pylint: disable=assignment-from-none
            mesh = mesh_func()
            if mesh is not None:
                if designated_filename is None:
                    name_str = ("_" + self.name) if self.name else ""
                    filename = f"{self.name}{self.__class__.__name__}{name_str}_{mesh_func.__name__}.stl"
                else:
                    filename = designated_filename
                filename = os.path.join(out_dir, filename)
                cq.exporters.export(mesh_func(), filename)
                logging.info(f"Exported {filename}")
            else:
                logging.debug(f"Skipped exporting {filename}: mesh not defined")
