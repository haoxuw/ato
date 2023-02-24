# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging
import os
import pathlib
from typing import List

import cadquery as cq

from mesh.cq_lib import cq_mesh


class CqMeshShelf:
    def __init__(
        self,
        cq_meshes: List[cq_mesh.CqMesh],
        distance=100,
        show_axes=False,
        out_dir=".",
    ):
        self.cq_meshes = list(cq_meshes)
        self.distance = distance
        self.show_axes = show_axes
        self.out_dir = out_dir
        pathlib.Path(out_dir).mkdir(parents=True, exist_ok=True)

    def __axes__(self):
        # ratio: X:1 Y:2 Z:3
        z_axis = cq.Workplane("XY").rect(1, 1).extrude(self.distance * 0.9)
        # extrude towards negative because y axis of the primary view plane, i.e.:
        # axis are determined by left hand rule, X = thumb, Y = middle, Z = ring
        # meanwhile only on XZ plane, positive extrude is towards the opposite of Y
        # meaning positive extrusion of ZX is facing positive Y
        y_axis = cq.Workplane("XZ").rect(1, 1).extrude(-self.distance * 0.6)
        x_axis = cq.Workplane("YZ").rect(1, 1).extrude(self.distance * 0.3)

        # x = YZ = roll = u
        # y = ZX = pitch = v
        # z = XY = yaw = w
        return x_axis.add(y_axis).add(z_axis)

    def shelf_mesh(self):
        shelf = cq.Workplane("XY").sphere(3)
        for index_x, cq_mesh_obj in list(enumerate(self.cq_meshes)):
            for index_y, mesh_func in enumerate(
                (
                    cq_mesh_obj.printable_mesh,
                    cq_mesh_obj.model_mesh,
                    cq_mesh_obj.space_mesh,
                    cq_mesh_obj.supporting_mesh,
                    cq_mesh_obj.urdf_mesh,
                )
            ):
                mesh = mesh_func()
                if mesh is not None:
                    offset = ((index_x) * self.distance, (index_y) * self.distance, 0)
                    if self.show_axes:
                        mesh.add(self.__axes__())
                    mesh = mesh.translate(offset)
                    logging.info(
                        f"Placed {cq_mesh_obj.__class__.__name__}:{mesh_func.__name__} at {offset}"
                    )
                    shelf = shelf.add(mesh)
                else:
                    logging.debug(
                        f"Skipped placing {cq_mesh_obj.__class__.__name__}:{mesh_func.__name__}"
                    )
        return shelf

    def export_each(self, only_printable_or_urdf=True):
        for cq_mesh_obj in self.cq_meshes:
            cq_mesh_obj.export(
                only_printable_or_urdf=only_printable_or_urdf, out_dir=self.out_dir
            )

    def export(self, filename="shelf.stl"):
        shelf_mesh = self.shelf_mesh()
        cq.exporters.export(shelf_mesh, os.path.join(self.out_dir, filename))
        logging.info(f"Exported {filename}")
