#!/bin/bash
# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

set -e

FOLDER=pitch_range_270

mkdir -p "/tmp/${FOLDER}"
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/BoneAugmented_urdf_mesh.stl -O /tmp/${FOLDER}/BoneAugmented_urdf_mesh.stl
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/JointAugmented_urdf_mesh.stl -O /tmp/${FOLDER}/JointAugmented_urdf_mesh.stl
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/GripperDragonWithServo_urdf_mesh.stl -O /tmp/${FOLDER}/GripperDragonWithServo_urdf_mesh.stl

FOLDER=pitch_range_180

mkdir -p "/tmp/${FOLDER}"
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/BoneAugmented_urdf_mesh.stl -O /tmp/${FOLDER}/BoneAugmented_urdf_mesh.stl
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/JointAugmented_urdf_mesh.stl -O /tmp/${FOLDER}/JointAugmented_urdf_mesh.stl
wget https://printable-mesh.s3.us-east-2.amazonaws.com/${FOLDER}/GripperDragonWithServo_urdf_mesh.stl -O /tmp/${FOLDER}/GripperDragonWithServo_urdf_mesh.stl