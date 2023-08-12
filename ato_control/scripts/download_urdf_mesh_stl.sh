#!/bin/bash
# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

set -e

for folder in pitch_range_270 pitch_range_180
do
  mkdir -p "/tmp/${folder}"

    for filename in BoneAugmented_urdf_mesh.stl JointAugmented_urdf_mesh.stl GripperDragonWithServo_urdf_mesh.stl
    do
    if [ ! -f "/tmp/${folder}/${filename}" ]; then
      echo "Downloading ${filename} to /tmp/${folder}/${filename}"
      wget https://printable-mesh.s3.us-east-2.amazonaws.com/${folder}/${filename} -O /tmp/${folder}/${filename}
    fi
    done

done
