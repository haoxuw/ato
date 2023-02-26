#!/bin/bash
# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

SCRIPT_FILE_PATH=$(readlink -f ${BASH_SOURCE[0]})
SCRIPT_FOLDER_PATH=$(dirname ${SCRIPT_FILE_PATH})
OUT_DIR=./generated

{ cd "${SCRIPT_FOLDER_PATH}/.." && rm -rf shelf.stl && ./scripts/generate_shelf.sh --out_dir ${OUT_DIR} $@ && ls ${OUT_DIR}/shelf.stl ; } || { echo 'Mesh module failed to generate shelf.stl.'; exit -2; }

echo
echo "Sanity passed"
echo
exit 0
