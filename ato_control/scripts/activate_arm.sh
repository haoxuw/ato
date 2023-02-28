#!/bin/bash
# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.
set -e
SCRIPT_FILE_PATH=$(readlink -f ${BASH_SOURCE[0]})
SCRIPT_FOLDER_PATH=$(dirname ${SCRIPT_FILE_PATH})

pushd "${SCRIPT_FOLDER_PATH}/../" > /dev/null
echo "Creating virtual environment, the initial run might take a while..."
source ./scripts/setup_venv.sh > /dev/null
echo "Virtual environment created successfuly"

if [ "$EUID" -eq 0 ]
  then 
  sudo pigpiod
  else
  echo 'Warning: failed to launch pigpiod, assuming not running on pi.'
fi

{ python activate_arm.py $@; } || { echo -e "\n\n\t *** Arm controller was not able to gracefully terminate ***\n\n"; exit -2; }

echo
echo "Arm controller exited succesfully"
echo
