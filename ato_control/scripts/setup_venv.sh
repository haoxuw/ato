#!/bin/bash
# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

set -e
VENV_FOLDER_NAME='venv'

SCRIPT_FILE_PATH=$(readlink -f ${BASH_SOURCE[0]})
SCRIPT_FOLDER_PATH=$(dirname ${SCRIPT_FILE_PATH})

pushd "${SCRIPT_FOLDER_PATH}/../" > /dev/null

# RPI runs on python 3.7 by default
[ ! -d "${VENV_FOLDER_NAME}" ] && echo "Creating python virtual environment under ${VENV_FOLDER_NAME}" && python3 -m venv ${VENV_FOLDER_NAME}

source ${VENV_FOLDER_NAME}/bin/activate
echo "Creating virtual environment, the initial run might take a while..."
pip install --upgrade pip
pip install -r ./requirements.txt || echo 'Warning: some dependencies failed to install, assuming expected.'
popd > /dev/null

echo
echo "Virtual environment activated @ ${SCRIPT_FOLDER_PATH}/../${VENV_FOLDER_NAME}"
echo
