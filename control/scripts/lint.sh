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
source ./scripts/setup_venv.sh > /dev/null

black --version
black . --check || { echo 'Linting failed, need to run $ black .'; exit -2; }

isort --version
isort . --check-only || { echo 'Linting failed, need to run $ isort control'; exit -3; }

pylint --version
pylint control || { echo 'Linting failed.'; exit -4; }

echo
echo "Linting passed"
echo
