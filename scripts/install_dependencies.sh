#!/usr/bin/env bash
set -e  # exit on first error
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"




bash ${CURRENT_DIR}/install_cmake3_13.sh
bash ${CURRENT_DIR}/install_gtsam.sh
bash ${CURRENT_DIR}/install_mrpt.sh