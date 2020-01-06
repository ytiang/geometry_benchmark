#!/usr/bin/env bash
set -e  # exit on first error
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

bash ${CURRENT_DIR}/install_cmake.sh
bash ${CURRENT_DIR}/install_benchmark.sh
bash ${CURRENT_DIR}/install_gtsam.sh
bash ${CURRENT_DIR}/install_mrpt.sh