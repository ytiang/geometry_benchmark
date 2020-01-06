#!/usr/bin/env bash
set -e  # exit on first error
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export CC=/usr/bin/gcc-7
export CXX=/usr/bin/g++-7

#bash ${CURRENT_DIR}/install_cmake.sh
bash ${CURRENT_DIR}/install_benchmark.sh
bash ${CURRENT_DIR}/install_wave_geometry.sh
bash ${CURRENT_DIR}/install_mrpt.sh
bash ${CURRENT_DIR}/install_gtsam.sh

