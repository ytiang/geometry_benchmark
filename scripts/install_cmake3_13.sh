#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_cmake_3_13() {
	cd ${SOURCE_DIR}
	if [ -d "Cmake" ];then
		echo "Already cloned Cmake!"
	else
		git clone https://github.com/Kitware/CMake.git
	fi

	## use version cmake 3.13.4
	cd Cmake/ && git pull && git reset --hard 30c3eff
	## install
	./bootstrap && make -j$(nproc) && sudo make install
	echo -e "\033[33m install cmake 3.13 successfully \033[0m"
}

install_cmake_3_13