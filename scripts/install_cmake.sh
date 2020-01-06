#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"
CMAKE_VERSION=3.13.4

install_cmake() {
	cd ${SOURCE_DIR}
	rm -rf cmake-${CMAKE_VERSION}*
	wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz

	tar -xf cmake-${CMAKE_VERSION}-Linux-x86_64.tar.gz
	## use version cmake 3.13.4
	cd cmake-${CMAKE_VERSION}-Linux-x86_64
	## install
	./bootstrap && make -j$(nproc) && sudo make install
	echo -e "\033[33m install cmake 3.13 successfully \033[0m"
}

install_cmake