#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_benchmark() {
	cd ${SOURCE_DIR}
	rm -rf benchmark
	git clone https://github.com/google/benchmark.git
	## use version 1.5.0
	cd benchmark/ && git pull && git reset --hard 090faec
	## clone googletest
	git clone https://github.com/google/googletest.git
	cd googletest && git reset --hard 703bd9c
	## build and install
	cd .. && rm -rf build && mkdir build
	cd build 
	cmake ..
	make -j$(nproc)
	sudo make install
	echo -e "\033[33m install benchmark successfully \033[0m"
}

install_benchmark