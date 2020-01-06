#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_benchmark() {
	cd ${SOURCE_DIR}
	rm -rf benchmark
	git clone https://github.com/google/benchmark.git
	## use version 1.5.0
	cd benchmark/ && git pull && git reset --hard 090faec
	## install
	rm -rf build && mkdir build
	cd build 
	cmake ..
	make -j$(nproc)
	sudo make install
	echo -e "\033[33m install gtsam 4.0.0 successfully \033[0m"
}

install_benchmark