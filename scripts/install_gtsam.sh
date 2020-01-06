#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_gtsam() {
	cd ${SOURCE_DIR}
	if [ -d "gtsam" ];then
		echo "Already cloned gtsam!"
	else
		git clone https://github.com/borglab/gtsam.git
	fi

	## use version 4.0.0
	cd gtsam/ && git pull && git reset --hard 52e8db6
	## install
	rm -rf build && mkdir build
	cd build 
	cmake ..
	make -j$(nproc)
	sudo make install
	echo -e "\033[33m install gtsam 4.0.0 successfully \033[0m"
}

install_gtsam