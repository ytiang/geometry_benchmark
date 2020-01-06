#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"


install_mrpt() {
	sudo apt install -y build-essential pkg-config libwxgtk3.0-dev libwxgtk3.0-gtk3-dev libopencv-dev libeigen3-dev
	sudo apt install -y libftdi-dev freeglut3-dev zlib1g-dev libusb-1.0-0-dev libudev-dev libfreenect-dev libdc1394-22-dev libavformat-dev libswscale-dev libassimp-dev libjpeg-dev libsuitesparse-dev libpcap-dev liboctomap-dev
	cd ${SOURCE_DIR}
	rm -rf mrpt*
	git clone https://github.com/MRPT/mrpt.git && cd mrpt && git reset --hard f67d0f8
	mkdir build && cd build
	cmake ..
	make -j$(nproc) && sudo make install
    echo -e "\033[33m install mrpt successfully \033[0m"
}

install_mrpt