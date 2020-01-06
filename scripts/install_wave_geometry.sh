#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_wave_geometry() {
	cd ${SOURCE_DIR}
	rm -rf _wave_geometry
	git clone https://github.com/wavelab/wave_geometry.git
	git reset --hard d68d978

	cd wave_geometry
	mkdir build && cd build
	cmake .. -DBUILD_TESTING=OFF
	sudo make install
	echo -e "\033[33m install wave_geometry successfully \033[0m"
}

install_wave_geometry