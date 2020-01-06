#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"

install_mrpt() {
    sudo add-apt-repository ppa:joseluisblancoc/mrpt
    sudo apt-get update
    sudo apt-get install libmrpt-dev mrpt-apps
    echo -e "\033[33m install mrpt successfully \033[0m"
}

install_mrpt