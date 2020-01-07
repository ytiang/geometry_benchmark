#!/bin/bash
set -e # exit on first error
SOURCE_DIR="/tmp"


install_mrpt() {
    sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5 -y 
    sudo apt-get update
    sudo apt-get install libmrpt-dev mrpt-apps -y
    echo -e "\033[33m install mrpt successfully \033[0m"
}

install_mrpt