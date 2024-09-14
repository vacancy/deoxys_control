#! /bin/bash
#
# install-system-packages.sh
# Copyright (C) 2024 Jiayuan Mao <maojiayuan@gmail.com>
#
# Distributed under terms of the MIT license.
#

# Print warning about system package installation
echo "WARNING: This script will install or update system packages."
echo "Including: build-essential cmake git libpoco-dev libeigen3-dev autoconf automake libtool curl make g++ unzip libzmq3-dev libreadline-dev bzip2 libmotif-dev libglfw3"
echo "This may affect existing software on your system."
echo "It is recommended to review the list of packages before proceeding."
echo
read -p "Do you want to continue? (y/N): " confirm
if [[ $confirm != [yY] && $confirm != [yY][eE][sS] ]]; then
    echo "Installation aborted."
    exit 1
fi

# Build essentials
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev -y

# For protoc
sudo apt-get install autoconf automake libtool curl make g++ unzip -y

# For zmq
sudo apt-get install libzmq3-dev -y

# Install other apt packages
sudo apt install libreadline-dev -y
sudo apt install bzip2 -y
sudo apt install libmotif-dev libglfw3 -y
