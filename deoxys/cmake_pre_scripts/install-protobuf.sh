#! /bin/bash
#
# build-protobuf.sh
# Copyright (C) 2024 Jiayuan Mao <maojiayuan@gmail.com>
#
# Distributed under terms of the MIT license.
#

echo "WARNING: This script will overwrite the system-installed protobuf (if it exists)."
echo "If you have existing projects that depend on the current protobuf installation,"
echo "they may be affected. Proceed with caution."
echo
read -p "Do you want to continue? (y/N): " confirm
if [[ $confirm != [yY] && $confirm != [yY][eE][sS] ]]; then
    echo "Installation aborted."
    exit 1
fi

# download protobuf
git clone --recursive https://github.com/protocolbuffers/protobuf.git
cd protobuf

# Prompt user for Protobuf version with default
default_version="v3.13.0"
read -p "Enter the desired Protobuf version (default: $default_version): " protobuf_version
protobuf_version=${protobuf_version:-$default_version}

# Checkout the specified version
git checkout $protobuf_version
git submodule update

./autogen.sh
./configure
make
make check
sudo make install
sudo ldconfig # refresh shared library cache.
