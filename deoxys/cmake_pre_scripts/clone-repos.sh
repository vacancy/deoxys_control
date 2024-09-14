#! /bin/bash
#
# clone-repos.sh
# Copyright (C) 2024 Jiayuan Mao <maojiayuan@gmail.com>
#
# Distributed under terms of the MIT license.
#

# download libfranka
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka

echo "Which libfranka version are you going to use?"
read version
echo "chose libfranka ${version}"
if [[ "$version" != "0.8.0" && "$version" != "0.9.0" && "$version" != "0.10.0" ]]
then
    printf "${RED} The version ${version} may not be supported!!! \n ${NC}"
fi
git checkout ${version}

git submodule update
cd ..

# download c++ zmq wrapper
git clone https://github.com/zeromq/zmqpp.git

# download yaml cpp package
git clone https://github.com/jbeder/yaml-cpp.git

# download spdlog package
git clone https://github.com/gabime/spdlog.git
cd spdlog
git checkout ac55e60488032b9acde8940a5de099541c4515da
cd ..

