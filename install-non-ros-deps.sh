#!/bin/sh
apt-get update  # To get the latest package lists
apt-get install libusb-dev libtool dh-autoreconf libudev-dev -y

git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2
