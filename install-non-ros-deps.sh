#!/bin/bash
apt-get update  # To get the latest package lists
apt-get install libusb-dev libtool dh-autoreconf libudev-dev ninja-build -y

if [ $(arch) == 'aarch64' ]; then
    if [ -f /etc/nv_tegra_release ]; then
         # install jetpack
         apt install \
         nvidia-jetpack \
         python3-libnvinfer-dev \
         python2.7-dev \
         python-dev \
         python-py \
         python-attr \
         python-funcsigs \
         python-pluggy \
         python-pytest \
         python-six \
         uff-converter-tf \
         libtbb-dev

        # installs Realsense SDK 
        echo "This is a Jetson device."
        mkdir ~/dev
        cd ~/dev
        git clone https://github.com/jetsonhacks/installRealSenseSDK
        cd installRealSenseSDK
        ./buildLibrealsense.sh -j 6 
        cd ..
        rm -rf installRealSenseSDK

        exit
    fi
fi
echo "This is not a Jetson device."
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install realsense2
exit
