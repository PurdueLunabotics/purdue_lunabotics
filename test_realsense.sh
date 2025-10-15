#!/bin/bash
# installs Realsense SDK 
if [ $(arch) == 'aarch64' ]; then
    if [ -f /etc/nv_tegra_release ]; then
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
