#!/bin/bash

# useful links:
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
# https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md
# https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/
# useful issues:
# https://github.com/IntelRealSense/librealsense/issues/7722
# https://github.com/IntelRealSense/librealsense/issues/6964

LIBREALSENSE_DIR=./librealsense
LIBREALSENSE_VERSION=2.50.0
   
export DEBIAN_FRONTEND=noninteractive

# install the core packages required to build librealsense
apt-get update -y && apt-get install -y --no-install-recommends libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev &&
  rm -rf /var/lib/apt/lists/* &&
  apt-get clean

# install distribution-specific packages (graphics libs - for librealsense OpenGL-enabled examples)
apt-get update -y && apt-get install -y --no-install-recommends libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev &&
  rm -rf /var/lib/apt/lists/* &&
  apt-get clean

# get librealsense
git clone https://github.com/IntelRealSense/librealsense.git -b v${LIBREALSENSE_VERSION}
# checkout version the last stable version of librealsense
git checkout $LIBREALSENSE_VERSION

# compile and build librealsense
cd "${LIBREALSENSE_DIR}"
mkdir build && cd build
cmake ../ \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_WITH_CUDA=true \
  -DCMAKE_BUILD_TYPE=RELEASE
make uninstall && make clean && make -j$(nproc) && make install

# set environment variables for installed pyrealsense2

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

#set librealsense devices rules
# issue in docker: https://github.com/IntelRealSense/realsense-ros/issues/388#issuecomment-410672421
#cd ./librealsense && cp ./config/99-realsense-libusb.rules /etc/udev/rules.d && udevadm control --reload-rules && udevadm trigger
