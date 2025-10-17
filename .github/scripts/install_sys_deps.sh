#!/bin/bash
set -e
set -o pipefail

echo "============================================================"
echo "Installing ORB-SLAM System Dependencies"
echo "============================================================"

echo "--- Installing common dependencies on AlmaLinux... ---"
yum clean all
yum update -y
yum install -y epel-release

# --- Install dependencies common to all builds ---
echo "--- Installing base development tools... ---"
yum install -y \
    eigen3-devel glew-devel libX11-devel mesa-libGL-devel libpng-devel \
    libXext-devel libtiff-devel libXv-devel libjpeg-turbo-devel \
    suitesparse-devel cmake gcc-c++ make git wget unzip yasm pkg-config \
    libavc1394-devel openssl-devel tbb-devel bzip2-devel libffi-devel \
    zlib-devel

# --- Build OpenCV ---
OPENCV_VERSION="4.8.0"
echo "--- Building OpenCV version ${OPENCV_VERSION} from source... ---"
mkdir -p /tmp/opencv_build && cd /tmp/opencv_build
wget -q -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
wget -q -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip
unzip -q opencv.zip && unzip -q opencv_contrib.zip
cd opencv-${OPENCV_VERSION} && mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_build/opencv_contrib-${OPENCV_VERSION}/modules \
    -D WITH_TBB=ON -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF ..
make -j$(nproc) && make install
ldconfig
rm -rf /tmp/opencv_build

# --- Build Pangolin ---
PANGOLIN_VERSION="v0.9.1"
echo "--- Building Pangolin version ${PANGOLIN_VERSION} from source ---"
cd /
wget "https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/${PANGOLIN_VERSION}.tar.gz" -O pangolin.tar.gz
tar -xzf pangolin.tar.gz && cd Pangolin-0.9.1 && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && make install
ldconfig
rm -rf /Pangolin-0.9.1 /pangolin.tar.gz

echo "============================================================"
echo "Common dependencies installed successfully"
echo "OpenCV Version: ${OPENCV_VERSION}"
echo "Pangolin Version: ${PANGOLIN_VERSION}"
echo "============================================================"