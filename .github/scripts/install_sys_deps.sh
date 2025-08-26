#!/bin/bash
# This script installs dependencies for the build, including building
# a specific version of OpenCV & Pangolin from source to ensure compatibility.

set -e
set -o pipefail

echo "--- Running yum clean and installing base dependencies on AlmaLinux... ---"
yum clean all
yum update -y
yum install -y epel-release

echo "--- Running custom packages for CI application ---"
PY_VERSION=$(python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
# Install dependencies required to build OpenCV and for the main project
yum install -y \
    python${PY_VERSION}-devel \
    eigen3-devel \
    glew-devel \
    libX11-devel \
    mesa-libGL-devel \
    libpng-devel \
    libXext-devel \
    libtiff-devel \
    libXv-devel \
    libjpeg-turbo-devel \
    suitesparse-devel \
    cmake \
    gcc-c++ \
    make \
    git \
    wget \
    unzip \
    yasm \
    pkg-config \
    libjpeg-turbo-devel \
    libpng-devel \
    libtiff-devel \
    libavc1394-devel \
    openssl-devel \
    tbb-devel 
    # boost-devel 

# --- OpenCV Build from Source ---
OPENCV_VERSION="4.8.0"
echo "--- Building OpenCV version ${OPENCV_VERSION} from source... ---"

# Create a temporary directory for the build
mkdir -p /tmp/opencv_build
cd /tmp/opencv_build

# Download the source code for OpenCV and OpenCV-contrib
wget -q -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
wget -q -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip
unzip -q opencv.zip
unzip -q opencv_contrib.zip

# Create a build directory
cd opencv-${OPENCV_VERSION}
mkdir -p build && cd build

# Configure CMake for the OpenCV build
# We disable tests and examples to speed up the build process.
cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_build/opencv_contrib-${OPENCV_VERSION}/modules \
    -D WITH_TBB=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    ..


# Compile and install OpenCV
make -j$(nproc)
make install

# Update the shared library cache
ldconfig

# Clean up the build files to save space
cd /
rm -rf /tmp/opencv_build


# Build and install a specific version of Pangolin from source
# This is necessary to match the version you developed against.
# ----------------------------------------------------------------
PANGOLIN_VERSION="v0.9.1"
PANGOLIN_INSTALL_PATH="/usr/local/pangolin"
echo "--- Building Pangolin version ${PANGOLIN_VERSION} from source to ${PANGOLIN_INSTALL_PATH} ---"

wget "https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/${PANGOLIN_VERSION}.tar.gz" -O pangolin.tar.gz
tar -xzf pangolin.tar.gz
cd Pangolin-0.9.1
mkdir build && cd build

# We specify the install path so we know where to find it later.
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${PANGOLIN_INSTALL_PATH}
make -j$(nproc)
make install
ldconfig

# Clean up build files
cd /
rm -rf /Pangolin-0.9.1 /pangolin.tar.gz


# --- Boost Build from Source ---
BOOST_VERSION="1.82.0"
BOOST_SRC="boost_${BOOST_VERSION//./_}"
BOOST_DIR="/usr/local/boost"

echo "--- Building Boost version ${BOOST_VERSION} from source... ---"

wget -q https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/${BOOST_SRC}.tar.bz2
tar xf ${BOOST_SRC}.tar.bz2
cd ${BOOST_SRC}

PYTHON_EXEC=$(which python)
./bootstrap.sh --with-libraries=python --with-python=${PYTHON_EXEC}
./b2 -j$(nproc) install --prefix=${BOOST_DIR}

# Clean up
cd /
rm -rf ${BOOST_SRC}

# Update library cache
ldconfig

echo "--- Boost ${BOOST_VERSION} installed to ${BOOST_DIR} ---"


echo "--- All dependencies, including OpenCV ${OPENCV_VERSION}, Boost ${BOOST_VERSION} and Pangolin ${PANGOLIN_VERSION} installed successfully. ---"