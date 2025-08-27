#!/bin/bash
set -e
set -o pipefail

# Get the target Python version from the environment cibuildwheel provides
PY_MAJOR=$(python -c "import sys; print(sys.version_info.major)")
PY_MINOR=$(python -c "import sys; print(sys.version_info.minor)")
PY_MICRO=$(python -c "import sys; print(sys.version_info.micro)")
PY_VERSION="${PY_MAJOR}.${PY_MINOR}.${PY_MICRO}"
PY_VER_SHORT="${PY_MAJOR}.${PY_MINOR}"
INSTALL_PREFIX="/usr/local/python-${PY_VER_SHORT}"

echo "--- Preparing to build Python ${PY_VERSION} from source ---"

# --- Build and Install Python from Source ---
cd /
wget "https://www.python.org/ftp/python/${PY_VERSION}/Python-${PY_VERSION}.tgz"
tar -xzf "Python-${PY_VERSION}.tgz"
cd "Python-${PY_VERSION}"

# Configure with an isolated prefix and enable shared library for boost_python
./configure --enable-optimizations --enable-shared
make -j$(nproc)
make install
rm -rf "/Python-${PY_VERSION}" "/Python-${PY_VERSION}.tgz"

# --- Update PATH to use our newly compiled Python ---
export PATH="${INSTALL_PREFIX}/bin:$PATH"

echo "Python ${PY_VERSION} installed to ${INSTALL_PREFIX}"
echo "New Python path: $(which python)"
python --version

# --- Build Boost against the current Python version ---
BOOST_VERSION="1.82.0"
BOOST_SRC="boost_${BOOST_VERSION//./_}"
echo "--- Building Boost ${BOOST_VERSION} for Python ${PY_VER_SHORT}... ---"
cd /
wget -q "https://archives.boost.io/release/${BOOST_VERSION}/source/${BOOST_SRC}.tar.bz2"
tar xf "${BOOST_SRC}.tar.bz2"
cd "${BOOST_SRC}"

# PYTHON_EXEC will now point to our new Python build
PYTHON_EXEC=$(which python)
./bootstrap.sh --with-libraries=system,filesystem,thread,serialization,chrono,python --with-python="${PYTHON_EXEC}"
./b2 -j$(nproc) install
ldconfig
rm -rf "/${BOOST_SRC}"

echo "--- Python-specific dependencies for ${PY_VER_SHORT} installed successfully. ---"