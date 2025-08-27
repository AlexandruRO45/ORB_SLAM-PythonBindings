#!/bin/bash
set -e
set -o pipefail


echo "--- Installing Python-specific dependencies... ---"
PY_VERSION=$(python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
yum install -y python${PY_VERSION}-devel

# --- Build Boost against the current Python version ---
BOOST_VERSION="1.82.0"
BOOST_SRC="boost_${BOOST_VERSION//./_}"
echo "--- Building Boost ${BOOST_VERSION} for Python ${PY_VERSION}... ---"
cd /
wget -q https://archives.boost.io/release/${BOOST_VERSION}/source/${BOOST_SRC}.tar.bz2
tar xf ${BOOST_SRC}.tar.bz2
cd ${BOOST_SRC}

PYTHON_EXEC=$(which python)
./bootstrap.sh --with-libraries=system,filesystem,thread,serialization,chrono,python --with-python=${PYTHON_EXEC}
./b2 -j$(nproc) install
ldconfig
rm -rf /${BOOST_SRC}

echo "--- Python-specific dependencies for ${PY_VERSION} installed successfully. ---"