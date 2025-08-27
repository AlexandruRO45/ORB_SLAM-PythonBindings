#!/bin/bash
set -e
set -o pipefail

# === ADD THIS DEBUGGING BLOCK START ===
echo "--- Verifying the Python Environment ---"
echo "Running Python version: $(python --version)"
echo "Path to Python executable: $(which python)"
echo "Searching for Python.h header file..."
# This command will find the main Python header if it exists.
# The '|| true' ensures the script doesn't fail if it's not found.
find /opt/python -name "Python.h" || true
echo "--- Verification Complete ---"
# === ADD THIS DEBUGGING BLOCK END ===

echo "--- Installing Python-specific dependencies... ---"
PY_VERSION=$(python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')")
case $PY_VERSION in
    38|39)
        echo "Installing python${PY_VERSION}-devel from yum..."
        yum install -y "python${PY_VERSION}-devel"
        ;;
    *)
        echo "Skipping yum install, headers for Python ${PY_VERSION} are pre-included by cibuildwheel."
        ;;
esac

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