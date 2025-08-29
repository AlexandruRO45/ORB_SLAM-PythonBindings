"""
ORB-SLAM3 Python bindings

This package provides Python bindings for the Semi-Direct Visual Odometry system.
"""

from ._version import __version__, __url__, __dependencies__

try:
    from .orbslam3 import (
        TrackingState,
        Sensor,
        System,
    )

except ImportError as e:
    # This provides a much better error message if the C++ part failed.
    # Include the original error for more detailed debugging.
    raise ImportError(
        "Failed to import the compiled ORB-SLAM3 C++ core (orbslam3.so).\n"
        "Please make sure the package was installed correctly after a full compilation.\n"
        f"Original error: {e}"
    ) from e

# ---- APIs -----
# This list defines the public API of the package.
# When a user runs 'from orbslam3 import *', these are the names that get imported.
__all__ = [
    "System",               # The main class for interacting with SLAM
    # "IMU",                  # The IMU class for handling inertial measurements
    "Sensor",               # The sensor enum
    "TrackingState",        # The tracking state enum

    # Metadata
    "__version__",
    "__url__",
    "__dependencies__",
]