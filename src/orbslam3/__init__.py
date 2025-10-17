"""
ORB-SLAM3 Python bindings with Enhanced Features

This package provides comprehensive Python bindings for ORB-SLAM3 with:
- Hardware-adaptive processing (CPU/GPU detection)
- Power mode management (HIGH/LOW)
- Rich tracking results with metrics
- Confidence-based outputs
- Full backward compatibility
"""

from ._version import __version__, __url__, __dependencies__


try:
    # Import enhanced module instead of orbslam3
    from .orbslam3_enhanced import (
        # Main SLAM System
        System,
        
        # Enumerations - Core
        TrackingState,
        Sensor,
        
        # Enumerations - Enhanced
        PowerMode,
        TrackingQuality,
        ConfidenceLevel,
        
        # Data Structures - Enhanced
        HardwareCapabilities,
        HardwareConfig,
        ProcessingParams,
        TrackingResult,
        PerformanceMetrics,
        MapInfo,
    )

except ImportError as e:
    # Fallback to original module if enhanced not available
    try:
        from .orbslam3 import (
            System,
            TrackingState,
            Sensor,
        )
        # Set dummy classes for missing enhanced features
        PowerMode = None
        TrackingQuality = None
        ConfidenceLevel = None
        HardwareCapabilities = None
        HardwareConfig = None
        ProcessingParams = None
        TrackingResult = None
        PerformanceMetrics = None
        MapInfo = None
        
        import warnings
        warnings.warn(
            "Using legacy orbslam3 module. Enhanced features not available. "
            "Please recompile with enhanced bindings for full functionality.",
            ImportWarning
        )
    except ImportError as e_fallback:
        # Neither module available - critical error
        raise ImportError(
            "Failed to import ORB-SLAM3 C++ core (neither orbslam3_enhanced.so nor orbslam3.so found).\n"
            "Please make sure the package was installed correctly after a full compilation.\n"
            f"Enhanced module error: {e}\n"
            f"Legacy module error: {e_fallback}"
        ) from e


# ============================================================================
# PUBLIC API DEFINITION
# ============================================================================

__all__ = [
    # ---- Core Classes ----
    "System",                   # Main SLAM system interface
    
    # ---- Core Enumerations ----
    "Sensor",                   # Sensor types (MONO, STEREO, RGBD, IMU_*)
    "TrackingState",            # Tracking states (OK, LOST, NOT_INITIALIZED, etc.)
    
    # ---- Enhanced Enumerations ----
    "PowerMode",                # Power modes (HIGH, LOW)
    "TrackingQuality",          # Quality levels (EXCELLENT, GOOD, FAIR, POOR, LOST)
    "ConfidenceLevel",          # Confidence levels (CRITICAL, ENHANCED, BASIC, MINIMAL, NONE)
    
    # ---- Enhanced Data Structures ----
    "HardwareCapabilities",     # System hardware detection results
    "HardwareConfig",           # Hardware configuration for SLAM
    "ProcessingParams",         # Algorithm parameters
    "TrackingResult",           # Rich tracking output with metrics
    "PerformanceMetrics",       # System performance statistics
    "MapInfo",                  # Map information summary
    
    # ---- Metadata ----
    "__version__",
    "__url__",
    "__dependencies__",
]


# ============================================================================
# CONVENIENCE FUNCTIONS
# ============================================================================

def detect_hardware():
    """
    Detect system hardware capabilities.
    
    Returns:
        HardwareCapabilities: Detected hardware information
        
    Example:
        >>> import orbslam3
        >>> caps = orbslam3.detect_hardware()
        >>> print(f"CPU cores: {caps.cpu_cores}")
        >>> print(f"GPU available: {caps.gpu_available}")
        >>> print(f"Memory: {caps.memory_mb} MB")
    """
    if System is None:
        raise RuntimeError("Enhanced features not available")
    return System.detect_hardware()


def create_system(vocab_file, settings_file, sensor_mode, power_mode="HIGH"):
    """
    Create and initialize an ORB-SLAM3 system with automatic hardware detection.
    
    Args:
        vocab_file (str): Path to ORB vocabulary file
        settings_file (str): Path to YAML settings file
        sensor_mode (Sensor): Sensor type (Sensor.MONOCULAR, Sensor.STEREO, etc.)
        power_mode (str): "HIGH" or "LOW" power mode
        
    Returns:
        System: Initialized SLAM system
        
    Example:
        >>> import orbslam3
        >>> slam = orbslam3.create_system(
        ...     "ORBvoc.txt",
        ...     "TUM1.yaml",
        ...     orbslam3.Sensor.MONOCULAR,
        ...     power_mode="HIGH"
        ... )
        >>> slam.initialize()
    """
    system = System(vocab_file, settings_file, sensor_mode)
    
    if PowerMode is not None:
        mode = PowerMode.HIGH if power_mode.upper() == "HIGH" else PowerMode.LOW
        system.set_power_mode(mode)
    
    return system


def get_version_info():
    """
    Get detailed version information about the ORB-SLAM3 Python bindings.
    
    Returns:
        dict: Version information including package version, dependencies, and features
        
    Example:
        >>> import orbslam3
        >>> info = orbslam3.get_version_info()
        >>> print(info)
    """
    return {
        "version": __version__,
        "url": __url__,
        "dependencies": __dependencies__,
        "enhanced_features": PowerMode is not None,
        "features": {
            "hardware_detection": HardwareCapabilities is not None,
            "power_modes": PowerMode is not None,
            "confidence_levels": ConfidenceLevel is not None,
            "performance_metrics": PerformanceMetrics is not None,
        }
    }


# Add convenience functions to __all__
__all__.extend([
    "detect_hardware",
    "create_system",
    "get_version_info",
])


# ============================================================================
# MODULE-LEVEL DOCUMENTATION
# ============================================================================

__doc__ = """
ORB-SLAM3 Enhanced Python Bindings
==================================

This package provides comprehensive Python bindings for ORB-SLAM3 with hardware 
adaptation and enhanced metrics.

Quick Start
-----------

Basic usage::

    import orbslam3
    import cv2
    
    # Create SLAM system
    slam = orbslam3.System(
        "ORBvoc.txt",
        "TUM1.yaml",
        orbslam3.Sensor.MONOCULAR
    )
    slam.initialize()
    
    # Process frames
    img = cv2.imread("frame.jpg")
    result = slam.process_mono_enhanced(img, timestamp)
    
    if result.success:
        print(f"Pose: {result.pose}")
        print(f"Quality: {result.tracking_quality}")
        print(f"Confidence: {result.confidence}")

Hardware Adaptation
------------------

Automatic hardware detection and power modes::

    # Detect hardware
    caps = orbslam3.detect_hardware()
    print(f"CPU cores: {caps.cpu_cores}")
    print(f"GPU: {caps.gpu_available}")
    
    # Set power mode
    slam.set_power_mode(orbslam3.PowerMode.HIGH)  # Max performance
    # or
    slam.set_power_mode(orbslam3.PowerMode.LOW)   # Power saving

Performance Monitoring
---------------------

Get comprehensive metrics::

    metrics = slam.get_metrics()
    print(f"FPS: {metrics.fps}")
    print(f"Memory: {metrics.memory_usage_mb} MB")
    print(f"Keyframes: {metrics.total_keyframes}")

Available Modes
--------------

Sensor Modes:
    - MONOCULAR: Single camera
    - STEREO: Stereo camera pair
    - RGBD: RGB-D camera
    - IMU_MONOCULAR: Monocular + IMU
    - IMU_STEREO: Stereo + IMU
    - IMU_RGBD: RGB-D + IMU

Power Modes:
    - HIGH: Maximum features, GPU acceleration, multi-threaded
    - LOW: Reduced features, CPU only, power efficient

Confidence Levels:
    - CRITICAL: All inputs available, optimal tracking
    - ENHANCED: Most inputs available, good tracking
    - BASIC: Minimum inputs, acceptable tracking
    - MINIMAL: Degraded mode
    - NONE: No valid tracking

For more information, visit: """ + __url__
