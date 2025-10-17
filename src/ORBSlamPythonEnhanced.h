/**
 * @file ORBSlamPythonEnhanced.h
 * @brief Enhanced Python wrapper for ORB-SLAM3 with hardware adaptation,
 *        power mode management, and comprehensive metrics for VNAV integration.
 * @author AlexandruRO45
 * @date October 2025
 */

#ifndef ORBSlamPythonEnhanced_H
#define ORBSlamPythonEnhanced_H

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>

// OpenCV
#include <opencv2/core/core.hpp>

// ORB-SLAM3 Core
#include "System.h"
#include "Tracking.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

// Math Libraries
#include <Eigen/Dense>
#include <sophus/se3.hpp>

// Boost.Python
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/tuple.hpp>


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

namespace ORB_SLAM3 {
    class System;
    class Tracking;
    class Frame;
    class KeyFrame;
    class MapPoint;
}


// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Power mode for adaptive processing
 */
enum class PowerMode {
    HIGH = 0,    ///< Maximum performance: all features, GPU acceleration
    LOW = 1      ///< Power saving: reduced features, CPU only
};

/**
 * @brief Tracking quality level
 */
enum class TrackingQuality {
    EXCELLENT = 4,   ///< >80% feature matches, stable tracking
    GOOD = 3,        ///< 60-80% matches
    FAIR = 2,        ///< 40-60% matches
    POOR = 1,        ///< <40% matches
    LOST = 0         ///< Tracking lost
};

/**
 * @brief Confidence level for pose estimation
 */
enum class ConfidenceLevel {
    CRITICAL = 4,    ///< All inputs available, optimal conditions
    ENHANCED = 3,    ///< Most inputs available
    BASIC = 2,       ///< Minimum inputs for operation
    MINIMAL = 1,     ///< Degraded mode
    NONE = 0         ///< No valid pose
};


// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Hardware capabilities detected on system
 */
struct HardwareCapabilities {
    int cpu_cores;              ///< Number of available CPU cores
    bool gpu_available;         ///< CUDA/OpenCL GPU available
    size_t memory_mb;           ///< System RAM in MB
    bool cuda_enabled;          ///< CUDA support enabled
    std::string cpu_model;      ///< CPU model name
    std::string gpu_model;      ///< GPU model name (if available)
    
    HardwareCapabilities();
};

/**
 * @brief Configuration for hardware-adapted parameters
 */
struct HardwareConfig {
    bool gpu_acceleration;      ///< Enable GPU processing
    int num_threads;            ///< Number of processing threads
    size_t max_memory_mb;       ///< Maximum memory allocation
    
    HardwareConfig();
    explicit HardwareConfig(const HardwareCapabilities& caps, PowerMode mode);
};

/**
 * @brief Processing parameters for adaptive operation
 */
struct ProcessingParams {
    // Feature extraction
    int max_features;           ///< Maximum ORB features to extract
    int min_features;           ///< Minimum features required
    float scale_factor;         ///< Pyramid scale factor
    int num_levels;             ///< Number of pyramid levels
    int ini_th_fast;            ///< Initial FAST threshold
    int min_th_fast;            ///< Minimum FAST threshold
    
    // Tracking
    float tracking_threshold;   ///< Minimum tracking quality threshold
    int max_frames_to_skip;     ///< Max frames to skip before reset
    bool use_motion_model;      ///< Enable motion model prediction
    
    // Optimization
    int local_ba_keyframes;     ///< Keyframes for local BA
    int local_ba_points;        ///< Points for local BA
    bool enable_loop_closing;   ///< Enable loop closure detection
    
    ProcessingParams();
    static ProcessingParams createHighMode();
    static ProcessingParams createLowMode();
};

/**
 * @brief Comprehensive tracking result with metrics
 */
struct TrackingResult {
    bool success;                   ///< Tracking succeeded
    Sophus::SE3f pose;             ///< Estimated camera pose (Tcw)
    double timestamp;               ///< Frame timestamp
    
    // Feature statistics
    int num_features_detected;      ///< Total features detected
    int num_features_tracked;       ///< Features successfully tracked
    int num_map_points;             ///< Active map points
    int num_keyframes;              ///< Total keyframes in map
    
    // Quality metrics
    float tracking_quality;         ///< Quality score [0.0-1.0]
    TrackingQuality quality_level;  ///< Discretized quality level
    ConfidenceLevel confidence;     ///< Overall confidence level
    
    // Timing
    double processing_time_ms;      ///< Frame processing time
    
    // State
    ORB_SLAM3::Tracking::eTrackingState state;  ///< Tracking state
    
    TrackingResult();
    bool isValid() const;
};

/**
 * @brief System performance metrics
 */
struct PerformanceMetrics {
    // Timing
    double avg_processing_time_ms;  ///< Average frame processing time
    double fps;                     ///< Frames per second
    double min_processing_time_ms;  ///< Minimum processing time
    double max_processing_time_ms;  ///< Maximum processing time
    
    // Memory
    size_t memory_usage_mb;         ///< Current memory usage
    size_t peak_memory_mb;          ///< Peak memory usage
    
    // Map statistics
    int total_keyframes;            ///< Total keyframes in map
    int total_map_points;           ///< Total 3D points in map
    int active_map_points;          ///< Currently tracked points
    
    // Tracking statistics
    int successful_frames;          ///< Frames tracked successfully
    int lost_frames;                ///< Frames where tracking lost
    int relocalization_count;       ///< Number of relocalizations
    
    PerformanceMetrics();
    void update(const TrackingResult& result);
    void reset();
};

/**
 * @brief Map information summary
 */
struct MapInfo {
    int num_keyframes;              ///< Number of keyframes
    int num_map_points;             ///< Number of 3D points
    Eigen::Vector3f map_center;     ///< Geometric center of map
    Eigen::Vector3f map_bounds_min; ///< Minimum bounds
    Eigen::Vector3f map_bounds_max; ///< Maximum bounds
    float coverage_area;            ///< Approximate coverage area (m²)
    double creation_time;           ///< Map creation timestamp
    
    MapInfo();
};


// ============================================================================
// MAIN WRAPPER CLASS
// ============================================================================

/**
 * @brief Enhanced Python wrapper for ORB-SLAM3 with VNAV integration
 * 
 * This class extends the basic ORBSlamPython wrapper with:
 * - Hardware detection and adaptation
 * - Power mode switching (HIGH/LOW)
 * - Comprehensive performance metrics
 * - Confidence-based outputs
 * - Thread-safe operation
 * - Backward compatibility with original interface
 */
class ORBSlamPythonEnhanced {
public:
    // ========================================================================
    // CONSTRUCTION & INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Constructor with automatic hardware detection
     * @param vocabFile Path to ORB vocabulary file
     * @param settingsFile Path to YAML settings file
     * @param sensorMode Sensor type (MONO, STEREO, RGBD, IMU_*)
     */
    ORBSlamPythonEnhanced(
        const std::string& vocabFile,
        const std::string& settingsFile,
        ORB_SLAM3::System::eSensor sensorMode = ORB_SLAM3::System::eSensor::RGBD
    );
    
    /**
     * @brief Constructor with explicit hardware configuration
     */
    ORBSlamPythonEnhanced(
        const std::string& vocabFile,
        const std::string& settingsFile,
        ORB_SLAM3::System::eSensor sensorMode,
        const HardwareConfig& hwConfig
    );
    
    /**
     * @brief Destructor
     */
    ~ORBSlamPythonEnhanced();
    
    /**
     * @brief Initialize the SLAM system
     * @return true if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Check if system is running
     */
    bool isRunning() const;
    
    
    // ========================================================================
    // CORE PROCESSING (Enhanced versions)
    // ========================================================================
    
    /**
     * @brief Process monocular frame with enhanced output
     * @param image Input image (grayscale or RGB)
     * @param timestamp Frame timestamp
     * @return Complete tracking result with metrics
     */
    TrackingResult processMonoEnhanced(cv::Mat image, double timestamp);
    
    /**
     * @brief Process monocular frame with IMU data
     */
    TrackingResult processMonoInertialEnhanced(
        cv::Mat image,
        double timestamp,
        boost::python::list imu_data
    );
    
    /**
     * @brief Process stereo frame
     */
    TrackingResult processStereoEnhanced(
        cv::Mat leftImage,
        cv::Mat rightImage,
        double timestamp
    );
    
    /**
     * @brief Process stereo frame with IMU
     */
    TrackingResult processStereoInertialEnhanced(
        cv::Mat leftImage,
        cv::Mat rightImage,
        double timestamp,
        boost::python::list imu_data
    );
    
    /**
     * @brief Process RGB-D frame
     */
    TrackingResult processRGBDEnhanced(
        cv::Mat image,
        cv::Mat depthImage,
        double timestamp
    );
    
    /**
     * @brief Process RGB-D frame with IMU
     */
    TrackingResult processRGBDInertialEnhanced(
        cv::Mat image,
        cv::Mat depthImage,
        double timestamp,
        boost::python::list imu_data
    );
    
    
    // ========================================================================
    // BACKWARD COMPATIBILITY (Original interface)
    // ========================================================================
    
    bool processMono(cv::Mat image, double timestamp);
    bool processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp);
    bool processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp);
    bool loadAndProcessMono(std::string imageFile, double timestamp);
    bool loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp);
    bool loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp);
    
    
    // ========================================================================
    // HARDWARE ADAPTATION
    // ========================================================================
    
    /**
     * @brief Detect system hardware capabilities
     * @return Hardware capabilities structure
     */
    static HardwareCapabilities detectHardware();
    
    /**
     * @brief Get current hardware configuration
     */
    HardwareConfig getHardwareConfig() const;
    
    /**
     * @brief Set power mode (adapts parameters automatically)
     * @param mode HIGH for maximum performance, LOW for power saving
     */
    void setPowerMode(PowerMode mode);
    
    /**
     * @brief Get current power mode
     */
    PowerMode getPowerMode() const;
    
    /**
     * @brief Update processing parameters
     * @param params Custom processing parameters
     */
    void updateParameters(const ProcessingParams& params);
    
    /**
     * @brief Get current processing parameters
     */
    ProcessingParams getParameters() const;
    
    
    // ========================================================================
    // METRICS & MONITORING
    // ========================================================================
    
    /**
     * @brief Get comprehensive performance metrics
     */
    PerformanceMetrics getMetrics() const;
    
    /**
     * @brief Get current tracking quality
     * @return Quality score [0.0-1.0]
     */
    float getTrackingQuality() const;
    
    /**
     * @brief Get tracking state
     */
    ORB_SLAM3::Tracking::eTrackingState getTrackingState() const;
    
    /**
     * @brief Get number of features detected in current frame
     */
    unsigned int getNumFeatures() const;
    
    /**
     * @brief Get number of matched features
     */
    unsigned int getNumMatches() const;
    
    /**
     * @brief Get confidence level of current pose estimate
     */
    ConfidenceLevel getConfidenceLevel() const;
    
    /**
     * @brief Reset performance metrics
     */
    void resetMetrics();
    
    
    // ========================================================================
    // MAP & TRAJECTORY ACCESS
    // ========================================================================
    
    /**
     * @brief Get map information summary
     */
    MapInfo getMapInfo() const;
    
    /**
     * @brief Get current frame pose as 4x4 matrix
     * @return NumPy array or None
     */
    PyObject* getFramePose() const;
    
    /**
     * @brief Get camera intrinsic matrix
     */
    PyObject* getCameraMatrix() const;
    
    /**
     * @brief Get distortion coefficients
     */
    boost::python::tuple getDistCoeff() const;
    
    /**
     * @brief Get keyframe poses
     * @return List of (timestamp, R, t) tuples
     */
    boost::python::list getKeyframePoints() const;
    
    /**
     * @brief Get full trajectory points
     * @return List of (timestamp, 4x4 pose) tuples
     */
    boost::python::list getTrajectoryPoints() const;
    
    /**
     * @brief Get currently tracked map points
     * @return List of (x, y, z) tuples
     */
    boost::python::list getTrackedMappoints() const;
    
    /**
     * @brief Get current frame's map points with 2D projections
     * @return List of ((x3d, y3d, z3d), (x2d, y2d)) tuples
     */
    boost::python::list getCurrentPoints() const;
    
    
    // ========================================================================
    // SYSTEM CONTROL
    // ========================================================================
    
    /**
     * @brief Reset the SLAM system
     */
    void reset();
    
    /**
     * @brief Shutdown the system gracefully
     */
    void shutdown();
    
    /**
     * @brief Activate localization-only mode (stop mapping)
     */
    void activateLocalizationMode();
    
    /**
     * @brief Deactivate localization mode (resume full SLAM)
     */
    void deactivateLocalizationMode();
    
    /**
     * @brief Set sensor mode
     */
    void setMode(ORB_SLAM3::System::eSensor mode);
    
    /**
     * @brief Enable/disable viewer
     */
    void setUseViewer(bool useViewer);
    
    /**
     * @brief Set RGB vs BGR mode
     */
    void setRGBMode(bool rgb);
    
    
    // ========================================================================
    // SETTINGS MANAGEMENT
    // ========================================================================
    
    bool saveSettings(boost::python::dict settings) const;
    boost::python::dict loadSettings() const;
    static bool saveSettingsFile(boost::python::dict settings, std::string settingsFilename);
    static boost::python::dict loadSettingsFile(std::string settingsFilename);
    
    
private:
    // ========================================================================
    // PRIVATE MEMBERS
    // ========================================================================
    
    // Core SLAM system
    std::shared_ptr<ORB_SLAM3::System> system_;
    
    // Configuration
    std::string vocabulary_file_;
    std::string settings_file_;
    ORB_SLAM3::System::eSensor sensor_mode_;
    bool use_viewer_;
    bool use_rgb_;
    
    // Hardware & Performance
    HardwareCapabilities hardware_caps_;
    PowerMode power_mode_;              
    HardwareConfig hardware_config_;
    ProcessingParams processing_params_;
    
    // Metrics
    mutable std::mutex metrics_mutex_;
    PerformanceMetrics metrics_;
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    
    // Timing
    std::chrono::high_resolution_clock::time_point frame_start_time_;
    std::vector<double> processing_times_;  // Rolling window for FPS
    static constexpr size_t MAX_TIMING_SAMPLES = 30;
    
    
    // ========================================================================
    // PRIVATE HELPER METHODS
    // ========================================================================
    
    /**
     * @brief Start frame timing
     */
    void startFrameTiming();
    
    /**
     * @brief End frame timing and update metrics
     */
    void endFrameTiming();
    
    /**
     * @brief Calculate tracking quality from current frame
     */
    float calculateTrackingQuality() const;
    
    /**
     * @brief Calculate confidence level based on inputs and quality
     */
    ConfidenceLevel calculateConfidenceLevel(
        bool hasImage,
        bool hasDepth,
        bool hasIMU,
        float trackingQuality
    ) const;
    
    /**
     * @brief Apply power mode parameters to ORB-SLAM3 system
     */
    void applyPowerModeParameters();
    
    /**
     * @brief Parse IMU data from Python list
     */
    static std::vector<ORB_SLAM3::IMU::Point> parseIMUData(boost::python::list imu_data);
    
    /**
     * @brief Convert Sophus::SE3f to cv::Mat (4x4)
     */
    static cv::Mat se3fToCvMat4f(const Sophus::SE3f& pose);
    
    /**
     * @brief Get current memory usage in MB
     */
    static size_t getCurrentMemoryUsageMB();
};


// ============================================================================
// HARDWARE DETECTOR (Separate utility class)
// ============================================================================

/**
 * @brief Utility class for hardware capability detection
 */
class HardwareDetector {
public:
    /**
     * @brief Detect all hardware capabilities
     */
    static HardwareCapabilities detect();
    
private:
    static int detectCPUCores();
    static bool detectGPU();
    static size_t detectMemory();
    static bool checkCUDA();
    static std::string getCPUModel();
    static std::string getGPUModel();
};


#endif // ORBSlamPythonEnhanced_H
