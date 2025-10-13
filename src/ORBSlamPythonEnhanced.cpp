/**
 * @file ORBSlamPythonEnhanced.cpp
 * @brief Implementation of enhanced Python wrapper for ORB-SLAM3
 * @author AlexandruRO45
 * @date October 2025
 */

#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API

#include "ORBSlamPythonEnhanced.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Boost Python CV Converter
#include <pyboostcvconverter/pyboostcvconverter.hpp>

// ORB-SLAM3 headers
#include "System.h"
#include "Tracking.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"

// Standard library
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>

// Platform-specific headers for hardware detection
#ifdef _WIN32
    #include <windows.h>
    #include <sysinfoapi.h>
#elif __linux__
    #include <sys/sysinfo.h>
    #include <unistd.h>
    #include <fstream>
#elif __APPLE__
    #include <sys/sysctl.h>
    #include <sys/types.h>
#endif

// CUDA detection (if available)
#ifdef CUDA_ENABLED
    #if defined(__has_include)
        #if __has_include(<cuda_runtime.h>)
            #include <cuda_runtime.h>
            #define HAS_CUDA_RUNTIME 1
        #else
            #define HAS_CUDA_RUNTIME 0
            #warning "CUDA requested but cuda_runtime.h not found - proceeding without GPU support"
        #endif
    #else
        // Compiler doesn't support __has_include, try to include anyway
        #ifdef __CUDACC__
            #include <cuda_runtime.h>
            #define HAS_CUDA_RUNTIME 1
        #else
            #define HAS_CUDA_RUNTIME 0
            #warning "CUDA requested but CUDA compiler not detected - proceeding without GPU support"
        #endif
    #endif
#else
    #define HAS_CUDA_RUNTIME 0
#endif


// ============================================================================
// NUMPY INITIALIZATION
// ============================================================================

#if PY_VERSION_HEX >= 0x03000000
#define NUMPY_IMPORT_ARRAY_RETVAL NULL
#else
#define NUMPY_IMPORT_ARRAY_RETVAL
#endif

#if (PY_VERSION_HEX >= 0x03000000)
static void *init_ar()
{
#else
static void init_ar()
{
#endif
    Py_Initialize();
    import_array();
    return NUMPY_IMPORT_ARRAY_RETVAL;
}


// ============================================================================
// DATA STRUCTURE IMPLEMENTATIONS
// ============================================================================

// HardwareCapabilities
// ----------------------------------------------------------------------------
HardwareCapabilities::HardwareCapabilities()
    : cpu_cores(0)
    , gpu_available(false)
    , memory_mb(0)
    , cuda_enabled(false)
    , cpu_model("Unknown")
    , gpu_model("Unknown")
{}

// HardwareConfig
// ----------------------------------------------------------------------------
HardwareConfig::HardwareConfig()
    : gpu_acceleration(false)
    , num_threads(1)
    , max_memory_mb(2048)
{}

HardwareConfig::HardwareConfig(const HardwareCapabilities& caps, PowerMode mode)
{
    if (mode == PowerMode::HIGH) {
        gpu_acceleration = caps.gpu_available;
        num_threads = std::max(1, caps.cpu_cores - 1); // Leave 1 core for system
        max_memory_mb = caps.memory_mb / 2; // Use up to 50% of RAM
    } else { // PowerMode::LOW
        gpu_acceleration = false;
        num_threads = 1;
        max_memory_mb = std::min(caps.memory_mb / 4, 1024UL); // Max 1GB or 25% RAM
    }
}

// ProcessingParams
// ----------------------------------------------------------------------------
ProcessingParams::ProcessingParams()
    : max_features(2000)
    , min_features(1000)
    , scale_factor(1.2f)
    , num_levels(8)
    , ini_th_fast(20)
    , min_th_fast(7)
    , tracking_threshold(0.5f)
    , max_frames_to_skip(5)
    , use_motion_model(true)
    , local_ba_keyframes(10)
    , local_ba_points(1000)
    , enable_loop_closing(true)
{}

ProcessingParams ProcessingParams::createHighMode()
{
    ProcessingParams params;
    params.max_features = 2000;
    params.min_features = 1000;
    params.num_levels = 8;
    params.ini_th_fast = 20;
    params.min_th_fast = 7;
    params.tracking_threshold = 0.5f;
    params.use_motion_model = true;
    params.enable_loop_closing = true;
    return params;
}

ProcessingParams ProcessingParams::createLowMode()
{
    ProcessingParams params;
    params.max_features = 1000;
    params.min_features = 500;
    params.num_levels = 4;
    params.ini_th_fast = 15;
    params.min_th_fast = 5;
    params.tracking_threshold = 0.3f;
    params.use_motion_model = true;
    params.enable_loop_closing = false;
    return params;
}

// TrackingResult
// ----------------------------------------------------------------------------
TrackingResult::TrackingResult()
    : success(false)
    , pose(Sophus::SE3f())
    , timestamp(0.0)
    , num_features_detected(0)
    , num_features_tracked(0)
    , num_map_points(0)
    , num_keyframes(0)
    , tracking_quality(0.0f)
    , quality_level(TrackingQuality::LOST)
    , confidence(ConfidenceLevel::NONE)
    , processing_time_ms(0.0)
    , state(ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
{}

bool TrackingResult::isValid() const
{
    return success && pose.so3().log().allFinite();
}

// PerformanceMetrics
// ----------------------------------------------------------------------------
PerformanceMetrics::PerformanceMetrics()
    : avg_processing_time_ms(0.0)
    , fps(0.0)
    , min_processing_time_ms(std::numeric_limits<double>::max())
    , max_processing_time_ms(0.0)
    , memory_usage_mb(0)
    , peak_memory_mb(0)
    , total_keyframes(0)
    , total_map_points(0)
    , active_map_points(0)
    , successful_frames(0)
    , lost_frames(0)
    , relocalization_count(0)
{}

void PerformanceMetrics::update(const TrackingResult& result)
{
    if (result.success) {
        successful_frames++;
    } else {
        lost_frames++;
    }
    
    // Update timing
    min_processing_time_ms = std::min(min_processing_time_ms, result.processing_time_ms);
    max_processing_time_ms = std::max(max_processing_time_ms, result.processing_time_ms);
    
    // Update map statistics
    total_keyframes = result.num_keyframes;
    total_map_points = result.num_map_points;
    active_map_points = result.num_features_tracked;
}

void PerformanceMetrics::reset()
{
    *this = PerformanceMetrics();
}

// MapInfo
// ----------------------------------------------------------------------------
MapInfo::MapInfo()
    : num_keyframes(0)
    , num_map_points(0)
    , map_center(Eigen::Vector3f::Zero())
    , map_bounds_min(Eigen::Vector3f::Zero())
    , map_bounds_max(Eigen::Vector3f::Zero())
    , coverage_area(0.0f)
    , creation_time(0.0)
{}


// ============================================================================
// HARDWARE DETECTOR IMPLEMENTATION
// ============================================================================

HardwareCapabilities HardwareDetector::detect()
{
    HardwareCapabilities caps;
    
    caps.cpu_cores = detectCPUCores();
    caps.memory_mb = detectMemory();
    caps.gpu_available = detectGPU();
    caps.cuda_enabled = checkCUDA();
    caps.cpu_model = getCPUModel();
    caps.gpu_model = getGPUModel();
    
    return caps;
}

int HardwareDetector::detectCPUCores()
{
#ifdef _WIN32
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    return sysinfo.dwNumberOfProcessors;
#elif __linux__ || __APPLE__
    return sysconf(_SC_NPROCESSORS_ONLN);
#else
    return std::thread::hardware_concurrency();
#endif
}

size_t HardwareDetector::detectMemory()
{
#ifdef _WIN32
    MEMORYSTATUSEX memInfo;
    memInfo.dwLength = sizeof(MEMORYSTATUSEX);
    GlobalMemoryStatusEx(&memInfo);
    return static_cast<size_t>(memInfo.ullTotalPhys / (1024 * 1024));
#elif __linux__
    struct sysinfo memInfo;
    sysinfo(&memInfo);
    return (memInfo.totalram * memInfo.mem_unit) / (1024 * 1024);
#elif __APPLE__
    int mib[2];
    int64_t physical_memory;
    size_t length;
    mib[0] = CTL_HW;
    mib[1] = HW_MEMSIZE;
    length = sizeof(int64_t);
    sysctl(mib, 2, &physical_memory, &length, NULL, 0);
    return physical_memory / (1024 * 1024);
#else
    return 0;
#endif
}

bool HardwareDetector::detectGPU()
{
#ifdef CUDA_ENABLED
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    return (error == cudaSuccess && deviceCount > 0);
#else
    // Check for OpenCL or other GPU frameworks if needed
    return false;
#endif
}

bool HardwareDetector::checkCUDA()
{
#ifdef CUDA_ENABLED
    return detectGPU();
#else
    return false;
#endif
}

std::string HardwareDetector::getCPUModel()
{
#ifdef __linux__
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.find("model name") != std::string::npos) {
            size_t pos = line.find(':');
            if (pos != std::string::npos) {
                return line.substr(pos + 2);
            }
        }
    }
#endif
    return "Unknown CPU";
}

std::string HardwareDetector::getGPUModel()
{
#ifdef CUDA_ENABLED
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    if (deviceCount > 0) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        return std::string(prop.name);
    }
#endif
    return "No GPU";
}


// ============================================================================
// MAIN CLASS IMPLEMENTATION
// ============================================================================

// Construction & Initialization
// ----------------------------------------------------------------------------

ORBSlamPythonEnhanced::ORBSlamPythonEnhanced(
    const std::string& vocabFile,
    const std::string& settingsFile,
    ORB_SLAM3::System::eSensor sensorMode
)
    : vocabulary_file_(vocabFile)
    , settings_file_(settingsFile)
    , sensor_mode_(sensorMode)
    , use_viewer_(false)
    , use_rgb_(true)
    , power_mode_(PowerMode::HIGH)
{
    // Detect hardware capabilities
    hardware_caps_ = HardwareDetector::detect();
    
    // Configure based on detected hardware and power mode
    hardware_config_ = HardwareConfig(hardware_caps_, power_mode_);
    
    // Set initial processing parameters
    processing_params_ = ProcessingParams::createHighMode();
    
    // Initialize timing
    processing_times_.reserve(MAX_TIMING_SAMPLES);
}

ORBSlamPythonEnhanced::ORBSlamPythonEnhanced(
    const std::string& vocabFile,
    const std::string& settingsFile,
    ORB_SLAM3::System::eSensor sensorMode,
    const HardwareConfig& hwConfig
)
    : vocabulary_file_(vocabFile)
    , settings_file_(settingsFile)
    , sensor_mode_(sensorMode)
    , use_viewer_(false)
    , use_rgb_(true)
    , power_mode_(PowerMode::HIGH)
    , hardware_config_(hwConfig)
{
    // Detect hardware capabilities
    hardware_caps_ = HardwareDetector::detect();
    
    // Set initial processing parameters
    processing_params_ = ProcessingParams::createHighMode();
    
    // Initialize timing
    processing_times_.reserve(MAX_TIMING_SAMPLES);
}

ORBSlamPythonEnhanced::~ORBSlamPythonEnhanced()
{
    if (system_) {
        system_->Shutdown();
    }
}

bool ORBSlamPythonEnhanced::initialize()
{
    try {
        system_ = std::make_shared<ORB_SLAM3::System>(
            vocabulary_file_,
            settings_file_,
            sensor_mode_,
            use_viewer_
        );
        
        // Apply initial power mode parameters
        applyPowerModeParameters();
        
        metrics_.reset();
        
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "ORBSlamPythonEnhanced::initialize() failed: " << e.what() << std::endl;
        return false;
    }
}

bool ORBSlamPythonEnhanced::isRunning() const
{
    return system_ != nullptr;
}


// Core Processing - Enhanced Methods
// ----------------------------------------------------------------------------

TrackingResult ORBSlamPythonEnhanced::processMonoEnhanced(
    cv::Mat image,
    double timestamp
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || image.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        // Process with ORB-SLAM3
        Sophus::SE3f pose = system_->TrackMonocular(image, timestamp);
        
        // Fill result
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        // Get feature statistics
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        // Calculate quality metrics
        result.tracking_quality = calculateTrackingQuality();
        
        if (result.tracking_quality >= 0.8f) {
            result.quality_level = TrackingQuality::EXCELLENT;
        } else if (result.tracking_quality >= 0.6f) {
            result.quality_level = TrackingQuality::GOOD;
        } else if (result.tracking_quality >= 0.4f) {
            result.quality_level = TrackingQuality::FAIR;
        } else if (result.tracking_quality >= 0.2f) {
            result.quality_level = TrackingQuality::POOR;
        } else {
            result.quality_level = TrackingQuality::LOST;
        }
        
        // Calculate confidence
        result.confidence = calculateConfidenceLevel(
            true,  // hasImage
            false, // hasDepth
            false, // hasIMU
            result.tracking_quality
        );
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processMonoEnhanced: " << e.what() << std::endl;
        result.success = false;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    // Update metrics
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}

TrackingResult ORBSlamPythonEnhanced::processMonoInertialEnhanced(
    cv::Mat image,
    double timestamp,
    boost::python::list imu_data
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || image.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        auto vImuMeas = parseIMUData(imu_data);
        Sophus::SE3f pose = system_->TrackMonocular(image, timestamp, vImuMeas);
        
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        // Feature statistics
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        result.tracking_quality = calculateTrackingQuality();
        result.confidence = calculateConfidenceLevel(true, false, true, result.tracking_quality);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processMonoInertialEnhanced: " << e.what() << std::endl;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}

TrackingResult ORBSlamPythonEnhanced::processStereoEnhanced(
    cv::Mat leftImage,
    cv::Mat rightImage,
    double timestamp
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || leftImage.empty() || rightImage.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        Sophus::SE3f pose = system_->TrackStereo(leftImage, rightImage, timestamp);
        
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        result.tracking_quality = calculateTrackingQuality();
        result.confidence = calculateConfidenceLevel(true, true, false, result.tracking_quality);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processStereoEnhanced: " << e.what() << std::endl;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}

TrackingResult ORBSlamPythonEnhanced::processStereoInertialEnhanced(
    cv::Mat leftImage,
    cv::Mat rightImage,
    double timestamp,
    boost::python::list imu_data
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || leftImage.empty() || rightImage.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        auto vImuMeas = parseIMUData(imu_data);
        Sophus::SE3f pose = system_->TrackStereo(leftImage, rightImage, timestamp, vImuMeas);
        
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        result.tracking_quality = calculateTrackingQuality();
        result.confidence = calculateConfidenceLevel(true, true, true, result.tracking_quality);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processStereoInertialEnhanced: " << e.what() << std::endl;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}

TrackingResult ORBSlamPythonEnhanced::processRGBDEnhanced(
    cv::Mat image,
    cv::Mat depthImage,
    double timestamp
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || image.empty() || depthImage.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        Sophus::SE3f pose = system_->TrackRGBD(image, depthImage, timestamp);
        
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        result.tracking_quality = calculateTrackingQuality();
        result.confidence = calculateConfidenceLevel(true, true, false, result.tracking_quality);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processRGBDEnhanced: " << e.what() << std::endl;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}

TrackingResult ORBSlamPythonEnhanced::processRGBDInertialEnhanced(
    cv::Mat image,
    cv::Mat depthImage,
    double timestamp,
    boost::python::list imu_data
)
{
    TrackingResult result;
    result.timestamp = timestamp;
    
    if (!system_ || image.empty() || depthImage.empty()) {
        return result;
    }
    
    startFrameTiming();
    
    try {
        auto vImuMeas = parseIMUData(imu_data);
        Sophus::SE3f pose = system_->TrackRGBD(image, depthImage, timestamp, vImuMeas);
        
        result.success = pose.so3().log().allFinite();
        result.pose = pose;
        result.state = static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
        
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            result.num_features_detected = pTracker->mCurrentFrame.mvKeys.size();
            result.num_features_tracked = getNumMatches();
            std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
            std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
            result.num_keyframes = vpKFs.size();
            result.num_map_points = vpMPs.size();
        }
        
        result.tracking_quality = calculateTrackingQuality();
        result.confidence = calculateConfidenceLevel(true, true, true, result.tracking_quality);
        
    } catch (const std::exception& e) {
        std::cerr << "Error in processRGBDInertialEnhanced: " << e.what() << std::endl;
    }
    
    endFrameTiming();
    result.processing_time_ms = 
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - frame_start_time_
        ).count();
    
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        metrics_.update(result);
    }
    
    return result;
}


// Backward Compatibility Methods
// ----------------------------------------------------------------------------

bool ORBSlamPythonEnhanced::processMono(cv::Mat image, double timestamp)
{
    if (!system_ || image.empty()) {
        return false;
    }
    
    try {
        Sophus::SE3f pose = system_->TrackMonocular(image, timestamp);
        return pose.so3().log().allFinite();
    } catch (...) {
        return false;
    }
}

bool ORBSlamPythonEnhanced::processStereo(
    cv::Mat leftImage,
    cv::Mat rightImage,
    double timestamp
)
{
    if (!system_ || leftImage.empty() || rightImage.empty()) {
        return false;
    }
    
    try {
        Sophus::SE3f pose = system_->TrackStereo(leftImage, rightImage, timestamp);
        return pose.so3().log().allFinite();
    } catch (...) {
        return false;
    }
}

bool ORBSlamPythonEnhanced::processRGBD(
    cv::Mat image,
    cv::Mat depthImage,
    double timestamp
)
{
    if (!system_ || image.empty() || depthImage.empty()) {
        return false;
    }
    
    try {
        Sophus::SE3f pose = system_->TrackRGBD(image, depthImage, timestamp);
        return pose.so3().log().allFinite();
    } catch (...) {
        return false;
    }
}

bool ORBSlamPythonEnhanced::loadAndProcessMono(
    std::string imageFile,
    double timestamp
)
{
    if (!system_) {
        return false;
    }
    
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (im.empty()) {
        return false;
    }
    
    if (use_rgb_) {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    
    return processMono(im, timestamp);
}

bool ORBSlamPythonEnhanced::loadAndProcessStereo(
    std::string leftImageFile,
    std::string rightImageFile,
    double timestamp
)
{
    if (!system_) {
        return false;
    }
    
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    
    if (leftImage.empty() || rightImage.empty()) {
        return false;
    }
    
    if (use_rgb_) {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    
    return processStereo(leftImage, rightImage, timestamp);
}

bool ORBSlamPythonEnhanced::loadAndProcessRGBD(
    std::string imageFile,
    std::string depthImageFile,
    double timestamp
)
{
    if (!system_) {
        return false;
    }
    
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    
    if (im.empty() || imDepth.empty()) {
        return false;
    }
    
    if (use_rgb_) {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    
    return processRGBD(im, imDepth, timestamp);
}


// ============================================================================
// HARDWARE ADAPTATION & POWER MANAGEMENT
// ============================================================================

HardwareCapabilities ORBSlamPythonEnhanced::detectHardware()
{
    return HardwareDetector::detect();
}

HardwareConfig ORBSlamPythonEnhanced::getHardwareConfig() const
{
    return hardware_config_;
}

void ORBSlamPythonEnhanced::setPowerMode(PowerMode mode)
{
    power_mode_ = mode;
    
    // Update processing parameters based on mode
    if (mode == PowerMode::HIGH) {
        processing_params_ = ProcessingParams::createHighMode();
    } else {
        processing_params_ = ProcessingParams::createLowMode();
    }
    
    // Update hardware config
    hardware_config_ = HardwareConfig(hardware_caps_, mode);
    
    // Apply changes to running system
    if (system_) {
        applyPowerModeParameters();
    }
}

PowerMode ORBSlamPythonEnhanced::getPowerMode() const
{
    return power_mode_;
}

void ORBSlamPythonEnhanced::updateParameters(const ProcessingParams& params)
{
    processing_params_ = params;
    
    if (system_) {
        applyPowerModeParameters();
    }
}

ProcessingParams ORBSlamPythonEnhanced::getParameters() const
{
    return processing_params_;
}

void ORBSlamPythonEnhanced::applyPowerModeParameters()
{
    // Note: ORB-SLAM3 doesn't provide runtime parameter updates directly
    // This would require modifying ORB-SLAM3 core or reinitializing the system
    // For now, parameters are applied at initialization
    // Future enhancement: Implement parameter hot-swapping
    
    // TODO: If ORB-SLAM3 is extended to support runtime parameter updates,
    // implement the logic here to update:
    // - ORB extractor parameters (nFeatures, nLevels, etc.)
    // - Tracking thresholds
    // - Local BA parameters
    // - Loop closing enable/disable
}


// ============================================================================
// METRICS & MONITORING
// ============================================================================

PerformanceMetrics ORBSlamPythonEnhanced::getMetrics() const
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    PerformanceMetrics metrics = metrics_;
    
    // Calculate FPS from processing times
    if (!processing_times_.empty()) {
        double avg_time = std::accumulate(
            processing_times_.begin(),
            processing_times_.end(),
            0.0
        ) / processing_times_.size();
        
        metrics.avg_processing_time_ms = avg_time;
        metrics.fps = (avg_time > 0) ? (1000.0 / avg_time) : 0.0;
    }
    
    // Get current memory usage
    metrics.memory_usage_mb = getCurrentMemoryUsageMB();
    metrics.peak_memory_mb = std::max(metrics.peak_memory_mb, metrics.memory_usage_mb);
    
    return metrics;
}

float ORBSlamPythonEnhanced::getTrackingQuality() const
{
    return calculateTrackingQuality();
}

ORB_SLAM3::Tracking::eTrackingState ORBSlamPythonEnhanced::getTrackingState() const
{
    if (system_) {
        return static_cast<ORB_SLAM3::Tracking::eTrackingState>(
            system_->GetTrackingState()
        );
    }
    return ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY;
}

unsigned int ORBSlamPythonEnhanced::getNumFeatures() const
{
    if (system_) {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            return pTracker->mCurrentFrame.mvKeys.size();
        }
    }
    return 0;
}

unsigned int ORBSlamPythonEnhanced::getNumMatches() const
{
    if (system_) {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            unsigned int matches = 0;
            unsigned int num = pTracker->mCurrentFrame.mvKeys.size();
            
            if (pTracker->mCurrentFrame.mvpMapPoints.size() < num) {
                num = pTracker->mCurrentFrame.mvpMapPoints.size();
            }
            if (pTracker->mCurrentFrame.mvbOutlier.size() < num) {
                num = pTracker->mCurrentFrame.mvbOutlier.size();
            }
            
            for (unsigned int i = 0; i < num; ++i) {
                ORB_SLAM3::MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP && !pTracker->mCurrentFrame.mvbOutlier[i] && pMP->Observations() > 0) {
                    ++matches;
                }
            }
            return matches;
        }
    }
    return 0;
}

ConfidenceLevel ORBSlamPythonEnhanced::getConfidenceLevel() const
{
    if (!system_) {
        return ConfidenceLevel::NONE;
    }
    
    float quality = calculateTrackingQuality();
    
    // Determine confidence based on available inputs
    bool hasImage = true; // Always true if we got here
    bool hasDepth = (sensor_mode_ == ORB_SLAM3::System::STEREO || 
                     sensor_mode_ == ORB_SLAM3::System::RGBD ||
                     sensor_mode_ == ORB_SLAM3::System::IMU_STEREO ||
                     sensor_mode_ == ORB_SLAM3::System::IMU_RGBD);
    bool hasIMU = (sensor_mode_ == ORB_SLAM3::System::IMU_MONOCULAR ||
                   sensor_mode_ == ORB_SLAM3::System::IMU_STEREO ||
                   sensor_mode_ == ORB_SLAM3::System::IMU_RGBD);
    
    return calculateConfidenceLevel(hasImage, hasDepth, hasIMU, quality);
}

void ORBSlamPythonEnhanced::resetMetrics()
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    metrics_.reset();
    processing_times_.clear();
}


// ============================================================================
// MAP & TRAJECTORY ACCESS
// ============================================================================

MapInfo ORBSlamPythonEnhanced::getMapInfo() const
{
    MapInfo info;
    
    if (!system_) {
        return info;
    }
    
    try {
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
        std::vector<ORB_SLAM3::MapPoint*> vpMPs = system_->GetTrackedMapPoints();
        
        info.num_keyframes = vpKFs.size();
        info.num_map_points = vpMPs.size();
        
        // Calculate map bounds and center from map points
        if (!vpMPs.empty()) {
            Eigen::Vector3f min_bound = Eigen::Vector3f::Constant(
                std::numeric_limits<float>::max()
            );
            Eigen::Vector3f max_bound = Eigen::Vector3f::Constant(
                std::numeric_limits<float>::lowest()
            );
            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            int valid_points = 0;
            
            for (ORB_SLAM3::MapPoint* pMP : vpMPs) {
                if (pMP && !pMP->isBad()) {
                    Eigen::Vector3f pos = pMP->GetWorldPos();
                    min_bound = min_bound.cwiseMin(pos);
                    max_bound = max_bound.cwiseMax(pos);
                    sum += pos;
                    valid_points++;
                }
            }
            
            if (valid_points > 0) {
                info.map_bounds_min = min_bound;
                info.map_bounds_max = max_bound;
                info.map_center = sum / valid_points;
                
                // Approximate coverage area (XY plane)
                Eigen::Vector3f extent = max_bound - min_bound;
                info.coverage_area = extent.x() * extent.y();
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error in getMapInfo: " << e.what() << std::endl;
    }
    
    return info;
}

PyObject* ORBSlamPythonEnhanced::getFramePose() const
{
    if (system_) {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            Sophus::SE3f poseSE3f = pTracker->mCurrentFrame.GetPose();
            if (poseSE3f.so3().log().allFinite()) {
                cv::Mat pose_cv = se3fToCvMat4f(poseSE3f);
                return pbcvt::fromMatToNDArray(pose_cv);
            }
        }
    }
    Py_RETURN_NONE;
}

PyObject* ORBSlamPythonEnhanced::getCameraMatrix() const
{
    if (system_) {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            cv::Mat cm = pTracker->mCurrentFrame.mK;
            return pbcvt::fromMatToNDArray(cm);
        }
    }
    Py_RETURN_NONE;
}

boost::python::tuple ORBSlamPythonEnhanced::getDistCoeff() const
{
    if (system_) {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (pTracker) {
            cv::Mat dist = pTracker->mCurrentFrame.mDistCoef;
            if (!dist.empty() && dist.rows >= 4) {
                return boost::python::make_tuple(
                    dist.at<float>(0),
                    dist.at<float>(1),
                    dist.at<float>(2),
                    dist.at<float>(3)
                );
            }
        }
    }
    return boost::python::tuple();
}

boost::python::list ORBSlamPythonEnhanced::getKeyframePoints() const
{
    boost::python::list trajectory;
    
    if (!system_) {
        return trajectory;
    }
    
    try {
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
        std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);
        
        for (size_t i = 0; i < vpKFs.size(); i++) {
            ORB_SLAM3::KeyFrame* pKF = vpKFs[i];
            
            if (pKF->isBad()) {
                continue;
            }
            
            Sophus::SE3f Tcw = pKF->GetPose();
            Sophus::SE3f Twc = Tcw.inverse();
            
            Eigen::Matrix3f R_eigen = Twc.rotationMatrix();
            Eigen::Vector3f t_eigen = Twc.translation();
            
            // Convert to cv::Mat
            cv::Mat R_cv(3, 3, CV_32F);
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    R_cv.at<float>(r, c) = R_eigen(r, c);
                }
            }
            
            cv::Mat t_cv(3, 1, CV_32F);
            for (int r = 0; r < 3; r++) {
                t_cv.at<float>(r, 0) = t_eigen(r);
            }
            
            PyObject* Rarr = pbcvt::fromMatToNDArray(R_cv);
            PyObject* Tarr = pbcvt::fromMatToNDArray(t_cv);
            
            trajectory.append(boost::python::make_tuple(
                pKF->mTimeStamp,
                boost::python::handle<>(Rarr),
                boost::python::handle<>(Tarr)
            ));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in getKeyframePoints: " << e.what() << std::endl;
    }
    
    return trajectory;
}

boost::python::list ORBSlamPythonEnhanced::getTrajectoryPoints() const
{
    boost::python::list trajectory;
    
    if (!system_) {
        return trajectory;
    }
    
    try {
        ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
        if (!pTracker) {
            return trajectory;
        }
        
        if (pTracker->mlpReferences.empty()) {
            return trajectory;
        }
        
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = system_->GetKeyFrames();
        std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);
        
        if (vpKFs.empty()) {
            return trajectory;
        }
        
        Sophus::SE3f Two = vpKFs[0]->GetPose().inverse();
        
        std::list<ORB_SLAM3::KeyFrame*>::const_iterator lRit = pTracker->mlpReferences.begin();
        std::list<double>::const_iterator lT = pTracker->mlFrameTimes.begin();
        
        for (std::list<Sophus::SE3f>::const_iterator lit = pTracker->mlRelativeFramePoses.begin(),
             lend = pTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++)
        {
            ORB_SLAM3::KeyFrame* pKF = *lRit;
            if (!pKF || pKF->isBad()) {
                continue;
            }
            
            Sophus::SE3f Trw = pKF->GetPose() * Two;
            Sophus::SE3f Tcw = (*lit) * Trw;
            
            cv::Mat Tcw_cv = se3fToCvMat4f(Tcw);
            PyObject* ndarr = pbcvt::fromMatToNDArray(Tcw_cv);
            
            trajectory.append(boost::python::make_tuple(
                *lT,
                boost::python::handle<>(ndarr)
            ));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in getTrajectoryPoints: " << e.what() << std::endl;
    }
    
    return trajectory;
}

boost::python::list ORBSlamPythonEnhanced::getTrackedMappoints() const
{
    boost::python::list map_points;
    
    if (!system_) {
        return map_points;
    }
    
    try {
        std::vector<ORB_SLAM3::MapPoint*> Mps = system_->GetTrackedMapPoints();
        
        for (size_t i = 0; i < Mps.size(); i++) {
            if (Mps[i] && !Mps[i]->isBad()) {
                Eigen::Vector3f wp = Mps[i]->GetWorldPos();
                map_points.append(boost::python::make_tuple(
                    wp(0),
                    wp(1),
                    wp(2)
                ));
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in getTrackedMappoints: " << e.what() << std::endl;
    }
    
    return map_points;
}

boost::python::list ORBSlamPythonEnhanced::getCurrentPoints() const
{
    boost::python::list map_points;
    
    if (system_) {
        try {
            ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
            if (pTracker) {
                const auto& currentFrame = pTracker->mCurrentFrame;
                
                for (size_t i = 0; i < static_cast<size_t>(currentFrame.N); ++i) {
                    ORB_SLAM3::MapPoint* pMP = currentFrame.mvpMapPoints[i];
                    if (pMP && !pMP->isBad() && 
                        i < currentFrame.mvbOutlier.size() && 
                        !currentFrame.mvbOutlier[i]) 
                    {
                        Eigen::Vector3f wp = pMP->GetWorldPos();
                        const cv::KeyPoint& kp = currentFrame.mvKeysUn[i];
                        
                        map_points.append(boost::python::make_tuple(
                            boost::python::make_tuple(wp(0), wp(1), wp(2)),
                            boost::python::make_tuple(kp.pt.x, kp.pt.y)
                        ));
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in getCurrentPoints: " << e.what() << std::endl;
        }
    }
    
    return map_points;
}


// ============================================================================
// SYSTEM CONTROL
// ============================================================================

void ORBSlamPythonEnhanced::reset()
{
    if (system_) {
        system_->Reset();
        resetMetrics();
    }
}

void ORBSlamPythonEnhanced::shutdown()
{
    if (system_) {
        system_->Shutdown();
        system_.reset();
    }
}

void ORBSlamPythonEnhanced::activateLocalizationMode()
{
    if (system_) {
        system_->ActivateLocalizationMode();
    }
}

void ORBSlamPythonEnhanced::deactivateLocalizationMode()
{
    if (system_) {
        system_->DeactivateLocalizationMode();
    }
}

void ORBSlamPythonEnhanced::setMode(ORB_SLAM3::System::eSensor mode)
{
    sensor_mode_ = mode;
}

void ORBSlamPythonEnhanced::setUseViewer(bool useViewer)
{
    use_viewer_ = useViewer;
}

void ORBSlamPythonEnhanced::setRGBMode(bool rgb)
{
    use_rgb_ = rgb;
}


// ============================================================================
// SETTINGS MANAGEMENT (Reuse original implementation)
// ============================================================================

bool ORBSlamPythonEnhanced::saveSettings(boost::python::dict settings) const
{
    return saveSettingsFile(settings, settings_file_);
}

boost::python::dict ORBSlamPythonEnhanced::loadSettings() const
{
    return loadSettingsFile(settings_file_);
}

bool ORBSlamPythonEnhanced::saveSettingsFile(
    boost::python::dict settings,
    std::string settingsFilename
)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);
    
    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index) {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check()) {
            continue;
        }
        std::string key = extractedKey;
        
        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check()) {
            fs << key << int(intValue);
            continue;
        }
        
        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check()) {
            fs << key << float(floatValue);
            continue;
        }
        
        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check()) {
            fs << key << std::string(stringValue);
            continue;
        }
    }
    
    return true;
}

boost::python::list readSequence(cv::FileNode fn, int depth = 10);
boost::python::dict readMap(cv::FileNode fn, int depth = 10);

boost::python::dict ORBSlamPythonEnhanced::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    cv::FileNode root = fs.root();
    
    if (root.isMap()) {
        return readMap(root);
    } else if (root.isSeq()) {
        boost::python::dict settings;
        settings["root"] = readSequence(root);
        return settings;
    }
    
    return boost::python::dict();
}


// ============================================================================
// PRIVATE HELPER METHODS
// ============================================================================

void ORBSlamPythonEnhanced::startFrameTiming()
{
    frame_start_time_ = std::chrono::high_resolution_clock::now();
}

void ORBSlamPythonEnhanced::endFrameTiming()
{
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double, std::milli>(
        end_time - frame_start_time_
    ).count();
    
    // Update rolling window of processing times
    processing_times_.push_back(duration);
    if (processing_times_.size() > MAX_TIMING_SAMPLES) {
        processing_times_.erase(processing_times_.begin());
    }
}

float ORBSlamPythonEnhanced::calculateTrackingQuality() const
{
    if (!system_) {
        return 0.0f;
    }
    
    ORB_SLAM3::Tracking* pTracker = system_->GetTracker();
    if (!pTracker) {
        return 0.0f;
    }
    
    unsigned int numFeatures = pTracker->mCurrentFrame.mvKeys.size();
    if (numFeatures == 0) {
        return 0.0f;
    }
    
    unsigned int numMatches = getNumMatches();
    
    float matchRatio = static_cast<float>(numMatches) / static_cast<float>(numFeatures);
    
    // Adjust quality based on tracking state
    ORB_SLAM3::Tracking::eTrackingState state = 
        static_cast<ORB_SLAM3::Tracking::eTrackingState>(system_->GetTrackingState());
    
    switch (state) {
        case ORB_SLAM3::Tracking::OK:
            return matchRatio;
        case ORB_SLAM3::Tracking::NOT_INITIALIZED:
            return 0.0f;
        case ORB_SLAM3::Tracking::LOST:
            return 0.0f;
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return matchRatio * 0.5f;
        default:
            return matchRatio * 0.5f;
    }
}

ConfidenceLevel ORBSlamPythonEnhanced::calculateConfidenceLevel(
    bool hasImage,
    bool hasDepth,
    bool hasIMU,
    float trackingQuality
) const
{
    // Count available inputs
    int inputScore = 0;
    if (hasImage) inputScore++;
    if (hasDepth) inputScore++;
    if (hasIMU) inputScore++;
    
    // Calculate confidence based on inputs and quality
    if (!hasImage || trackingQuality < 0.1f) {
        return ConfidenceLevel::NONE;
    }
    
    if (inputScore >= 3 && trackingQuality >= 0.7f) {
        return ConfidenceLevel::CRITICAL;  // All inputs + good tracking
    } else if (inputScore >= 2 && trackingQuality >= 0.5f) {
        return ConfidenceLevel::ENHANCED;  // Most inputs + fair tracking
    } else if (trackingQuality >= 0.3f) {
        return ConfidenceLevel::BASIC;     // Minimum viable tracking
    } else {
        return ConfidenceLevel::MINIMAL;   // Degraded mode
    }
}

std::vector<ORB_SLAM3::IMU::Point> ORBSlamPythonEnhanced::parseIMUData(
    boost::python::list imu_data
)
{
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    
    for (int i = 0; i < boost::python::len(imu_data); ++i) {
        boost::python::tuple measurement = 
            boost::python::extract<boost::python::tuple>(imu_data[i]);
        
        double ax = boost::python::extract<double>(measurement[0]);
        double ay = boost::python::extract<double>(measurement[1]);
        double az = boost::python::extract<double>(measurement[2]);
        double gx = boost::python::extract<double>(measurement[3]);
        double gy = boost::python::extract<double>(measurement[4]);
        double gz = boost::python::extract<double>(measurement[5]);
        double ts = boost::python::extract<double>(measurement[6]);
        
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(ax, ay, az, gx, gy, gz, ts));
    }
    
    return vImuMeas;
}

cv::Mat ORBSlamPythonEnhanced::se3fToCvMat4f(const Sophus::SE3f& pose)
{
    Eigen::Matrix4f eigen_mat = pose.matrix();
    cv::Mat cv_mat(4, 4, CV_32F);
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cv_mat.at<float>(i, j) = eigen_mat(i, j);
        }
    }
    
    return cv_mat;
}

size_t ORBSlamPythonEnhanced::getCurrentMemoryUsageMB()
{
#ifdef __linux__
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.find("VmRSS:") != std::string::npos) {
            std::istringstream iss(line.substr(7));
            size_t memory_kb;
            iss >> memory_kb;
            return memory_kb / 1024;
        }
    }
#elif _WIN32
    PROCESS_MEMORY_COUNTERS_EX pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
    return pmc.WorkingSetSize / (1024 * 1024);
#endif
    return 0;
}


// ============================================================================
// HELPER FUNCTIONS FOR SETTINGS (from original implementation)
// ============================================================================

boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    
    if (fn.isMap()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            std::string key = item.name();
            
            if (item.isNone()) {
                map[key] = boost::python::object();
            } else if (item.isInt()) {
                map[key] = int(item);
            } else if (item.isString()) {
                map[key] = std::string(item);
            } else if (item.isReal()) {
                map[key] = float(item);
            } else if (item.isSeq() && depth > 0) {
                map[key] = readSequence(item, depth - 1);
            } else if (item.isMap() && depth > 0) {
                map[key] = readMap(item, depth - 1);
            }
        }
    }
    
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    
    if (fn.isSeq()) {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it) {
            cv::FileNode item = *it;
            
            if (item.isNone()) {
                sequence.append(boost::python::object());
            } else if (item.isInt()) {
                sequence.append(int(item));
            } else if (item.isString()) {
                sequence.append(std::string(item));
            } else if (item.isReal()) {
                sequence.append(float(item));
            } else if (item.isSeq() && depth > 0) {
                sequence.append(readSequence(item, depth - 1));
            } else if (item.isMap() && depth > 0) {
                sequence.append(readMap(item, depth - 1));
            }
        }
    }
    
    return sequence;
}


// ============================================================================
// BOOST.PYTHON MODULE DEFINITION
// ============================================================================

BOOST_PYTHON_MODULE(orbslam3_enhanced)
{
    init_ar();
    
    // OpenCV Mat converters
    boost::python::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();
    
    
    // ========================================================================
    // ENUMERATIONS
    // ========================================================================
    
    boost::python::enum_<PowerMode>("PowerMode")
        .value("HIGH", PowerMode::HIGH)
        .value("LOW", PowerMode::LOW);
    
    boost::python::enum_<TrackingQuality>("TrackingQuality")
        .value("EXCELLENT", TrackingQuality::EXCELLENT)
        .value("GOOD", TrackingQuality::GOOD)
        .value("FAIR", TrackingQuality::FAIR)
        .value("POOR", TrackingQuality::POOR)
        .value("LOST", TrackingQuality::LOST);
    
    boost::python::enum_<ConfidenceLevel>("ConfidenceLevel")
        .value("CRITICAL", ConfidenceLevel::CRITICAL)
        .value("ENHANCED", ConfidenceLevel::ENHANCED)
        .value("BASIC", ConfidenceLevel::BASIC)
        .value("MINIMAL", ConfidenceLevel::MINIMAL)
        .value("NONE", ConfidenceLevel::NONE);
    
    boost::python::enum_<ORB_SLAM3::Tracking::eTrackingState>("TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM3::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM3::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM3::Tracking::eTrackingState::OK)
        .value("RECENTLY_LOST", ORB_SLAM3::Tracking::eTrackingState::RECENTLY_LOST)
        .value("LOST", ORB_SLAM3::Tracking::eTrackingState::LOST);
    
    boost::python::enum_<ORB_SLAM3::System::eSensor>("Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD);
    
    
    // ========================================================================
    // DATA STRUCTURES
    // ========================================================================
    
    boost::python::class_<HardwareCapabilities>("HardwareCapabilities", boost::python::init<>())
        .def_readwrite("cpu_cores", &HardwareCapabilities::cpu_cores)
        .def_readwrite("gpu_available", &HardwareCapabilities::gpu_available)
        .def_readwrite("memory_mb", &HardwareCapabilities::memory_mb)
        .def_readwrite("cuda_enabled", &HardwareCapabilities::cuda_enabled)
        .def_readwrite("cpu_model", &HardwareCapabilities::cpu_model)
        .def_readwrite("gpu_model", &HardwareCapabilities::gpu_model);
    
    boost::python::class_<HardwareConfig>("HardwareConfig", boost::python::init<>())
        .def_readwrite("gpu_acceleration", &HardwareConfig::gpu_acceleration)
        .def_readwrite("num_threads", &HardwareConfig::num_threads)
        .def_readwrite("max_memory_mb", &HardwareConfig::max_memory_mb);
    
    boost::python::class_<ProcessingParams>("ProcessingParams", boost::python::init<>())
        .def_readwrite("max_features", &ProcessingParams::max_features)
        .def_readwrite("min_features", &ProcessingParams::min_features)
        .def_readwrite("scale_factor", &ProcessingParams::scale_factor)
        .def_readwrite("num_levels", &ProcessingParams::num_levels)
        .def_readwrite("tracking_threshold", &ProcessingParams::tracking_threshold)
        .def("create_high_mode", &ProcessingParams::createHighMode)
        .staticmethod("create_high_mode")
        .def("create_low_mode", &ProcessingParams::createLowMode)
        .staticmethod("create_low_mode");
    
    boost::python::class_<TrackingResult>("TrackingResult", boost::python::init<>())
        .def_readwrite("success", &TrackingResult::success)
        .def_readwrite("timestamp", &TrackingResult::timestamp)
        .def_readwrite("num_features_detected", &TrackingResult::num_features_detected)
        .def_readwrite("num_features_tracked", &TrackingResult::num_features_tracked)
        .def_readwrite("num_map_points", &TrackingResult::num_map_points)
        .def_readwrite("num_keyframes", &TrackingResult::num_keyframes)
        .def_readwrite("tracking_quality", &TrackingResult::tracking_quality)
        .def_readwrite("quality_level", &TrackingResult::quality_level)
        .def_readwrite("confidence", &TrackingResult::confidence)
        .def_readwrite("processing_time_ms", &TrackingResult::processing_time_ms)
        .def_readwrite("state", &TrackingResult::state)
        .def("is_valid", &TrackingResult::isValid);
    
    boost::python::class_<PerformanceMetrics>("PerformanceMetrics", boost::python::init<>())
        .def_readwrite("avg_processing_time_ms", &PerformanceMetrics::avg_processing_time_ms)
        .def_readwrite("fps", &PerformanceMetrics::fps)
        .def_readwrite("min_processing_time_ms", &PerformanceMetrics::min_processing_time_ms)
        .def_readwrite("max_processing_time_ms", &PerformanceMetrics::max_processing_time_ms)
        .def_readwrite("memory_usage_mb", &PerformanceMetrics::memory_usage_mb)
        .def_readwrite("peak_memory_mb", &PerformanceMetrics::peak_memory_mb)
        .def_readwrite("total_keyframes", &PerformanceMetrics::total_keyframes)
        .def_readwrite("total_map_points", &PerformanceMetrics::total_map_points)
        .def_readwrite("successful_frames", &PerformanceMetrics::successful_frames)
        .def_readwrite("lost_frames", &PerformanceMetrics::lost_frames);
    
    boost::python::class_<MapInfo>("MapInfo", boost::python::init<>())
        .def_readwrite("num_keyframes", &MapInfo::num_keyframes)
        .def_readwrite("num_map_points", &MapInfo::num_map_points)
        .def_readwrite("coverage_area", &MapInfo::coverage_area);
    
    
    // ========================================================================
    // MAIN CLASS
    // ========================================================================
    
    boost::python::class_<ORBSlamPythonEnhanced, boost::noncopyable>(
        "System",
        boost::python::init<const std::string&, const std::string&, 
                           boost::python::optional<ORB_SLAM3::System::eSensor>>())
        
        // Initialization
        .def("initialize", &ORBSlamPythonEnhanced::initialize)
        .def("is_running", &ORBSlamPythonEnhanced::isRunning)
        
        // Enhanced processing methods
        .def("process_mono_enhanced", &ORBSlamPythonEnhanced::processMonoEnhanced)
        .def("process_mono_inertial_enhanced", &ORBSlamPythonEnhanced::processMonoInertialEnhanced)
        .def("process_stereo_enhanced", &ORBSlamPythonEnhanced::processStereoEnhanced)
        .def("process_stereo_inertial_enhanced", &ORBSlamPythonEnhanced::processStereoInertialEnhanced)
        .def("process_rgbd_enhanced", &ORBSlamPythonEnhanced::processRGBDEnhanced)
        .def("process_rgbd_inertial_enhanced", &ORBSlamPythonEnhanced::processRGBDInertialEnhanced)
        
        // Backward compatible methods
        .def("process_image_mono", &ORBSlamPythonEnhanced::processMono)
        .def("process_image_stereo", &ORBSlamPythonEnhanced::processStereo)
        .def("process_image_rgbd", &ORBSlamPythonEnhanced::processRGBD)
        .def("load_and_process_mono", &ORBSlamPythonEnhanced::loadAndProcessMono)
        .def("load_and_process_stereo", &ORBSlamPythonEnhanced::loadAndProcessStereo)
        .def("load_and_process_rgbd", &ORBSlamPythonEnhanced::loadAndProcessRGBD)
        
        // Hardware adaptation
        .def("detect_hardware", &ORBSlamPythonEnhanced::detectHardware)
        .staticmethod("detect_hardware")
        .def("get_hardware_config", &ORBSlamPythonEnhanced::getHardwareConfig)
        .def("set_power_mode", &ORBSlamPythonEnhanced::setPowerMode)
        .def("get_power_mode", &ORBSlamPythonEnhanced::getPowerMode)
        .def("update_parameters", &ORBSlamPythonEnhanced::updateParameters)
        .def("get_parameters", &ORBSlamPythonEnhanced::getParameters)
        
        // Metrics & monitoring
        .def("get_metrics", &ORBSlamPythonEnhanced::getMetrics)
        .def("get_tracking_quality", &ORBSlamPythonEnhanced::getTrackingQuality)
        .def("get_tracking_state", &ORBSlamPythonEnhanced::getTrackingState)
        .def("get_num_features", &ORBSlamPythonEnhanced::getNumFeatures)
        .def("get_num_matched_features", &ORBSlamPythonEnhanced::getNumMatches)
        .def("get_confidence_level", &ORBSlamPythonEnhanced::getConfidenceLevel)
        .def("reset_metrics", &ORBSlamPythonEnhanced::resetMetrics)
        
        // Map & trajectory
        .def("get_map_info", &ORBSlamPythonEnhanced::getMapInfo)
        .def("get_frame_pose", &ORBSlamPythonEnhanced::getFramePose)
        .def("get_camera_matrix", &ORBSlamPythonEnhanced::getCameraMatrix)
        .def("get_dist_coef", &ORBSlamPythonEnhanced::getDistCoeff)
        .def("get_keyframe_points", &ORBSlamPythonEnhanced::getKeyframePoints)
        .def("get_trajectory_points", &ORBSlamPythonEnhanced::getTrajectoryPoints)
        .def("get_tracked_mappoints", &ORBSlamPythonEnhanced::getTrackedMappoints)
        .def("get_current_points", &ORBSlamPythonEnhanced::getCurrentPoints)
        
        // System control
        .def("reset", &ORBSlamPythonEnhanced::reset)
        .def("shutdown", &ORBSlamPythonEnhanced::shutdown)
        .def("activate_localization_mode", &ORBSlamPythonEnhanced::activateLocalizationMode)
        .def("deactivate_localization_mode", &ORBSlamPythonEnhanced::deactivateLocalizationMode)
        .def("set_mode", &ORBSlamPythonEnhanced::setMode)
        .def("set_use_viewer", &ORBSlamPythonEnhanced::setUseViewer)
        .def("set_rgb_mode", &ORBSlamPythonEnhanced::setRGBMode)
        
        // Settings
        .def("save_settings", &ORBSlamPythonEnhanced::saveSettings)
        .def("load_settings", &ORBSlamPythonEnhanced::loadSettings)
        .def("save_settings_file", &ORBSlamPythonEnhanced::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPythonEnhanced::loadSettingsFile)
        .staticmethod("load_settings_file");
}

