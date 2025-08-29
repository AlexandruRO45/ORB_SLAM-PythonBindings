#define PY_ARRAY_UNIQUE_SYMBOL pbcvt_ARRAY_API
#include <opencv2/core/core.hpp>
#include <pyboostcvconverter/pyboostcvconverter.hpp>
#include "ORBSlamPython.h"
#include "System.h"
#include "Frame.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"

#include <Eigen/Dense>
#include <sophus/se3.hpp>


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

// Helper function to convert Sophus::SE3f to a 4x4 cv::Mat
cv::Mat se3f_to_cvmat4f(const Sophus::SE3f& pose) {
    Eigen::Matrix4f eigen_mat = pose.matrix();
    cv::Mat cv_mat(4, 4, CV_32F);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cv_mat.at<float>(i, j) = eigen_mat(i, j);
        }
    }
    return cv_mat;
}


BOOST_PYTHON_MODULE(orbslam3)
{
    init_ar();

    boost::python::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
    pbcvt::matFromNDArrayBoostConverter();

    boost::python::enum_<ORB_SLAM3::Tracking::eTrackingState>("TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM3::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM3::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM3::Tracking::eTrackingState::OK)
        .value("LOST", ORB_SLAM3::Tracking::eTrackingState::LOST);

    boost::python::enum_<ORB_SLAM3::System::eSensor>("Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD);

    boost::python::class_<ORBSlamPython, boost::noncopyable>("System", boost::python::init<const char *, const char *, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def(boost::python::init<std::string, std::string, boost::python::optional<ORB_SLAM3::System::eSensor>>())
        .def("initialize", &ORBSlamPython::initialize)
        .def("load_and_process_mono", &ORBSlamPython::loadAndProcessMono)
        .def("process_image_mono", &ORBSlamPython::processMono)
        .def("process_image_mono_inertial", &ORBSlamPython::processMono_Inertial)
        .def("load_and_process_stereo", &ORBSlamPython::loadAndProcessStereo)
        .def("process_image_stereo", &ORBSlamPython::processStereo)
        .def("process_image_stereo_inertial", &ORBSlamPython::processStereo_Inertial)
        .def("load_and_process_rgbd", &ORBSlamPython::loadAndProcessRGBD)
        .def("process_image_rgbd", &ORBSlamPython::processRGBD)
        .def("process_image_rgbd_inertial", &ORBSlamPython::processRGBD_Inertial)
        .def("shutdown", &ORBSlamPython::shutdown)
        .def("is_running", &ORBSlamPython::isRunning)
        .def("reset", &ORBSlamPython::reset)
        .def("activateSLAM", &ORBSlamPython::activateSLAMTraking)
        .def("deactivateSLAM", &ORBSlamPython::deactivateSLAMTraking)
        .def("get_current_points", &ORBSlamPython::getCurrentPoints)
        .def("get_frame_pose", &ORBSlamPython::getFramePose)
        .def("get_camera_matrix", &ORBSlamPython::getCameraMatrix)
        .def("get_dist_coef", &ORBSlamPython::getDistCoeff)
        .def("set_mode", &ORBSlamPython::setMode)
        .def("set_use_viewer", &ORBSlamPython::setUseViewer)
        .def("get_keyframe_points", &ORBSlamPython::getKeyframePoints)
        .def("get_trajectory_points", &ORBSlamPython::getTrajectoryPoints)
        .def("get_tracked_mappoints", &ORBSlamPython::getTrackedMappoints)
        .def("get_tracking_state", &ORBSlamPython::getTrackingState)
        .def("get_num_features", &ORBSlamPython::getNumFeatures)
        .def("get_num_matched_features", &ORBSlamPython::getNumMatches)
        .def("save_settings", &ORBSlamPython::saveSettings)
        .def("load_settings", &ORBSlamPython::loadSettings)
        .def("save_settings_file", &ORBSlamPython::saveSettingsFile)
        .staticmethod("save_settings_file")
        .def("load_settings_file", &ORBSlamPython::loadSettingsFile)
        .staticmethod("load_settings_file");
}

ORBSlamPython::ORBSlamPython(std::string vocabFile, std::string settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      bUseRGB(true)
{
}

ORBSlamPython::ORBSlamPython(const char *vocabFile, const char *settingsFile, ORB_SLAM3::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      bUseRGB(true)
{
}

ORBSlamPython::~ORBSlamPython()
{
}

bool ORBSlamPython::initialize()
{
    system = std::make_shared<ORB_SLAM3::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer);
    return true;
}

bool ORBSlamPython::isRunning()
{
    return system != nullptr;
}

void ORBSlamPython::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSlamPython::loadAndProcessMono(std::string imageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processMono(im, timestamp);
}

bool ORBSlamPython::processMono(cv::Mat image, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (image.data)
    {
        Sophus::SE3f pose = system->TrackMonocular(image, timestamp);
        return pose.so3().log().allFinite();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processStereo(leftImage, rightImage, timestamp);
}

bool ORBSlamPython::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (leftImage.data && rightImage.data)
    {
        Sophus::SE3f pose = system->TrackStereo(leftImage, rightImage, timestamp);
        return pose.so3().log().allFinite();
    }
    else
    {
        return false;
    }
}

bool ORBSlamPython::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    return this->processRGBD(im, imDepth, timestamp);
}

bool ORBSlamPython::processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (image.data && depthImage.data)
    {
        Sophus::SE3f pose = system->TrackRGBD(image, depthImage, timestamp);
        return pose.so3().log().allFinite();
    }
    else
    {
        return false;
    }
}

// --- IMU BLOCK ---

std::vector<ORB_SLAM3::IMU::Point> parse_imu_data(boost::python::list imu_data) {
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    for (int i = 0; i < boost::python::len(imu_data); ++i) {
        boost::python::tuple measurement = boost::python::extract<boost::python::tuple>(imu_data[i]);
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

bool ORBSlamPython::processMono_Inertial(cv::Mat image, double timestamp, boost::python::list imu_data) {
    if (!system) {
        return false;
    }
    if (image.data) {
        auto vImuMeas = parse_imu_data(imu_data);
        Sophus::SE3f pose = system->TrackMonocular(image, timestamp, vImuMeas);
        return pose.so3().log().allFinite();
    } else {
        return false;
    }
}

bool ORBSlamPython::processStereo_Inertial(cv::Mat leftImage, cv::Mat rightImage, double timestamp, boost::python::list imu_data) {
    if (!system) {
        return false;
    }
    if (leftImage.data && rightImage.data) {
        auto vImuMeas = parse_imu_data(imu_data);
        Sophus::SE3f pose = system->TrackStereo(leftImage, rightImage, timestamp, vImuMeas);
        return pose.so3().log().allFinite();
    } else {
        return false;
    }
}

bool ORBSlamPython::processRGBD_Inertial(cv::Mat image, cv::Mat depthImage, double timestamp, boost::python::list imu_data) {
    if (!system) {
        return false;
    }
    if (image.data && depthImage.data) {
        auto vImuMeas = parse_imu_data(imu_data);
        Sophus::SE3f pose = system->TrackRGBD(image, depthImage, timestamp, vImuMeas);
        return pose.so3().log().allFinite();
    } else {
        return false;
    }
}

// --- END OF IMU BLOCK ---

void ORBSlamPython::shutdown()
{
    if (system)
    {
        system->Shutdown();
        system.reset();
    }
}

void ORBSlamPython::activateSLAMTraking()
{
    if (system)
    {
        system->ActivateLocalizationMode();
    }
}

void ORBSlamPython::deactivateSLAMTraking()
{
    if (system)
    {
        system->DeactivateLocalizationMode();
    }
}

ORB_SLAM3::Tracking::eTrackingState ORBSlamPython::getTrackingState() const
{
    if (system)
    {
        return static_cast<ORB_SLAM3::Tracking::eTrackingState>(system->GetTrackingState());
    }
    return ORB_SLAM3::Tracking::eTrackingState::SYSTEM_NOT_READY;
}

unsigned int ORBSlamPython::getNumFeatures() const
{
    if (system)
    {
        return system->GetTracker()->mCurrentFrame.mvKeys.size();
    }
    return 0;
}

unsigned int ORBSlamPython::getNumMatches() const
{
    if (system)
    {
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        unsigned int matches = 0;
        unsigned int num = pTracker->mCurrentFrame.mvKeys.size();
        if (pTracker->mCurrentFrame.mvpMapPoints.size() < num)
        {
            num = pTracker->mCurrentFrame.mvpMapPoints.size();
        }
        if (pTracker->mCurrentFrame.mvbOutlier.size() < num)
        {
            num = pTracker->mCurrentFrame.mvbOutlier.size();
        }
        for (unsigned int i = 0; i < num; ++i)
        {
            ORB_SLAM3::MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if (pMP && !pTracker->mCurrentFrame.mvbOutlier[i] && pMP->Observations() > 0)
            {
                ++matches;
            }
        }
        return matches;
    }
    return 0;
}

PyObject* ORBSlamPython::getFramePose() const
{
    if (system)
    {
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        Sophus::SE3f poseSE3f = pTracker->mCurrentFrame.GetPose();
        if (poseSE3f.so3().log().allFinite())
        {
            cv::Mat pose_cv = se3f_to_cvmat4f(poseSE3f);
            return pbcvt::fromMatToNDArray(pose_cv);
        }
    }
    Py_RETURN_NONE;
}

PyObject *ORBSlamPython::getCameraMatrix() const
{
    if (system)
    {
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        cv::Mat cm = pTracker->mCurrentFrame.mK;
        return pbcvt::fromMatToNDArray(cm);
    }
    return Py_None; // Return python None correctly
}

boost::python::tuple ORBSlamPython::getDistCoeff() const
{
    if (system)
    {
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        cv::Mat dist = pTracker->mCurrentFrame.mDistCoef;
        if (!dist.empty()) {
            return boost::python::make_tuple(
                dist.at<float>(0),
                dist.at<float>(1),
                dist.at<float>(2),
                dist.at<float>(3));
        }
    }
    return boost::python::make_tuple();
}

boost::python::list ORBSlamPython::getKeyframePoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    vector<ORB_SLAM3::KeyFrame *> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);
    
    boost::python::list trajectory;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        ORB_SLAM3::KeyFrame *pKF = vpKFs[i];

        if (pKF->isBad())
            continue;

        Sophus::SE3f Tcw = pKF->GetPose();
        Sophus::SE3f Twc = Tcw.inverse();
        
        Eigen::Matrix3f R_eigen = Twc.rotationMatrix();
        Eigen::Vector3f t_eigen = Twc.translation();

        cv::Mat R_cv(3, 3, CV_32F);
        for(int r=0; r<3; r++) for(int c=0; c<3; c++) R_cv.at<float>(r,c) = R_eigen(r,c);
        
        cv::Mat t_cv(3, 1, CV_32F);
        for(int r=0; r<3; r++) t_cv.at<float>(r,0) = t_eigen(r);

        PyObject *Rarr = pbcvt::fromMatToNDArray(R_cv);
        PyObject *Tarr = pbcvt::fromMatToNDArray(t_cv);
        trajectory.append(boost::python::make_tuple(
            pKF->mTimeStamp,
            boost::python::handle<>(Rarr),
            boost::python::handle<>(Tarr)));
    }

    return trajectory;
}

boost::python::list ORBSlamPython::getTrackedMappoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    vector<ORB_SLAM3::MapPoint *> Mps = system->GetTrackedMapPoints();

    boost::python::list map_points;
    for (size_t i = 0; i < Mps.size(); i++)
    {
        if (Mps[i] && !Mps[i]->isBad())
        {
            Eigen::Vector3f wp = Mps[i]->GetWorldPos();
            map_points.append(boost::python::make_tuple(
                wp(0),
                wp(1),
                wp(2)));
        }
    }

    return map_points;
}

boost::python::list ORBSlamPython::getCurrentPoints() const
{
    if (system)
    {
        ORB_SLAM3::Tracking *pTracker = system->GetTracker();
        boost::python::list map_points;
        const auto& currentFrame = pTracker->mCurrentFrame;
        
        for (size_t i = 0; i < currentFrame.N; ++i)
        {
            ORB_SLAM3::MapPoint *pMP = currentFrame.mvpMapPoints[i];
            if (pMP && !pMP->isBad() && !currentFrame.mvbOutlier[i])
            {
                Eigen::Vector3f wp = pMP->GetWorldPos();
                const cv::KeyPoint& kp = currentFrame.mvKeysUn[i];
                map_points.append(boost::python::make_tuple(
                    boost::python::make_tuple(wp(0), wp(1), wp(2)),
                    boost::python::make_tuple(kp.pt.x, kp.pt.y)
                ));
            }
        }
        return map_points;
    }
    return boost::python::list();
}

boost::python::list ORBSlamPython::getTrajectoryPoints() const
{
    if (!system)
    {
        return boost::python::list();
    }

    boost::python::list trajectory;
    ORB_SLAM3::Tracking* pTracker = system->GetTracker();
    
    if (pTracker->mlpReferences.empty()) {
        return trajectory;
    }

    vector<ORB_SLAM3::KeyFrame*> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    if (vpKFs.empty()) {
        return trajectory;
    }

    Sophus::SE3f Two = vpKFs[0]->GetPose().inverse();

    std::list<ORB_SLAM3::KeyFrame*>::const_iterator lRit = pTracker->mlpReferences.begin();
    std::list<double>::const_iterator lT = pTracker->mlFrameTimes.begin();
    for (std::list<Sophus::SE3f>::const_iterator lit = pTracker->mlRelativeFramePoses.begin(), lend = pTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;
        if (!pKF || pKF->isBad()) {
            continue;
        }

        Sophus::SE3f Trw = pKF->GetPose() * Two;
        Sophus::SE3f Tcw = (*lit) * Trw;
        
        cv::Mat Tcw_cv = se3f_to_cvmat4f(Tcw);
        PyObject *ndarr = pbcvt::fromMatToNDArray(Tcw_cv);
        trajectory.append(boost::python::make_tuple(
            *lT,
            boost::python::handle<>(ndarr)));
    }

    return trajectory;
}

void ORBSlamPython::setMode(ORB_SLAM3::System::eSensor mode)
{
    sensorMode = mode;
}

void ORBSlamPython::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

void ORBSlamPython::setRGBMode(bool rgb)
{
    bUseRGB = rgb;
}

bool ORBSlamPython::saveSettings(boost::python::dict settings) const
{
    return ORBSlamPython::saveSettingsFile(settings, settingsFile);
}

boost::python::dict ORBSlamPython::loadSettings() const
{
    return ORBSlamPython::loadSettingsFile(settingsFile);
}

bool ORBSlamPython::saveSettingsFile(boost::python::dict settings, std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::WRITE);

    boost::python::list keys = settings.keys();
    for (int index = 0; index < boost::python::len(keys); ++index)
    {
        boost::python::extract<std::string> extractedKey(keys[index]);
        if (!extractedKey.check())
        {
            continue;
        }
        std::string key = extractedKey;

        boost::python::extract<int> intValue(settings[key]);
        if (intValue.check())
        {
            fs << key << int(intValue);
            continue;
        }

        boost::python::extract<float> floatValue(settings[key]);
        if (floatValue.check())
        {
            fs << key << float(floatValue);
            continue;
        }

        boost::python::extract<std::string> stringValue(settings[key]);
        if (stringValue.check())
        {
            fs << key << std::string(stringValue);
            continue;
        }
    }

    return true;
}

// Helpers for reading cv::FileNode objects into python objects.
boost::python::list readSequence(cv::FileNode fn, int depth = 10);
boost::python::dict readMap(cv::FileNode fn, int depth = 10);

boost::python::dict ORBSlamPython::loadSettingsFile(std::string settingsFilename)
{
    cv::FileStorage fs(settingsFilename.c_str(), cv::FileStorage::READ);
    cv::FileNode root = fs.root();
    if (root.isMap())
    {
        return readMap(root);
    }
    else if (root.isSeq())
    {
        boost::python::dict settings;
        settings["root"] = readSequence(root);
        return settings;
    }
    return boost::python::dict();
}

// ----------- HELPER DEFINITIONS -----------
boost::python::dict readMap(cv::FileNode fn, int depth)
{
    boost::python::dict map;
    if (fn.isMap())
    {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it)
        {
            cv::FileNode item = *it;
            std::string key = item.name();

            if (item.isNone())
            {
                map[key] = boost::python::object();
            }
            else if (item.isInt())
            {
                map[key] = int(item);
            }
            else if (item.isString())
            {
                map[key] = std::string(item);
            }
            else if (item.isReal())
            {
                map[key] = float(item);
            }
            else if (item.isSeq() && depth > 0)
            {
                map[key] = readSequence(item, depth - 1);
            }
            else if (item.isMap() && depth > 0)
            {
                map[key] = readMap(item, depth - 1); // Depth-limited recursive call to read inner maps
            }
        }
    }
    return map;
}

boost::python::list readSequence(cv::FileNode fn, int depth)
{
    boost::python::list sequence;
    if (fn.isSeq())
    {
        cv::FileNodeIterator it = fn.begin(), itEnd = fn.end();
        for (; it != itEnd; ++it)
        {
            cv::FileNode item = *it;

            if (item.isNone())
            {
                sequence.append(boost::python::object());
            }
            else if (item.isInt())
            {
                sequence.append(int(item));
            }
            else if (item.isString())
            {
                sequence.append(std::string(item));
            }
            else if (item.isReal())
            {
                sequence.append(float(item));
            }
            else if (item.isSeq() && depth > 0)
            {
                sequence.append(readSequence(item, depth - 1)); // Depth-limited recursive call to read nested sequences
            }
            else if (item.isMap() && depth > 0)
            {
                sequence.append(readMap(item, depth - 1));
            }
        }
    }
    return sequence;
}
