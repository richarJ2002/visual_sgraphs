/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            Tracking.h
 *
 * @brief           Declares Tracking, the front end that estimates the camera
 *                  pose of every new frame and decides when to create key
 *                  frames.
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Atlas.h"
#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "Frame.h"
#include "ImuTypes.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "TrackingStatus.h"
#include "Utils/Settings/objects/Settings.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <mutex>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
class KeyFrameDatabase;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Marker;
class Room;
} // namespace semantic
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
namespace utils
{
namespace settings
{
class Settings;
} // namespace settings
} // namespace utils

class Tracking
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System                    *p_sys_in,
             ORBVocabulary             *p_vocabulary_in,
             FrameDrawer               *p_frameDrawer_in,
             MapDrawer                 *p_mapDrawer_in,
             Atlas                     *p_atlas_in,
             KeyFrameDatabase          *p_keyFrameDatabase_in,
             const std::string         &settingPath_in,
             const int                  sensorType_in,
             utils::settings::Settings *p_settings_in,
             const std::string         &nameSeq_in = std::string());

    // Parse the config file
    [[nodiscard]] TrackingStatus parseCamParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);
    [[nodiscard]] TrackingStatus parseORBParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);
    [[nodiscard]] TrackingStatus parseIMUParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);

    // Preprocess the input and call Track(). Extract features and performs
    // stereo matching.
    [[nodiscard]] TrackingStatus
        grabImageStereo(const cv::Mat &imageRectifiedLeft_in,
                        const cv::Mat &imageRectifiedRight_in,
                        const double  &timestamp_in,
                        std::string    filename_in,
                        const std::vector<semantic::Marker *> markers_in,
                        const std::vector<semantic::Room *>   rooms_in,
                        Sophus::SE3f                         &cameraPose_out);
    [[nodiscard]] TrackingStatus grabImageRGBD(
        const cv::Mat                                &imageRgb_in,
        const cv::Mat                                &imageD_in,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pointcloud_in,
        const double                                 &timestamp_in,
        std::string                                   filename_in,
        const std::vector<semantic::Marker *>         markers_in,
        const std::vector<semantic::Room *>           rooms_in,
        Sophus::SE3f                                 &cameraPose_out);
    [[nodiscard]] TrackingStatus
        grabImageMonocular(const cv::Mat                        &image_in,
                           const double                         &timestamp_in,
                           std::string                           filename_in,
                           const std::vector<semantic::Marker *> markers_in,
                           const std::vector<semantic::Room *>   rooms_in,
                           Sophus::SE3f &cameraPose_out);

    [[nodiscard]] TrackingStatus
        grabImuData(const IMU::Point &imuMeasurement_in);

    // Setters of various classes
    [[nodiscard]] TrackingStatus setViewer(Viewer *p_viewer_in);
    [[nodiscard]] TrackingStatus setLoopClosing(LoopClosing *p_loopClosing_in);
    [[nodiscard]] TrackingStatus setLocalMapper(LocalMapping *p_localMapper_in);

    [[nodiscard]] TrackingStatus setStepByStep(bool isEnabled_in);
    [[nodiscard]] TrackingStatus getStepByStep(bool &stepByStep_out) const;

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when
    // projecting points
    [[nodiscard]] TrackingStatus
        changeCalibration(const std::string &settingPath_in);

    // Use this function if you have deactivated local mapping and you only want
    // to localize the camera.
    [[nodiscard]] TrackingStatus informOnlyTracking(const bool &flag_in);

    [[nodiscard]] TrackingStatus updateFrameIMU(const float      s_in,
                                                const IMU::Bias &b_in,
                                                KeyFrame *p_currentKeyFrame_in);
    [[nodiscard]] TrackingStatus getLastKeyFrame(KeyFrame *&p_lastKeyFrame_out)
    {
        p_lastKeyFrame_out = p_lastKeyFrame;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    [[nodiscard]] TrackingStatus
        getCamTwc(Sophus::SE3f &poseCameraToWorld_out) const;
    [[nodiscard]] TrackingStatus getImuTwb(Sophus::SE3f &poseBodyToWorld_out);
    [[nodiscard]] TrackingStatus getImuVwb(Eigen::Vector3f &imuVwb_out) const;
    [[nodiscard]] TrackingStatus
        isImuPreintegrated(bool &isImuPreintegrated_out) const;

    [[nodiscard]] TrackingStatus createMapInAtlas();
    // std::mutex mMutexTracks;

    //--
    [[nodiscard]] TrackingStatus newDataset();
    [[nodiscard]] TrackingStatus getNumberDataset(int &numberDataset_out) const;
    [[nodiscard]] TrackingStatus
        getMatchesInliers(int &matchesInliers_out) const;

    // DEBUG
    [[nodiscard]] TrackingStatus
        saveSubTrajectory(std::string textNameFileFrames_in,
                          std::string textNameFileKeyFrame_in,
                          std::string folder_in = "");
    [[nodiscard]] TrackingStatus
        saveSubTrajectory(std::string textNameFileFrames_in,
                          std::string textNameFileKeyFrame_in,
                          Map        *p_map_in);

    [[nodiscard]] TrackingStatus getImageScale(float &imageScale_out) const;

    // Semantic Entities
    /*!
     * @brief Get the points close to a given marker
     * @param p_currentMarker_in the address of the current marker
     */
    [[nodiscard]] TrackingStatus
        findPointsCloseToMarker(const semantic::Marker  *p_currentMarker_in,
                                std::vector<MapPoint *> &pointsClose_out);

    /*!
     * @brief Get the points close to a given location
     * @param points_in the set of map-points
     * @param location_in the given location
     * @param distanceThreshold_in the pre-defined threshold
     */
    [[nodiscard]] TrackingStatus
        findPointsCloseToLocation(const std::vector<MapPoint *> &points_in,
                                  const Eigen::Vector3f         &location_in,
                                  double                   distanceThreshold_in,
                                  std::vector<MapPoint *> &pointsClose_out);

#ifdef REGISTER_LOOP
    [[nodiscard]] TrackingStatus requestStop();
    [[nodiscard]] TrackingStatus isStopped(bool &isStopped_out);
    [[nodiscard]] TrackingStatus release();
    [[nodiscard]] TrackingStatus stopRequested(bool &isStopRequested_out);
#endif

  public:
    // Tracking states
    enum TrackingState
    {
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YET    = 0,
        NOT_INITIALIZED  = 1,
        OK               = 2,
        RECENTLY_LOST    = 3,
        LOST             = 4,
        OK_KLT           = 5
    };

    TrackingState state;
    TrackingState lastProcessedState;

    // Input sensor
    int sensor;

    // Current Frame
    Frame currentFrame;
    Frame lastFrame;

    cv::Mat imageGray;

    // Initialization Variables (Monocular)
    std::vector<int>         iniLastMatches;
    std::vector<int>         iniMatches;
    std::vector<cv::Point2f> previousMatchedPoints;
    std::vector<cv::Point3f> iniP3D;
    Frame                    initialFrame;
    Sophus::SE3f             poseTc0w;

    // Lists used to recover the full camera trajectory at the end of the
    // execution. Basically we store the reference keyframe for each frame and
    // its relative transformation
    std::list<Sophus::SE3f> relativeFramePoses;
    std::list<KeyFrame *>   referenceKeyFrames;
    std::list<double>       frameTimes;
    std::list<bool>         lostFlags;

    // frames with estimated pose
    int  trackedFr;
    bool isStepRequested;

    // True if local mapping is deactivated and we are performing only
    // localization
    bool isTrackingOnlyMode;

    [[nodiscard]] TrackingStatus
        reset(bool isRequestedByLocalMapping_in = false);
    [[nodiscard]] TrackingStatus
        resetActiveMap(bool isRequestedByLocalMapping_in = false);

    float  meanTrack;
    bool   shouldInitializeWithThreeKeyFrames;
    double t0;    // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool   isFastInitEnabled = false;

    [[nodiscard]] TrackingStatus
        getLocalMapPoints(std::vector<MapPoint *> &localMapPoints_out);

    bool shouldWriteStats;

    // Semantic map entities
    std::vector<vs_graphs::core::semantic::Room *> env_rooms;

#ifdef REGISTER_TIMES
    [[nodiscard]] TrackingStatus localMapStats2File();
    [[nodiscard]] TrackingStatus trackStats2File();
    [[nodiscard]] TrackingStatus printTimeStats();

    std::vector<double> stereoRectificationTimes_ms;
    std::vector<double> imageResizeTimes_ms;
    std::vector<double> orbExtractionTimes_ms;
    std::vector<double> stereoMatchTimes_ms;
    std::vector<double> imuIntegrationTimes_ms;
    std::vector<double> posePredictionTimes_ms;
    std::vector<double> localMapTrackTimes_ms;
    std::vector<double> newKeyFrameTimes_ms;
    std::vector<double> trackTotalTimes_ms;
#endif

  protected:
    // Main tracking function. It is independent of the input sensor.
    [[nodiscard]] TrackingStatus track();

    // Map initialization for stereo and RGB-D
    [[nodiscard]] TrackingStatus stereoInitialization();

    // Map initialization for monocular
    [[nodiscard]] TrackingStatus monocularInitialization();

    // void CreateNewMapPoints();
    [[nodiscard]] TrackingStatus createInitialMapMonocular();

    [[nodiscard]] TrackingStatus checkReplacedInLastFrame();
    [[nodiscard]] TrackingStatus trackReferenceKeyFrame(bool &isTracked_out);
    [[nodiscard]] TrackingStatus updateLastFrame();
    [[nodiscard]] TrackingStatus trackWithMotionModel(bool &isTracked_out);
    [[nodiscard]] TrackingStatus predictStateIMU(bool &isPredicted_out);

    [[nodiscard]] TrackingStatus relocalization(bool &isRelocalized_out);

    [[nodiscard]] TrackingStatus updateLocalMap();
    [[nodiscard]] TrackingStatus updateLocalPoints();
    [[nodiscard]] TrackingStatus updateLocalKeyFrames();

    [[nodiscard]] TrackingStatus trackLocalMap(bool &isTracked_out);
    [[nodiscard]] TrackingStatus searchLocalPoints();

    [[nodiscard]] TrackingStatus needNewKeyFrame(bool &needNewKeyFrame_out);
    [[nodiscard]] TrackingStatus createNewKeyFrame();

    // Perform preintegration from last frame
    [[nodiscard]] TrackingStatus preintegrateIMU();

    // Reset IMU biases and compute frame velocity
    [[nodiscard]] TrackingStatus resetFrameIMU();

    bool isMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *p_imuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> queueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled
    // by PreintegrateIMU)
    std::vector<IMU::Point> imuFromLastFrame;
    std::mutex              imuQueueMutex;

    // Imu calibration parameters
    IMU::Calib *p_imuCalibration;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias lastBias;

    // In case of performing only localization, this flag is true when there are
    // no matches to points in the map. Still tracking will continue if there
    // are enough matches with temporal points. In that case we are doing visual
    // odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool isVisualOdometry;

    // Other Thread Pointers
    LoopClosing  *p_loopClosing;
    LocalMapping *p_localMapper;

    // ORB
    ORBextractor *p_orbExtractorLeft;
    ORBextractor *p_orbExtractorRight{nullptr};
    ORBextractor *p_iniOrbExtractor{nullptr};

    // BoW
    ORBVocabulary    *p_orbVocabulary;
    KeyFrameDatabase *p_keyFrameDatabase;

    // Initalization (only for monocular)
    bool isReadyToInitialize;
    bool isInitSet;

    // Local Map
    KeyFrame               *p_referenceKF;
    std::vector<KeyFrame *> localKeyFrames;
    std::vector<MapPoint *> localMapPoints;

    // System
    System *p_system;

    // Drawers
    Viewer      *p_viewer;
    FrameDrawer *p_frameDrawer;
    MapDrawer   *p_mapDrawer;
    bool         isStepByStepMode;

    // Atlas
    Atlas *p_atlas;

    // Calibration matrix
    cv::Mat         calibrationMatrix;
    Eigen::Matrix3f calibrationMatrixEigen;
    cv::Mat         distortionCoefficients;
    float           mbf;
    float           imageScale;

    // IMU parameters
    float  imuFrequency;
    float  imuThresh;
    bool   shouldInsertKeyFramesWhenLost;
    double imuPeriod = 0.001;

    // New KeyFrame rules (according to fps)
    int minFrames;
    int maxFrames;

    int firstImuFrameId;
    int framesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two
    // keyframes.
    float depthThreshold;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values
    // are scaled.
    float depthMapFactor;

    // Current matches in frame
    int matchesInliers;

    // Keyframe insertion thresholds (configurable for aggressive corridor
    // tracking)
    int    minInliersForKF            = 30;
    int    minCloseInliersForKF       = 15;
    double minKeyFrameTemporalSpacing = 1.0;

    // Local map size (configurable)
    int maxKFsInLocalMap = 300;

    // Motion model search radius expansion
    float motionModelSearchRadiusMultiplier = 2.0F;
    int   motionModelMaxSearchRadius        = 30;

    // Initialization parameters
    int initializationMinPoints = 100;

    // Relocalization parameters
    int relocalizationMinInliers = 10;

    // Last Frame, KeyFrame and Relocalisation Info
    KeyFrame    *p_lastKeyFrame;
    unsigned int lastKeyFrameId;
    unsigned int lastRelocFrameId;
    double       timeStampLost;
    double       time_recently_lost;

    unsigned int firstFrameId;
    unsigned int initialFrameId;
    unsigned int lastInitFrameId;

    bool hasCreatedMap;

    // Motion Model
    bool         isVelocityAvailable{false};
    Sophus::SE3f velocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool isRgbEnabled;

    std::list<MapPoint *> temporalMapPoints;

    // int nMapChangeIndex;

    int numDataset;

    std::ofstream trackStatsFile;

    std::ofstream trackTimesFile;
    double        imuPreintegrationTime;
    double        posePredictionTime;
    double        localMapTrackTime;
    double        newKeyFrameDecisionTime;

    // Adaptive FAST threshold: track feature count to adjust threshold
    int lastFrameFeatures;
    int consecutiveLowFeatures;
    int baseInitialFastThreshold;
    int baseMinimumFastThreshold;

    camera_models::geometriccamera::GeometricCamera *p_camera, *p_camera2;

    int initId, lastId;

    Sophus::SE3f poseTlr;

    [[nodiscard]] TrackingStatus
        newParameterLoader(utils::settings::Settings *p_settings_inout);
    [[nodiscard]] TrackingStatus
        loadTrackingParameters(const std::string &settingPath_in);
    [[nodiscard]] TrackingStatus
        adjustFASTThreshold(); // Adaptive threshold based on tracking
                               // quality

#ifdef REGISTER_LOOP
    [[nodiscard]] TrackingStatus stop(bool &isStopped_out);

    bool       hasStopped;
    bool       isStopRequested;
    bool       isStopBlocked;
    std::mutex stopMutex;
#endif

  public:
    cv::Mat imageRight;
};

} // namespace core
} // namespace vs_graphs

#endif // TRACKING_H
