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

/*!
 * @brief           Front end of the SLAM pipeline. It turns every incoming
 *                  image into a camera pose, tracks it against the map, and
 *                  decides when to create a key frame for the other threads to
 *                  refine.
 */
class Tracking
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Builds the tracker and loads its camera, ORB, IMU and
     *                  tracking parameters.
     *
     * @param[in]       p_sys_in
     *                  System this tracker belongs to. Borrowed, not null.
     *
     * @param[in]       p_vocabulary_in
     *                  ORB bag-of-words vocabulary. Borrowed, not null.
     *
     * @param[in]       p_frameDrawer_in
     *                  Drawer fed after each frame. Borrowed, not null.
     *
     * @param[in]       p_mapDrawer_in
     *                  Drawer told the current camera pose. Borrowed, not null.
     *
     * @param[in]       p_atlas_in
     *                  Atlas that holds the maps and the camera models.
     *                  Borrowed, not null.
     *
     * @param[in]       p_keyFrameDatabase_in
     *                  Bag-of-words key frame database used for relocalization.
     *                  Borrowed, not null.
     *
     * @param[in]       settingPath_in
     *                  Path of the YAML settings file. Always read for the
     *                  Tracking.* entries; also read for the camera, ORB and
     *                  IMU values when p_settings_in is null.
     *
     * @param[in]       sensorType_in
     *                  System sensor type value (monocular, stereo, RGB-D, or
     *                  one of those with an IMU).
     *
     * @param[in]       p_settings_in
     *                  Already parsed settings. When not null the camera, ORB
     *                  and IMU values come from it instead of the YAML file.
     *                  Borrowed, may be null.
     */
    Tracking(System                    *p_sys_in,
             ORBVocabulary             *p_vocabulary_in,
             FrameDrawer               *p_frameDrawer_in,
             MapDrawer                 *p_mapDrawer_in,
             Atlas                     *p_atlas_in,
             KeyFrameDatabase          *p_keyFrameDatabase_in,
             const std::string         &settingPath_in,
             const int                  sensorType_in,
             utils::settings::Settings *p_settings_in);

    /*!
     * @brief           Reads the camera section of the settings file: builds
     *                  the camera model (registered in the Atlas), the
     *                  calibration matrix, distortion coefficients, image
     *                  scale, stereo baseline times focal length, close/far
     *                  depth threshold, depth scale and colour order, and sets
     *                  maxFrames from the frame rate.
     *
     * @param[in]       settings_in
     *                  Opened settings file, only read.
     *
     * @param[out]      isParsed_out
     *                  True when every required entry was present and valid,
     *                  false otherwise. An unknown Camera.type is logged with
     *                  its name and gives false; no camera is built and the
     *                  rest of the section is not read.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus parseCamParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);
    /*!
     * @brief           Reads the ORB extractor section of the settings file and
     *                  creates the ORB extractors the sensor needs, remembering
     *                  the FAST thresholds as the base values for the adaptive
     *                  threshold.
     *
     * @param[in]       settings_in
     *                  Opened settings file, only read.
     *
     * @param[out]      isParsed_out
     *                  True when every required entry was present and valid,
     *                  false otherwise; nothing is created after a missing
     *                  entry.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus parseORBParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);
    /*!
     * @brief           Reads the IMU section of the settings file
     *                  (camera-to-body extrinsic, noise, walk, frequency and
     *                  thresholds) and creates the IMU calibration and the
     *                  first preintegration object.
     *
     * @param[in]       settings_in
     *                  Opened settings file, only read.
     *
     * @param[out]      isParsed_out
     *                  True when every required entry was present and valid,
     *                  false otherwise; nothing is created after a missing
     *                  entry.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus parseIMUParamFile(cv::FileStorage &settings_in,
                                                   bool &isParsed_out);

    /*!
     * @brief           Converts a rectified stereo pair to grayscale, extracts
     *                  features and stereo matches, then tracks the frame.
     *
     * @param[in]       imageRectifiedLeft_in
     *                  Rectified left image.
     *
     * @param[in]       imageRectifiedRight_in
     *                  Rectified right image, also kept in imageRight for
     *                  display.
     *
     * @param[in]       timestamp_in
     *                  Capture time of the pair, seconds.
     *
     * @param[in]       filename_in
     *                  Name of the image file, stored in the frame.
     *
     * @param[in]       markers_in
     *                  Markers detected in this image, handed to the frame.
     *
     * @param[out]      cameraPose_out
     *                  Pose the current frame holds after tracking, mapping
     *                  world-frame points into the camera frame. Read state to
     *                  know whether tracking succeeded.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        grabImageStereo(const cv::Mat &imageRectifiedLeft_in,
                        const cv::Mat &imageRectifiedRight_in,
                        const double  &timestamp_in,
                        std::string    filename_in,
                        const std::vector<semantic::Marker *> markers_in,
                        Sophus::SE3f                         &cameraPose_out);
    /*!
     * @brief           Converts an RGB-D image to grayscale and metric depth,
     *                  extracts features, then tracks the frame.
     *
     * @param[in]       imageRgb_in
     *                  Colour image; its channel order follows isRgbEnabled.
     *
     * @param[in]       imageD_in
     *                  Depth image, scaled to metres with depthMapFactor.
     *
     * @param[in]       p_pointcloud_in
     *                  Point cloud of the same view, handed to the frame.
     *
     * @param[in]       timestamp_in
     *                  Capture time of the image, seconds.
     *
     * @param[in]       filename_in
     *                  Name of the image file, stored in the frame.
     *
     * @param[in]       markers_in
     *                  Markers detected in this image, handed to the frame.
     *
     * @param[out]      cameraPose_out
     *                  Pose the current frame holds after tracking, mapping
     *                  world-frame points into the camera frame. Read state to
     *                  know whether tracking succeeded.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus grabImageRGBD(
        const cv::Mat                                &imageRgb_in,
        const cv::Mat                                &imageD_in,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pointcloud_in,
        const double                                 &timestamp_in,
        std::string                                   filename_in,
        const std::vector<semantic::Marker *>         markers_in,
        Sophus::SE3f                                 &cameraPose_out);
    /*!
     * @brief           Converts a monocular image to grayscale, extracts
     *                  features, then tracks the frame. Uses the initialization
     *                  extractor until the map is initialized.
     *
     * @param[in]       image_in
     *                  Camera image; its channel order follows isRgbEnabled.
     *
     * @param[in]       timestamp_in
     *                  Capture time of the image, seconds.
     *
     * @param[in]       filename_in
     *                  Name of the image file, stored in the frame.
     *
     * @param[in]       markers_in
     *                  Markers detected in this image, handed to the frame.
     *
     * @param[out]      cameraPose_out
     *                  Pose the current frame holds after tracking, mapping
     *                  world-frame points into the camera frame. Read state to
     *                  know whether tracking succeeded.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        grabImageMonocular(const cv::Mat                        &image_in,
                           const double                         &timestamp_in,
                           std::string                           filename_in,
                           const std::vector<semantic::Marker *> markers_in,
                           Sophus::SE3f &cameraPose_out);

    /*!
     * @brief           Queues one IMU measurement, to be preintegrated with the
     *                  next frame. Safe to call from the IMU thread.
     *
     * @param[in]       imuMeasurement_in
     *                  IMU sample (acceleration, angular velocity, time)
     *                  appended to queueImuData.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        grabImuData(const IMU::Point &imuMeasurement_in);

    // Setters of various classes
    /*!
     * @brief           Tells the tracker which viewer to stop and release while
     *                  it resets.
     *
     * @param[in]       p_viewer_in
     *                  Viewer. Borrowed, may be null.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus setViewer(Viewer *p_viewer_in);
    /*!
     * @brief           Tells the tracker which loop closing thread to reset.
     *
     * @param[in]       p_loopClosing_in
     *                  Loop closing object. Borrowed; must not be null before
     *                  tracking starts.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus setLoopClosing(LoopClosing *p_loopClosing_in);
    /*!
     * @brief           Tells the tracker which local mapping thread receives
     *                  its key frames.
     *
     * @param[in]       p_localMapper_in
     *                  Local mapping object. Borrowed; must not be null before
     *                  tracking starts.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus setLocalMapper(LocalMapping *p_localMapper_in);

    /*!
     * @brief           Turns step-by-step mode on or off; while on, track()
     *                  waits for isStepRequested before processing each frame.
     *
     * @param[in]       isEnabled_in
     *                  True to wait for a step request before every frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus setStepByStep(bool isEnabled_in);
    /*!
     * @brief           Reports whether step-by-step mode is on.
     *
     * @param[out]      stepByStep_out
     *                  True while step-by-step mode is on.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus getStepByStep(bool &stepByStep_out) const;

    /*!
     * @brief           Replaces the pinhole intrinsics, distortion coefficients
     *                  and stereo baseline times focal length with the values
     *                  of another settings file. The focal length should stay
     *                  similar, or the scale prediction fails when points are
     *                  projected. The camera model itself is not rebuilt.
     *
     * @param[in]       settingPath_in
     *                  Path of the YAML settings file with the new Camera.*
     *                  entries.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        changeCalibration(const std::string &settingPath_in);

    /*!
     * @brief           Switches localization-only mode: when true, tracking
     *                  uses the existing map but creates no key frames, which
     *                  is what you want after local mapping has been
     *                  deactivated.
     *
     * @param[in]       flag_in
     *                  True for localization-only mode, false for normal
     *                  tracking and mapping.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus informOnlyTracking(const bool &flag_in);

    /*!
     * @brief           Applies a new IMU scale and bias after inertial
     *                  optimization: stretches the stored relative translations
     *                  of the current map by the scale, and stores the bias in
     *                  the last and current frames.
     *
     * @param[in]       s_in
     *                  Scale factor multiplied into the stored relative
     *                  translations.
     *
     * @param[in]       b_in
     *                  New IMU bias, stored in lastBias and in both frames.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Key frame whose map is rescaled; becomes the last key
     *                  frame. Borrowed, not null.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus updateFrameIMU(const float      s_in,
                                                const IMU::Bias &b_in,
                                                KeyFrame *p_currentKeyFrame_in);
    /*!
     * @brief           Gives the key frame created most recently.
     *
     * @param[out]      p_lastKeyFrame_out
     *                  Last key frame, or null when none exists yet. Borrowed.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus getLastKeyFrame(KeyFrame *&p_lastKeyFrame_out)
    {
        p_lastKeyFrame_out = p_lastKeyFrame;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    /*!
     * @brief           Gives the camera pose of the current frame.
     *
     * @param[out]      cameraPose_cameraToWorld_out
     *                  Pose mapping camera-frame points into the world frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        getCamTwc(Sophus::SE3f &cameraPose_cameraToWorld_out) const;
    /*!
     * @brief           Gives the IMU (body) pose of the current frame.
     *
     * @param[out]      bodyPose_bodyToWorld_out
     *                  Pose mapping body-frame points into the world frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        getImuTwb(Sophus::SE3f &bodyPose_bodyToWorld_out);
    /*!
     * @brief           Gives the IMU velocity of the current frame.
     *
     * @param[out]      imuVwb_out
     *                  Velocity of the body in the world frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus getImuVwb(Eigen::Vector3f &imuVwb_out) const;
    /*!
     * @brief           Tells whether the current frame already has IMU
     *                  preintegration attached.
     *
     * @param[out]      isImuPreintegrated_out
     *                  True when the current frame holds a preintegration
     *                  object.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        isImuPreintegrated(bool &isImuPreintegrated_out) const;

    /*!
     * @brief           Starts a new map in the Atlas after tracking was lost or
     *                  the timestamps jumped: restarts initialization, clears
     *                  both frames and the matches, and gives IMU sensors a
     *                  fresh preintegration.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus createMapInAtlas();
    // std::mutex mMutexTracks;

    //--
    /*!
     * @brief           Counts one more dataset, so frames from now on carry the
     *                  next dataset id.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus newDataset();
    /*!
     * @brief           Gives the id of the current dataset.
     *
     * @param[out]      numberDataset_out
     *                  Number of datasets started so far (0 for the first one).
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus getNumberDataset(int &numberDataset_out) const;
    /*!
     * @brief           Gives the number of inlier matches found by the last
     *                  local map tracking.
     *
     * @param[out]      matchesInliers_out
     *                  Inlier count of the last trackLocalMap().
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        getMatchesInliers(int &matchesInliers_out) const;

    /*!
     * @brief           Gives the factor by which input images are scaled
     *                  relative to the calibration.
     *
     * @param[out]      imageScale_out
     *                  Image scale (1 means unscaled).
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus getImageScale(float &imageScale_out) const;

    // Semantic Entities
    /*!
     * @brief           Get the points close to a given marker
     *
     * @param[in]       p_currentMarker_in
     *                  the address of the current marker
     *
     * @param[out]      pointsClose_out
     *                  the map points close to the marker
     */
    [[nodiscard]] TrackingStatus
        findPointsCloseToMarker(const semantic::Marker  *p_currentMarker_in,
                                std::vector<MapPoint *> &pointsClose_out);

    /*!
     * @brief           Get the points close to a given location
     *
     * @param[in]       points_in
     *                  the set of map-points
     *
     * @param[in]       location_in
     *                  the given location
     *
     * @param[in]       distanceThreshold_in
     *                  the pre-defined threshold
     *
     * @param[out]      pointsClose_out
     *                  the map points within the threshold
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
    /*!
     * @brief           Where the tracker stands: before any image,
     *                  initializing, tracking normally, recently lost (still
     *                  trying to recover), or lost.
     */
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

    /*!
     * @brief           Current tracking state.
     */
    TrackingState state;
    /*!
     * @brief           State at the start of the latest track() call, before
     *                  that call changed it. The frame drawer reads it to know
     *                  what to show.
     */
    TrackingState lastProcessedState{NO_IMAGES_YET};

    /*!
     * @brief           Sensor configuration, as a System sensor type value
     *                  (monocular, stereo, RGB-D, or one of those with an IMU).
     */
    int sensor;

    // Current Frame
    /*!
     * @brief           Frame being tracked.
     */
    Frame currentFrame;
    /*!
     * @brief           Previous frame; a copy of currentFrame taken at the end
     *                  of track().
     */
    Frame lastFrame;

    /*!
     * @brief           Grayscale version of the (left) image being processed.
     */
    cv::Mat imageGray;

    // Initialization Variables (Monocular)
    /*!
     * @brief           Monocular initialization: for each keypoint of
     *                  initialFrame, the index of its match in the current
     *                  frame, negative when unmatched.
     */
    std::vector<int>         iniMatches;
    /*!
     * @brief           Monocular initialization: pixel position in the latest
     *                  frame of each initialFrame keypoint, used as the centre
     *                  of the next match search.
     */
    std::vector<cv::Point2f> previousMatchedPoints;
    /*!
     * @brief           Monocular initialization: triangulated 3D point of each
     *                  matched keypoint, in the camera frame of initialFrame.
     */
    std::vector<cv::Point3f> iniP3D;
    /*!
     * @brief           Monocular initialization: reference frame that later
     *                  frames are matched against.
     */
    Frame                    initialFrame;
    /*!
     * @brief           Pose given to initialFrame in the monocular case:
     *                  identity, or the rotation of the world frame from the
     *                  WorldRPY settings. Maps world-frame points into the
     *                  initial camera frame.
     */
    Sophus::SE3f             poseTc0w;

    /*!
     * @brief           For every tracked frame, its camera pose relative to its
     *                  reference key frame, mapping points from the reference
     *                  key frame camera frame into the frame camera frame. Used
     *                  to rebuild the full trajectory at the end of a run.
     */
    std::list<Sophus::SE3f> relativeFramePoses;
    /*!
     * @brief           For every tracked frame, its reference key frame, same
     *                  order as relativeFramePoses. Borrowed.
     */
    std::list<KeyFrame *>   referenceKeyFrames;
    /*!
     * @brief           For every tracked frame, its timestamp in seconds, same
     *                  order as relativeFramePoses.
     */
    std::list<double>       frameTimes;
    /*!
     * @brief           For every tracked frame, true when the tracker was LOST
     *                  on it, same order as relativeFramePoses.
     */
    std::list<bool>         lostFlags;

    /*!
     * @brief           Set by the viewer to let one frame through while
     *                  step-by-step mode waits; track() clears it.
     */
    bool isStepRequested;

    /*!
     * @brief           True when local mapping is deactivated and the tracker
     *                  only localizes the camera.
     */
    bool isTrackingOnlyMode;

    /*!
     * @brief           Resets the whole system: stops the viewer, resets local
     *                  mapping, loop closing and the key frame database, clears
     *                  the Atlas and starts a fresh map, and restarts frame and
     *                  key frame numbering. Blocks until the viewer has
     *                  stopped.
     *
     * @param[in]       isRequestedByLocalMapping_in
     *                  True when local mapping asked for the reset, so it is
     *                  not asked to reset itself again.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        reset(bool isRequestedByLocalMapping_in = false);
    /*!
     * @brief           Resets only the active map: waits for a running map
     *                  merge, stops the viewer, resets local mapping, loop
     *                  closing, the key frame database and the map, and marks
     *                  the frames of the reset maps as lost in lostFlags.
     *
     * @param[in]       isRequestedByLocalMapping_in
     *                  True when local mapping asked for the reset, so it is
     *                  not asked to reset itself again.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        resetActiveMap(bool isRequestedByLocalMapping_in = false);

    /*!
     * @brief           True when the IMU.FastInit setting is on: stereo and
     *                  RGB-D IMU initialization then skips the check that the
     *                  acceleration changed enough.
     */
    bool isFastInitEnabled = false;

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
    /*!
     * @brief           Main per-frame routine, the same for every sensor:
     *                  initializes the map, estimates the camera pose (motion
     *                  model, reference key frame or relocalization), refines
     *                  it against the local map, decides on a new key frame,
     *                  and records the frame for the trajectory. In
     *                  step-by-step mode it first waits for a step request.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus track();

    /*!
     * @brief           Initializes the map for stereo and RGB-D (with or
     *                  without IMU): turns the current frame into the first key
     *                  frame and creates map points from its depth. Does
     *                  nothing when the frame has too few keypoints or, for IMU
     *                  sensors, too little acceleration change; requests an
     *                  active map reset when too few points were created. Sets
     *                  state to OK on success.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus stereoInitialization();

    /*!
     * @brief           Initializes the map for monocular sensors: picks a
     *                  reference frame, matches later frames to it,
     *                  triangulates, and calls createInitialMapMonocular() once
     *                  enough matches exist.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus monocularInitialization();

    // void CreateNewMapPoints();
    /*!
     * @brief           Builds the first two key frames and the map points from
     *                  the triangulated matches, optimizes them, and scales the
     *                  map so the median scene depth is one (four with an IMU).
     *                  Requests an active map reset when the depth is invalid
     *                  or fewer than 50 points are tracked; otherwise sets
     *                  state to OK.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus createInitialMapMonocular();

    /*!
     * @brief           Replaces the map points of the last frame that local
     *                  mapping has since fused into other points with their
     *                  replacements.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus checkReplacedInLastFrame();
    /*!
     * @brief           Estimates the pose by matching the current frame to the
     *                  reference key frame with the bag-of-words vocabulary and
     *                  optimizing from the last frame pose.
     *
     * @param[out]      isTracked_out
     *                  False when fewer than 8 matches were found. Otherwise
     *                  true for IMU sensors, and for the others true when at
     *                  least 10 inlier matches are map points with
     *                  observations.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus trackReferenceKeyFrame(bool &isTracked_out);
    /*!
     * @brief           Updates the last frame pose from its reference key
     *                  frame. Unless local mapping is active on a stereo or
     *                  RGB-D sensor, it also creates temporary visual odometry
     *                  map points from the closest depth points, kept in
     *                  temporalMapPoints.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus updateLastFrame();
    /*!
     * @brief           Estimates the pose by moving the last frame pose with
     *                  the constant velocity (or the IMU prediction),
     *                  projecting the last frame map points, and optimizing;
     *                  the search radius widens until 20 matches are found or
     *                  motionModelMaxSearchRadius is reached.
     *
     * @param[out]      isTracked_out
     *                  False when fewer than 20 matches were found, except that
     *                  IMU sensors always get true. Otherwise true for IMU
     *                  sensors and, for the others, when at least 10 inlier
     *                  matches are map points; in localization-only mode it
     *                  follows the match count and also sets isVisualOdometry.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus trackWithMotionModel(bool &isTracked_out);
    /*!
     * @brief           Predicts the current IMU pose and velocity by
     *                  integrating the preintegrated IMU measurements from the
     *                  last key frame (when the map was updated) or the last
     *                  frame.
     *
     * @param[out]      isPredicted_out
     *                  True when a prediction was made, false when the current
     *                  frame has no previous frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus predictStateIMU(bool &isPredicted_out);

    /*!
     * @brief           Tries to recover from being lost: asks the key frame
     *                  database for candidates, ranks them by matches plus
     *                  closeness to the centroids of complete rooms, and runs
     *                  PnP RANSAC and pose optimization on each.
     *
     * @param[out]      isRelocalized_out
     *                  True when a pose supported by relocalizationMinInliers
     *                  inliers was found, false otherwise.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus relocalization(bool &isRelocalized_out);

    /*!
     * @brief           Refreshes the local map: shows the current local points
     *                  in the Atlas, then rebuilds the local key frames and the
     *                  local map points.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus updateLocalMap();
    /*!
     * @brief           Rebuilds localMapPoints from the map points of the local
     *                  key frames, skipping bad and duplicate points.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus updateLocalPoints();
    /*!
     * @brief           Rebuilds localKeyFrames: the key frames that see the
     *                  points of the current (or last) frame, their neighbours
     *                  up to maxKFsInLocalMap, and the one sharing the most
     *                  points becomes the reference key frame.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus updateLocalKeyFrames();

    /*!
     * @brief           Refines the pose against the local map: updates the
     *                  local map, searches its points in the frame, optimizes
     *                  the pose, and counts the inliers into matchesInliers.
     *
     * @param[out]      isTracked_out
     *                  True when enough inliers remain; the required count
     *                  depends on the sensor, IMU state and how recently the
     *                  tracker relocalized.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus trackLocalMap(bool &isTracked_out);
    /*!
     * @brief           Projects the local map points into the current frame and
     *                  matches those not yet matched.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus searchLocalPoints();

    /*!
     * @brief           Decides whether the current frame should become a key
     *                  frame, from the inlier count, the frames and seconds
     *                  since the last key frame, local mapping load, and
     *                  tracking quality. Never in localization-only mode.
     *
     * @param[out]      needNewKeyFrame_out
     *                  True when a key frame should be created.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus needNewKeyFrame(bool &needNewKeyFrame_out);
    /*!
     * @brief           Creates a key frame from the current frame, makes it the
     *                  reference and last key frame, gives it to local mapping,
     *                  and for stereo and RGB-D sensors also creates map points
     *                  from the closest depth points.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus createNewKeyFrame();

    /*!
     * @brief           Takes the queued IMU measurements between the previous
     *                  and the current frame into imuFromLastFrame and
     *                  preintegrates them into the current frame; waits while
     *                  measurements have not arrived yet.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus preintegrateIMU();

    /*!
     * @brief           True when the current map changed since the previous
     *                  frame, set by track().
     */
    bool isMapUpdated;

    /*!
     * @brief           IMU measurements preintegrated since the last key frame;
     *                  replaced by a fresh object at each key frame. Set up for
     *                  IMU sensors only; nullptr otherwise.
     */
    IMU::Preintegrated *p_imuPreintegratedFromLastKF{nullptr};

    /*!
     * @brief           IMU measurements waiting to be used; filled by
     *                  grabImuData(), emptied by preintegrateIMU(). Guarded by
     *                  imuQueueMutex.
     */
    std::list<IMU::Point> queueImuData;

    /*!
     * @brief           IMU measurements between the previous and the current
     *                  frame, filled by preintegrateIMU().
     */
    std::vector<IMU::Point> imuFromLastFrame;
    /*!
     * @brief           Guards queueImuData between the IMU thread and the
     *                  tracking thread.
     */
    std::mutex              imuQueueMutex;

    /*!
     * @brief           IMU calibration read from the settings: camera-to-body
     *                  extrinsic and noise. Allocated by the settings parser
     *                  for IMU sensors, never deleted.
     */
    IMU::Calib *p_imuCalibration;

    /*!
     * @brief           IMU bias estimated at the last key frame.
     */
    IMU::Bias lastBias;

    /*!
     * @brief           In localization-only mode, true when the last frame
     *                  matched few map points; tracking continues with
     *                  temporary points (visual odometry) while relocalization
     *                  tries to recover drift-free localization to the map.
     */
    bool isVisualOdometry;

    // Other Thread Pointers
    /*!
     * @brief           Loop closing thread, reset together with the tracker.
     *                  Borrowed, nullptr until setLoopClosing().
     */
    LoopClosing  *p_loopClosing{nullptr};
    /*!
     * @brief           Local mapping thread that receives key frames. Borrowed,
     *                  nullptr until setLocalMapper().
     */
    LocalMapping *p_localMapper{nullptr};

    // ORB
    /*!
     * @brief           ORB extractor for the left (or only) image. Allocated by
     *                  the settings parser, never deleted.
     */
    ORBextractor *p_orbExtractorLeft;
    /*!
     * @brief           ORB extractor for the right image; stereo sensors only,
     *                  null otherwise. Allocated by the settings parser, never
     *                  deleted.
     */
    ORBextractor *p_orbExtractorRight{nullptr};
    /*!
     * @brief           ORB extractor with five times the features, used before
     *                  a monocular map is initialized; null for other sensors.
     *                  Allocated by the settings parser, never deleted.
     */
    ORBextractor *p_iniOrbExtractor{nullptr};

    // BoW
    /*!
     * @brief           Bag-of-words vocabulary. Borrowed.
     */
    ORBVocabulary    *p_orbVocabulary;
    /*!
     * @brief           Key frame database searched by relocalization. Borrowed.
     */
    KeyFrameDatabase *p_keyFrameDatabase;

    // Initalization (only for monocular)
    /*!
     * @brief           Monocular initialization: true once a reference frame
     *                  was chosen and later frames are matched against it.
     */
    bool isReadyToInitialize;

    // Local Map
    /*!
     * @brief           Reference key frame of the current frame: the local key
     *                  frame sharing the most map points with it. Borrowed,
     *                  nullptr before the first key frame.
     */
    KeyFrame               *p_referenceKF{nullptr};
    /*!
     * @brief           Key frames of the local map. Borrowed.
     */
    std::vector<KeyFrame *> localKeyFrames;
    /*!
     * @brief           Map points of the local key frames. Borrowed.
     */
    std::vector<MapPoint *> localMapPoints;

    // System
    /*!
     * @brief           System that owns this tracker. Borrowed.
     */
    System *p_system;

    // Drawers
    /*!
     * @brief           Viewer, null until setViewer() is called. Borrowed.
     */
    Viewer      *p_viewer;
    /*!
     * @brief           Drawer updated after every frame. Borrowed.
     */
    FrameDrawer *p_frameDrawer;
    /*!
     * @brief           Drawer told the current camera pose. Borrowed.
     */
    MapDrawer   *p_mapDrawer;
    /*!
     * @brief           True while track() waits for isStepRequested before each
     *                  frame.
     */
    bool         isStepByStepMode;

    // Atlas
    /*!
     * @brief           Atlas holding the maps and the camera models. Borrowed.
     */
    Atlas *p_atlas;

    // Calibration matrix
    /*!
     * @brief           Camera intrinsic matrix (3x3, fx, fy, cx, cy in pixels),
     *                  scaled by imageScale.
     */
    cv::Mat         calibrationMatrix;
    /*!
     * @brief           Same intrinsic matrix as calibrationMatrix, as an Eigen
     *                  matrix.
     */
    Eigen::Matrix3f calibrationMatrixEigen;
    /*!
     * @brief           OpenCV distortion coefficients (k1, k2, p1, p2,
     *                  optionally k3) of the left camera.
     */
    cv::Mat         distortionCoefficients;
    /*!
     * @brief           Stereo baseline times focal length, in metres times
     *                  pixels.
     */
    float           mbf{0.0F};
    /*!
     * @brief           Factor between the input images and the calibration (1
     *                  means unscaled).
     */
    float           imageScale{1.0F};

    // IMU parameters
    /*!
     * @brief           IMU sample rate, hertz.
     */
    float  imuFrequency{0.0F};
    /*!
     * @brief           Smallest change of the average acceleration between two
     *                  frames, in the accelerometer unit, that lets stereo and
     *                  RGB-D IMU initialization proceed.
     */
    float  imuThresh{0.0F};
    /*!
     * @brief           True when key frames may still be created while
     *                  RECENTLY_LOST with an IMU (InsertKFsWhenLost setting).
     */
    bool   shouldInsertKeyFramesWhenLost{false};
    /*!
     * @brief           Time between IMU samples, seconds; used to decide which
     *                  queued samples belong to a frame.
     */
    double imuPeriod = 0.001;

    // New KeyFrame rules (according to fps)
    /*!
     * @brief           Fewest frames that must pass after a key frame before
     *                  another one may be created; always 0.
     */
    int minFrames{0};
    /*!
     * @brief           Frames after which a new key frame is forced; set to the
     *                  camera frame rate.
     */
    int maxFrames{0};

    /*!
     * @brief           Frames after a relocalization during which IMU tracking
     *                  is treated as unreliable; set to maxFrames for IMU
     *                  sensors, 0 otherwise.
     */
    int framesToResetIMU{0};

    /*!
     * @brief           Depth in metres that separates close points from far
     *                  points. Close points are reliable and are inserted from
     *                  one frame; far points need a match in two key frames.
     */
    float depthThreshold{0.0F};

    /*!
     * @brief           RGB-D only: factor that converts the raw depth values to
     *                  metres (the inverse of the DepthMapFactor setting, 1
     *                  when that is zero).
     */
    float depthMapFactor{1.0F};

    /*!
     * @brief           Inlier matches of the current frame after the last local
     *                  map tracking.
     */
    int matchesInliers{0};

    // Keyframe insertion thresholds (configurable for aggressive corridor
    // tracking)
    /*!
     * @brief           Fewest inlier matches a frame needs before it may become
     *                  a key frame (Tracking.MinInliersForKF).
     */
    int    minInliersForKF = 30;
    /*!
     * @brief           Fewest close inlier matches a frame needs before it may
     *                  become a key frame; never above minInliersForKF
     *                  (Tracking.MinCloseInliersForKF).
     */
    int    minCloseInliersForKF = 15;
    /*!
     * @brief           Shortest time between two key frames, seconds
     *                  (Tracking.MinTemporalSpacingKF).
     */
    double minKeyFrameTemporalSpacing = 1.0;

    // Local map size (configurable)
    /*!
     * @brief           Most key frames kept in the local map
     *                  (Tracking.MaxKFsInLocalMap).
     */
    int maxKFsInLocalMap = 300;

    // Motion model search radius expansion
    /*!
     * @brief           Factor by which the motion model search window grows
     *                  each time it finds too few matches
     *                  (Tracking.MotionModelSearchRadiusMultiplier).
     */
    float motionModelSearchRadiusMultiplier = 2.0F;
    /*!
     * @brief           Largest motion model search radius, in the matcher
     *                  radius unit (Tracking.MotionModelMaxSearchRadius).
     */
    int   motionModelMaxSearchRadius = 30;

    // Initialization parameters
    /*!
     * @brief           Fewest keypoints or points needed to initialize the map
     *                  (Tracking.InitializationMinPoints).
     */
    int initializationMinPoints = 100;

    // Relocalization parameters
    /*!
     * @brief           Fewest inliers that make a relocalization pose
     *                  acceptable (Tracking.RelocalizationMinInliers).
     */
    int relocalizationMinInliers = 10;

    // Last Frame, KeyFrame and Relocalisation Info
    /*!
     * @brief           Key frame created most recently; null before the first
     *                  one. Borrowed.
     */
    KeyFrame    *p_lastKeyFrame;
    /*!
     * @brief           Id of the frame that became the last key frame.
     */
    unsigned int lastKeyFrameId{0};
    /*!
     * @brief           Id of the frame at which the tracker last relocalized or
     *                  restarted; 0 at the start.
     */
    unsigned int lastRelocFrameId;
    /*!
     * @brief           Timestamp of the frame at which the state became
     *                  RECENTLY_LOST, seconds.
     */
    double       timeStampLost{0.0};
    /*!
     * @brief           Longest time in RECENTLY_LOST before an IMU tracker
     *                  gives up and becomes LOST, seconds.
     */
    double       time_recently_lost;

    /*!
     * @brief           Id of the first frame processed once the Atlas holds a
     *                  single map.
     */
    unsigned int firstFrameId;
    /*!
     * @brief           Id of the first frame of the current map.
     */
    unsigned int initialFrameId;
    /*!
     * @brief           Id of the last frame before the latest map was started.
     */
    unsigned int lastInitFrameId{0};

    /*!
     * @brief           True from createMapInAtlas() until the next track()
     *                  call, so that call skips IMU preintegration.
     */
    bool hasCreatedMap;

    // Motion Model
    /*!
     * @brief           True when velocity holds a valid motion between the last
     *                  two frames.
     */
    bool         isVelocityAvailable{false};
    /*!
     * @brief           Camera motion between the last two frames: maps points
     *                  in the previous camera frame into the current camera
     *                  frame. Valid only when isVelocityAvailable is true.
     */
    Sophus::SE3f velocity;

    /*!
     * @brief           True when colour images are in RGB order, false for BGR;
     *                  ignored for grayscale.
     */
    bool isRgbEnabled{false};

    /*!
     * @brief           Temporary visual odometry map points; owned by the
     *                  tracker and deleted at the end of each frame.
     */
    std::list<MapPoint *> temporalMapPoints;

    // int nMapChangeIndex;

    /*!
     * @brief           Id of the current dataset, stored in each frame; counted
     *                  up by newDataset().
     */
    int numDataset;

    // Adaptive FAST threshold: track feature count to adjust threshold
    /*!
     * @brief           Keypoint count of the previous frame, for the adaptive
     *                  FAST threshold.
     */
    int lastFrameFeatures{0};
    /*!
     * @brief           Frames in a row with few keypoints; drives how far
     *                  adjustFASTThreshold() lowers the thresholds.
     */
    int consecutiveLowFeatures{0};
    /*!
     * @brief           Initial FAST threshold from the settings, the value the
     *                  adaptive threshold starts from and returns to.
     */
    int baseInitialFastThreshold{0};
    /*!
     * @brief           Minimum FAST threshold from the settings, the value the
     *                  adaptive threshold starts from and returns to.
     */
    int baseMinimumFastThreshold{0};

    /*!
     * @brief           Camera model of the left (or only) camera. Owned by the
     *                  Atlas; borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera{nullptr};
    /*!
     * @brief           Camera model of the second camera, only for fisheye
     *                  stereo; null otherwise. Owned by the Atlas; borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera2;

    /*!
     * @brief           Monocular: id of the key frame created when the map was
     *                  initialized.
     */
    int initId;
    /*!
     * @brief           Monocular: id of the latest frame handed to
     *                  grabImageMonocular().
     */
    int lastId;

    /*!
     * @brief           Fisheye stereo: pose of the right camera relative to the
     *                  left one, mapping left-camera points into the right
     *                  camera frame.
     */
    Sophus::SE3f poseTlr;

    /*!
     * @brief           Loads the camera, ORB and IMU values from an already
     *                  parsed settings object: camera models, intrinsics,
     *                  stereo baseline, depth threshold and scale, frame rate,
     *                  colour order, ORB extractors and IMU calibration.
     *
     * @param[in,out]   p_settings_inout
     *                  Parsed settings, queried for every value. Borrowed, not
     *                  null.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        newParameterLoader(utils::settings::Settings *p_settings_inout);
    /*!
     * @brief           Reads the optional Tracking.* entries of the settings
     *                  file. An entry that is missing keeps its default; one
     *                  with the wrong type or outside its allowed range is
     *                  reported and also keeps its default. Prints the
     *                  effective values.
     *
     * @param[in]       settingPath_in
     *                  Path of the YAML settings file.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus
        loadTrackingParameters(const std::string &settingPath_in);
    /*!
     * @brief           Lowers the FAST thresholds of all ORB extractors, by up
     *                  to 18 and never below 1, while the frames have few
     *                  keypoints (under half of the previous frame or under
     *                  400), and restores them when a frame has more than 2500.
     *
     * @return          TRACKING_STATUS_SUCCESS always.
     */
    [[nodiscard]] TrackingStatus adjustFASTThreshold();

#ifdef REGISTER_LOOP
    [[nodiscard]] TrackingStatus stop(bool &isStopped_out);

    bool       hasStopped;
    bool       isStopRequested;
    bool       isStopBlocked;
    std::mutex stopMutex;
#endif

  public:
    /*!
     * @brief           Right image of the latest stereo pair, kept for the
     *                  frame drawer; only set by grabImageStereo().
     */
    cv::Mat imageRight;
};

} // namespace core
} // namespace vs_graphs

#endif // TRACKING_H
