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
 * @file            Frame.h
 *
 * @brief           Declares Frame, one camera image with its extracted
 *                  features, camera model and estimated pose.
 */

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "FrameStatus.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Utils/Converter/objects/Converter.h"
#include "Utils/Settings/objects/Settings.h"

#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Eigen/Core"
#include "sophus/se3.hpp"

namespace vs_graphs
{
namespace core
{
/*!
 * @brief        Number of grid rows (cells along image y) that bucket key
 *               points by pixel position.
 */
#define FRAME_GRID_ROWS 48
/*!
 * @brief        Number of grid columns (cells along image x) that bucket key
 *               points by pixel position.
 */
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
namespace camera_models
{
class GeometricCamera;
}
class ORBextractor;
namespace semantic
{
class Marker;
}
class Wall;

/*!
 * @brief        One camera image or stereo pair with its key points,
 *               descriptors, camera pose and IMU state, as passed through
 *               Tracking and turned into a key frame.
 */
class Frame
{
  public:
    Frame();

    /*!
     * @brief        Copies a frame. Descriptors, calibration and distortion are
     *               deep-copied; map point, key frame and IMU pointers are
     *               shared. Pose and velocity are re-applied only when the
     *               source has them.
     *
     * @param[in]    frame_in
     *               Frame to copy.
     */
    Frame(const Frame &frame_in);

    /*! Preserve the legacy member-wise assignment semantics explicitly. */
    Frame &operator=(const Frame &frame_in) = default;

    /*!
     * @brief        Builds a frame from a rectified stereo pair. Extracts ORB
     *               features in both images on two threads and matches them to
     *               get depth. When the left image yields no key points the
     *               constructor returns early.
     *
     * @param[in]    imageColor_in
     *               Colour image, kept as colorImg for semantic segmentation.
     * @param[in]    imageLeft_in
     *               Rectified grayscale left image.
     * @param[in]    imageRight_in
     *               Rectified grayscale right image.
     * @param[in]    timeStamp_in
     *               Image time stamp, seconds.
     * @param[in]    p_extractorLeft_in
     *               ORB extractor for the left image; borrowed, not null.
     * @param[in]    p_extractorRight_in
     *               ORB extractor for the right image; borrowed, not null.
     * @param[in]    p_vocabulary_in
     *               ORB vocabulary for bag of words; borrowed.
     * @param[in]    K_in
     *               3x3 camera intrinsic matrix, pixels; copied.
     * @param[in]    distanceCoefficients_in
     *               OpenCV distortion coefficients; copied.
     * @param[in]    bf_in
     *               Stereo baseline multiplied by fx (metres times pixels).
     * @param[in]    thresholdDepth_in
     *               Depth threshold separating close from far points.
     * @param[in]    p_camera_in
     *               Camera model of the left camera; borrowed.
     * @param[in]    p_previousF_in
     *               Previous frame, borrowed, may be null; its velocity is
     *               copied when it has one.
     * @param[in]    imuCalibration_in
     *               IMU calibration, including the body-to-camera transform.
     * @param[in]    markers_in
     *               Semantic markers seen in this image; stored as mapMarkers.
     */
    Frame(const cv::Mat &imageColor_in,
          const cv::Mat &imageLeft_in,
          const cv::Mat &imageRight_in,
          const double  &timeStamp_in,
          ORBextractor  *p_extractorLeft_in,
          ORBextractor  *p_extractorRight_in,
          ORBVocabulary *p_vocabulary_in,
          cv::Mat       &K_in,
          cv::Mat       &distanceCoefficients_in,
          const float   &bf_in,
          const float   &thresholdDepth_in,
          camera_models::geometriccamera::GeometricCamera *p_camera_in,
          Frame            *p_previousF_in    = static_cast<Frame *>(nullptr),
          const IMU::Calib &imuCalibration_in = IMU::Calib(),
          const std::vector<semantic::Marker *> markers_in =
              std::vector<semantic::Marker *>{});

    /*!
     * @brief        Builds a frame from a fisheye stereo pair with a camera
     *               model per camera. Extracts ORB features in both images on
     *               two threads and triangulates matches in the overlap area.
     *               When no key points are found the constructor returns early.
     *
     * @param[in]    imageColor_in
     *               Colour image, kept as colorImg for semantic segmentation.
     * @param[in]    imageLeft_in
     *               Grayscale left image; a copy is kept as imgLeft.
     * @param[in]    imageRight_in
     *               Grayscale right image; a copy is kept as imgRight.
     * @param[in]    timeStamp_in
     *               Image time stamp, seconds.
     * @param[in]    p_extractorLeft_in
     *               ORB extractor for the left image; borrowed, not null.
     * @param[in]    p_extractorRight_in
     *               ORB extractor for the right image; borrowed, not null.
     * @param[in]    p_vocabulary_in
     *               ORB vocabulary for bag of words; borrowed.
     * @param[in]    K_in
     *               3x3 camera intrinsic matrix, pixels; copied.
     * @param[in]    distanceCoefficients_in
     *               OpenCV distortion coefficients; copied.
     * @param[in]    bf_in
     *               Stereo baseline multiplied by fx (metres times pixels).
     * @param[in]    thresholdDepth_in
     *               Depth threshold separating close from far points.
     * @param[in]    p_camera_in
     *               Camera model of the left camera; borrowed.
     * @param[in]    p_camera2_in
     *               Camera model of the right camera; borrowed.
     * @param[in]    Tlr_in
     *               Transform between the cameras, stored as poseTlr (right
     *               camera frame to left camera frame).
     * @param[in]    p_previousF_in
     *               Previous frame, borrowed, may be null; its velocity is
     *               copied when it has one.
     * @param[in]    imuCalibration_in
     *               IMU calibration, including the body-to-camera transform.
     * @param[in]    markers_in
     *               Semantic markers seen in this image; stored as mapMarkers.
     */
    Frame(const cv::Mat &imageColor_in,
          const cv::Mat &imageLeft_in,
          const cv::Mat &imageRight_in,
          const double  &timeStamp_in,
          ORBextractor  *p_extractorLeft_in,
          ORBextractor  *p_extractorRight_in,
          ORBVocabulary *p_vocabulary_in,
          cv::Mat       &K_in,
          cv::Mat       &distanceCoefficients_in,
          const float   &bf_in,
          const float   &thresholdDepth_in,
          camera_models::geometriccamera::GeometricCamera *p_camera_in,
          camera_models::geometriccamera::GeometricCamera *p_camera2_in,
          Sophus::SE3f                                    &Tlr_in,
          Frame            *p_previousF_in    = static_cast<Frame *>(nullptr),
          const IMU::Calib &imuCalibration_in = IMU::Calib(),
          const std::vector<semantic::Marker *> markers_in =
              std::vector<semantic::Marker *>{});

    /*!
     * @brief        Builds a frame from an RGB-D image. Extracts ORB features
     *               from the grayscale image and takes each key point depth
     *               from the depth image. When no key points are found the
     *               constructor returns early.
     *
     * @param[in]    imageColor_in
     *               Colour image, kept as colorImg for semantic segmentation.
     * @param[in]    imageGray_in
     *               Grayscale image used for ORB extraction.
     * @param[in]    imageDepth_in
     *               Depth image, float metres, aligned with the grayscale
     *               image.
     * @param[in]    p_pointcloud_in
     *               Point cloud of this frame; shared, stored as pointClouds.
     * @param[in]    timeStamp_in
     *               Image time stamp, seconds.
     * @param[in]    p_extractor_in
     *               ORB extractor; borrowed, not null.
     * @param[in]    p_vocabulary_in
     *               ORB vocabulary for bag of words; borrowed.
     * @param[in]    K_in
     *               3x3 camera intrinsic matrix, pixels; copied.
     * @param[in]    distanceCoefficients_in
     *               OpenCV distortion coefficients; copied.
     * @param[in]    bf_in
     *               Virtual stereo baseline multiplied by fx (metres times
     *               pixels).
     * @param[in]    thresholdDepth_in
     *               Depth threshold separating close from far points.
     * @param[in]    p_camera_in
     *               Camera model; borrowed.
     * @param[in]    p_previousF_in
     *               Previous frame, borrowed, may be null; its velocity is
     *               copied when it has one.
     * @param[in]    imuCalibration_in
     *               IMU calibration, including the body-to-camera transform.
     * @param[in]    markers_in
     *               Semantic markers seen in this image; stored as mapMarkers.
     */
    Frame(const cv::Mat                                &imageColor_in,
          const cv::Mat                                &imageGray_in,
          const cv::Mat                                &imageDepth_in,
          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pointcloud_in,
          const double                                 &timeStamp_in,
          ORBextractor                                 *p_extractor_in,
          ORBVocabulary                                *p_vocabulary_in,
          cv::Mat                                      &K_in,
          cv::Mat                                      &distanceCoefficients_in,
          const float                                  &bf_in,
          const float                                  &thresholdDepth_in,
          camera_models::geometriccamera::GeometricCamera *p_camera_in,
          Frame            *p_previousF_in    = static_cast<Frame *>(nullptr),
          const IMU::Calib &imuCalibration_in = IMU::Calib(),
          const std::vector<semantic::Marker *> markers_in =
              std::vector<semantic::Marker *>{});

    /*!
     * @brief        Builds a frame from a single camera image. Extracts ORB
     *               features from the grayscale image; no depth is available.
     *               When no key points are found the constructor returns early.
     *
     * @param[in]    imageColor_in
     *               Colour image, kept as colorImg for semantic segmentation.
     * @param[in]    imageGray_in
     *               Grayscale image used for ORB extraction.
     * @param[in]    timeStamp_in
     *               Image time stamp, seconds.
     * @param[in]    p_extractor_in
     *               ORB extractor; borrowed, not null.
     * @param[in]    p_vocabulary_in
     *               ORB vocabulary for bag of words; borrowed.
     * @param[in]    p_camera_inout
     *               Camera model that supplies the intrinsics; borrowed, stored
     *               as p_camera and not modified.
     * @param[in]    distanceCoefficients_in
     *               OpenCV distortion coefficients; copied.
     * @param[in]    bf_in
     *               Stereo baseline multiplied by fx; kept as mbf.
     * @param[in]    thresholdDepth_in
     *               Depth threshold separating close from far points.
     * @param[in]    p_previousF_in
     *               Previous frame, borrowed, may be null; its velocity is
     *               copied when it has one.
     * @param[in]    imuCalibration_in
     *               IMU calibration, including the body-to-camera transform.
     * @param[in]    markers_in
     *               Semantic markers seen in this image; stored as mapMarkers.
     */
    Frame(const cv::Mat                                   &imageColor_in,
          const cv::Mat                                   &imageGray_in,
          const double                                    &timeStamp_in,
          ORBextractor                                    *p_extractor_in,
          ORBVocabulary                                   *p_vocabulary_in,
          camera_models::geometriccamera::GeometricCamera *p_camera_inout,
          cv::Mat          &distanceCoefficients_in,
          const float      &bf_in,
          const float      &thresholdDepth_in,
          Frame            *p_previousF_in    = static_cast<Frame *>(nullptr),
          const IMU::Calib &imuCalibration_in = IMU::Calib(),
          const std::vector<semantic::Marker *> markers_in =
              std::vector<semantic::Marker *>{});

    /*!
     * @brief Extract ORB features from the given grayscale image
     *
     * @param flag_in The flag to indicate which image to extract features from
     * (0 for left, 1 for right)
     * @param imageGray_in The grayscale image to extract features from
     * @param x0_in The x-coordinate of the top-left corner of the ROI
     * @param x1_in The x-coordinate of the bottom-right corner of the ROI
     */
    [[nodiscard]] FrameStatus extractOrbFeatures(int            flag_in,
                                                 const cv::Mat &imageGray_in,
                                                 const int      x0_in,
                                                 const int      x1_in);

    /*!
     * @brief        Converts the ORB descriptors into a bag-of-words vector and
     *               feature vector. Does nothing when bowVector is already
     *               filled.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus computeBagOfWords();

    /*!
     * @brief        Sets the camera pose and refreshes the cached rotations,
     *               translations and camera centre. IMU pose getters derive
     *               from this pose.
     *
     * @param[in]    cameraPose_worldToCamera_in
     *               Camera pose mapping world-frame points into the camera
     *               frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        setPose(const Sophus::SE3<float> &cameraPose_worldToCamera_in);

    /*!
     * @brief        Stores the IMU linear velocity and marks the velocity as
     *               available.
     *
     * @param[in]    Vw_in
     *               IMU linear velocity in the world frame, metres per second.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus setVelocity(Eigen::Vector3f Vw_in);

    /*!
     * @brief        Returns the stored IMU linear velocity. It is meaningful
     *               only when hasVelocity reports true.
     *
     * @param[out]   getVelocity_out
     *               IMU linear velocity in the world frame, metres per second.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getVelocity(Eigen::Vector3f &getVelocity_out) const;

    /*!
     * @brief        Sets the IMU (body) pose and velocity. The camera pose is
     *               derived from the body pose through the IMU calibration.
     *
     * @param[in]    bodyRotation_bodyToWorld_in
     *               Body orientation, body frame to world frame.
     * @param[in]    bodyTranslation_bodyToWorld_in
     *               Body origin in the world frame, metres.
     * @param[in]    Vwb_in
     *               Body linear velocity in the world frame, metres per second.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus setImuPoseVelocity(
        const Eigen::Matrix3f &bodyRotation_bodyToWorld_in,
        const Eigen::Vector3f &bodyTranslation_bodyToWorld_in,
        const Eigen::Vector3f &Vwb_in);

    /*!
     * @brief        Returns the IMU (body) origin in the world frame.
     *
     * @param[out]   getImuPosition_out
     *               Body origin in the world frame, metres.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getImuPosition(Eigen::Matrix<float, 3, 1> &getImuPosition_out) const;
    /*!
     * @brief        Returns the IMU (body) orientation relative to the world
     *               frame.
     *
     * @param[out]   imuRotation_out
     *               Rotation from the body frame to the world frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getImuRotation(Eigen::Matrix<float, 3, 3> &imuRotation_out);
    /*!
     * @brief        Returns the IMU (body) pose, derived from the camera pose
     *               and the IMU calibration.
     *
     * @param[out]   imuPose_out
     *               Body pose mapping body-frame points into the world frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus getImuPose(Sophus::SE3<float> &imuPose_out);

    /*!
     * @brief        Returns the stereo transform from the left to the right
     *               camera. It is set only by the fisheye stereo constructor.
     *
     * @param[out]   relativePoseTrl_out
     *               Pose mapping left-camera-frame points into the right camera
     *               frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getRelativePoseTrl(Sophus::SE3f &relativePoseTrl_out);
    /*!
     * @brief        Returns the stereo transform from the right to the left
     *               camera. It is set only by the fisheye stereo constructor.
     *
     * @param[out]   relativePoseTlr_out
     *               Pose mapping right-camera-frame points into the left camera
     *               frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getRelativePoseTlr(Sophus::SE3f &relativePoseTlr_out);
    /*!
     * @brief        Returns the rotation part of the right-to-left stereo
     *               transform.
     *
     * @param[out]   relativePoseTlrRotation_out
     *               Rotation from the right camera frame to the left camera
     *               frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus getRelativePoseTlrRotation(
        Eigen::Matrix3f &relativePoseTlrRotation_out);
    /*!
     * @brief        Returns the translation part of the right-to-left stereo
     *               transform, which is the right camera origin in the left
     *               camera frame.
     *
     * @param[out]   relativePoseTlrTranslation_out
     *               Right camera origin in the left camera frame, metres.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus getRelativePoseTlrTranslation(
        Eigen::Vector3f &relativePoseTlrTranslation_out);

    /*!
     * @brief        Stores a new IMU bias and passes it to the IMU
     *               preintegration when one is attached.
     *
     * @param[in]    b_in
     *               New gyroscope and accelerometer bias.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus setNewBias(const IMU::Bias &b_in);

    /*!
     * @brief        Checks whether a map point is visible from this frame's
     *               camera (either camera for fisheye stereo) and fills the map
     *               point's tracking fields for the matcher.
     *
     * @param[in,out] p_mapPoint_inout
     *               Map point to test; borrowed, not null. Its isTrackedInView
     *               and track* fields are written.
     * @param[in]    viewingCosLimit_in
     *               Smallest allowed cosine between the viewing ray and the map
     *               point mean normal.
     * @param[out]   isInFrustum_out
     *               True when the point is in front of the camera, inside the
     *               image bounds, inside its scale-invariance distance range
     *               and within the viewing angle limit.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus isInFrustum(MapPoint *p_mapPoint_inout,
                                          float     viewingCosLimit_in,
                                          bool     &isInFrustum_out);

    /*!
     * @brief        Projects a map point into the image and applies the lens
     *               distortion model.
     *
     * @param[in]    p_mapPoint_in
     *               Map point to project; borrowed, not null.
     * @param[out]   keyPoint_out
     *               Distorted pixel position; written only when isProjected_out
     *               is true.
     * @param[out]   u_out
     *               Pixel column; distorted when the point is projected,
     *               otherwise not meaningful.
     * @param[out]   v_out
     *               Pixel row; distorted when the point is projected, otherwise
     *               not meaningful.
     * @param[out]   isProjected_out
     *               False when the point is behind the camera or outside the
     *               image bounds.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus projectPointDistort(MapPoint    *p_mapPoint_in,
                                                  cv::Point2f &keyPoint_out,
                                                  float       &u_out,
                                                  float       &v_out,
                                                  bool        &isProjected_out);

    /*!
     * @brief        Transforms a world-frame point into this frame's camera
     *               frame.
     *
     * @param[in]    pCw_in
     *               Point in the world frame, metres.
     * @param[out]   referencePoint_out
     *               The same point in the camera frame, metres.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        inReferenceCoordinates(Eigen::Vector3f  pCw_in,
                               Eigen::Vector3f &referencePoint_out);

    /*!
     * @brief        Computes the grid cell of a key point and says whether that
     *               cell is inside the grid.
     *
     * @param[in]    keyPoint_in
     *               Key point whose pixel position is looked up.
     * @param[out]   positionX_out
     *               Grid column, rounded; outside 0..FRAME_GRID_COLS-1 when the
     *               key point is off the grid.
     * @param[out]   positionY_out
     *               Grid row, rounded; outside 0..FRAME_GRID_ROWS-1 when the
     *               key point is off the grid.
     * @param[out]   isPositionInGrid_out
     *               True when the cell lies inside the grid.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus isPositionInGrid(const cv::KeyPoint &keyPoint_in,
                                               int  &positionX_out,
                                               int  &positionY_out,
                                               bool &isPositionInGrid_out);

    /*!
     * @brief        Finds the key points inside a square window around a pixel,
     *               using the grid to skip distant cells.
     *
     * @param[in]    x_in
     *               Pixel column of the window centre.
     * @param[in]    y_in
     *               Pixel row of the window centre.
     * @param[in]    r_in
     *               Half-width of the square window, pixels.
     * @param[out]   featuresInArea_out
     *               Replaced by the indices of the key points found, into
     *               keyPointsUndistorted, keyPoints (left) or keyPointsRight,
     *               whichever the frame uses.
     * @param[in]    minimumLevel_in
     *               Lowest pyramid level accepted; a value of 0 or below sets
     *               no lower limit.
     * @param[in]    maximumLevel_in
     *               Highest pyramid level accepted; a negative value sets no
     *               upper limit.
     * @param[in]    isRightCamera_in
     *               True to search the right image of a fisheye stereo frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getFeaturesInArea(const float         &x_in,
                          const float         &y_in,
                          const float         &r_in,
                          std::vector<size_t> &featuresInArea_out,
                          const int            minimumLevel_in  = -1,
                          const int            maximumLevel_in  = -1,
                          const bool           isRightCamera_in = false) const;

    /*!
     * @brief        Searches for each left key point a match in the right
     *               image. When found, the depth and the right x coordinate are
     *               stored in depths and uRight; the rest stay at -1.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus computeStereoMatches();

    /*!
     * @brief        Gives each key point a virtual right x coordinate and a
     *               depth taken from the depth image. Key points without valid
     *               depth keep -1 in uRight and depths.
     *
     * @param[in]    imageDepth_in
     *               Depth image, float metres, aligned with the grayscale
     *               image.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        computeStereoFromRGBD(const cv::Mat &imageDepth_in);

    /*!
     * @brief        Back-projects a key point with a valid depth into a 3D
     *               point in the world frame.
     *
     * @param[in]    index_in
     *               Index of the key point.
     * @param[out]   x3D_out
     *               Point in the world frame, metres; written only when
     *               isUnprojected_out is true.
     * @param[out]   isUnprojected_out
     *               False when the key point has no valid depth.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus unprojectStereo(const int       &index_in,
                                              Eigen::Vector3f &x3D_out,
                                              bool &isUnprojected_out);

    /*!
     * @brief        Prior on this frame's IMU pose, velocity and biases,
     *               created by the IMU pose optimisation. The optimiser deletes
     *               the previous frame's constraint after use; the frame never
     *               deletes it. Null when there is none.
     */
    ConstraintPoseImu *p_poseImuConstraint;

    /*!
     * @brief        Says whether IMU preintegration has been attached to this
     *               frame. Takes the frame IMU mutex.
     *
     * @param[out]   isImuPreintegrated_out
     *               True once setIntegrated has been called.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus isImuPreintegrated(bool &isImuPreintegrated_out);
    /*!
     * @brief        Marks this frame as having IMU preintegration. Takes the
     *               frame IMU mutex.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus setIntegrated();

    /*!
     * @brief        Says whether a pose has been assigned to this frame.
     *
     * @param[out]   isSet_out
     *               True once setPose or setImuPoseVelocity has been called.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus isSet(bool &isSet_out) const;

    /*!
     * @brief        Recomputes the cached rotations, translation and camera
     *               centre from the camera pose.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus updatePoseMatrices();

    /*!
     * @brief        Returns the camera centre in the world frame.
     *
     * @param[out]   getCameraCenter_out
     *               Camera origin in the world frame, metres.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getCameraCenter(Eigen::Vector3f &getCameraCenter_out) const;

    /*!
     * @brief        Returns the camera pose.
     *
     * @param[out]   getPose_out
     *               Camera pose mapping world-frame points into the camera
     *               frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus getPose(Sophus::SE3<float> &getPose_out) const;

    /*!
     * @brief        Returns the camera-to-world rotation.
     *
     * @param[out]   cameraRotation_cameraToWorld_out
     *               Rotation from the camera frame to the world frame.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        getRotationRwc(Eigen::Matrix3f &cameraRotation_cameraToWorld_out) const;

    /*!
     * @brief        Says whether an IMU velocity has been stored.
     *
     * @param[out]   hasVelocity_out
     *               True once setVelocity or setImuPoseVelocity has been
     *               called.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus hasVelocity(bool &hasVelocity_out) const;

  private:
    // Sophus/Eigen migration
    /*!
     * @brief        Camera pose mapping world-frame points into the camera
     *               frame.
     */
    Sophus::SE3<float>         poseTcw;
    /*!
     * @brief        Camera-to-world rotation, cached from poseTcw by
     *               updatePoseMatrices.
     */
    Eigen::Matrix<float, 3, 3> rotationRwc;
    /*!
     * @brief        Camera centre in the world frame, metres; cached from
     *               poseTcw.
     */
    Eigen::Matrix<float, 3, 1> centerOw;
    /*!
     * @brief        World-to-camera rotation, cached from poseTcw.
     */
    Eigen::Matrix<float, 3, 3> rotationRcw;
    /*!
     * @brief        World-to-camera translation, metres; cached from poseTcw.
     */
    Eigen::Matrix<float, 3, 1> translationTcw;
    /*!
     * @brief        True once setPose or setImuPoseVelocity has assigned a
     *               pose.
     */
    bool                       isPoseAvailable;

    // Rcw_ not necessary as Sophus has a method for extracting the rotation
    // matrix: Tcw_.rotationMatrix() tcw_ not necessary as Sophus has a method
    // for extracting the translation vector: Tcw_.translation() Twc_ not
    // necessary as Sophus has a method for easily computing the inverse pose:
    // Tcw_.inverse()

    /*!
     * @brief        Stereo transform from the right to the left camera frame;
     *               set only by the fisheye stereo constructor.
     */
    Sophus::SE3<float>         poseTlr;
    /*!
     * @brief        Stereo transform from the left to the right camera frame,
     *               the inverse of poseTlr; set only by the fisheye stereo
     *               constructor.
     */
    Sophus::SE3<float>         poseTrl;
    /*!
     * @brief        Rotation part of poseTlr, right camera frame to left camera
     *               frame.
     */
    Eigen::Matrix<float, 3, 3> rotationRlr;
    /*!
     * @brief        Translation part of poseTlr: the right camera origin in the
     *               left camera frame, metres.
     */
    Eigen::Vector3f            translationTlr;

    /*!
     * @brief        IMU linear velocity in the world frame, metres per second.
     */
    Eigen::Vector3f velocityVw;
    /*!
     * @brief        True once a velocity has been stored with setVelocity or
     *               setImuPoseVelocity.
     */
    bool            isVelocityAvailable;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief        ORB vocabulary used for the bag of words and
     *               relocalisation; borrowed from the system.
     */
    ORBVocabulary *p_orbVocabulary;

    /*!
     * @brief        ORB extractor for the left (or only) image; borrowed.
     */
    ORBextractor *p_orbExtractorLeft;
    /*!
     * @brief        ORB extractor for the right image; borrowed, null unless
     *               the frame is a stereo pair.
     */
    ORBextractor *p_orbExtractorRight;

    /*!
     * @brief        Image time stamp, seconds.
     */
    double timeStamp;

    /*!
     * @brief        3x3 camera intrinsic matrix, pixels, as an OpenCV matrix.
     */
    cv::Mat         calibrationMatrix;
    /*!
     * @brief        The same intrinsic matrix as an Eigen matrix.
     */
    Eigen::Matrix3f calibrationMatrixEigen;
    /*!
     * @brief        Focal length along image x, pixels; shared by all frames
     *               and set by the first frame.
     */
    static float    fx;
    /*!
     * @brief        Focal length along image y, pixels; shared by all frames
     *               and set by the first frame.
     */
    static float    fy;
    /*!
     * @brief        Principal point column, pixels; shared by all frames and
     *               set by the first frame.
     */
    static float    cx;
    /*!
     * @brief        Principal point row, pixels; shared by all frames and set
     *               by the first frame.
     */
    static float    cy;
    /*!
     * @brief        Inverse of fx, per pixel.
     */
    static float    invfx;
    /*!
     * @brief        Inverse of fy, per pixel.
     */
    static float    invfy;
    /*!
     * @brief        OpenCV distortion coefficients (k1, k2, p1, p2 and
     *               optionally k3).
     */
    cv::Mat         distortionCoefficients;

    /*!
     * @brief        Stereo baseline multiplied by fx, metres times pixels.
     */
    float mbf;

    /*!
     * @brief        Stereo baseline, metres (mbf divided by fx).
     */
    float mb = 0.0f;

    /*!
     * @brief        Depth threshold separating close points, inserted from one
     *               view, from far points, inserted from two views as in the
     *               monocular case.
     */
    float depthThreshold;

    /*!
     * @brief        Number of key points; for a fisheye stereo frame, left plus
     *               right.
     */
    int keyPointCount;

    /*!
     * @brief        Key points of the left (or only) image at their original
     *               pixel positions, kept for visualisation.
     */
    std::vector<cv::KeyPoint> keyPoints;
    /*!
     * @brief        Key points of the right image at their original pixel
     *               positions; filled only for stereo frames.
     */
    std::vector<cv::KeyPoint> keyPointsRight;
    /*!
     * @brief        Key points of the left (or only) image after undistortion;
     *               these are the positions the system uses. Identical to
     *               keyPoints when the images are rectified.
     */
    std::vector<cv::KeyPoint> keyPointsUndistorted;

    /*!
     * @brief        Point cloud that goes with this frame; set only by the
     *               RGB-D constructor, otherwise empty.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointClouds;

    /*!
     * @brief        Map point matched to each key point, by key point index;
     *               nullptr when there is none. The frame does not own the map
     *               points.
     */
    std::vector<MapPoint *> mapPoints;

    /*!
     * @brief        Semantic markers found in this frame; passed in at
     *               construction and not owned.
     */
    std::vector<semantic::Marker *> mapMarkers;

    /*!
     * @brief        Right-image x coordinate, pixels, matched to each left key
     *               point; negative when the key point has no stereo or depth
     *               information (monocular).
     */
    std::vector<float> uRight;
    /*!
     * @brief        Depth of each key point, metres; negative when it has none.
     */
    std::vector<float> depths;

    /*!
     * @brief        Bag-of-words vector of the descriptors; empty until
     *               computeBagOfWords runs.
     */
    DBoW2::BowVector     bowVector;
    /*!
     * @brief        Vocabulary nodes and the descriptor indices under them;
     *               empty until computeBagOfWords runs.
     */
    DBoW2::FeatureVector featureVector;

    /*!
     * @brief        ORB descriptors of the left (or only) image; each row
     *               belongs to the key point of the same index.
     */
    cv::Mat descriptors;
    /*!
     * @brief        ORB descriptors of the right image; each row belongs to the
     *               key point of the same index in keyPointsRight.
     */
    cv::Mat descriptorsRight;

    /*!
     * @brief        Flag per key point marking its map point association as an
     *               outlier.
     */
    std::vector<bool> outlierFlags;
    /*!
     * @brief        Set to 0 by the constructors and by
     *               computeStereoFishEyeMatches() and copied by the copy
     *               constructor; nothing counts into it or reads it.
     */
    int               closeMapPointCount = 0;

    /*!
     * @brief        Grid columns per pixel of image width (FRAME_GRID_COLS over
     *               the image bounds); shared by all frames.
     */
    static float             gridElementWidthInverse;
    /*!
     * @brief        Grid rows per pixel of image height (FRAME_GRID_ROWS over
     *               the image bounds); shared by all frames.
     */
    static float             gridElementHeightInverse;
    /*!
     * @brief        Key point indices bucketed by grid cell, indexed
     *               [column][row], so that projecting map points only inspects
     *               nearby key points.
     */
    std::vector<std::size_t> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /*!
     * @brief        IMU bias used while predicting this frame's state; set from
     *               imuBias by the IMU state prediction.
     */
    IMU::Bias predictedBias;

    /*!
     * @brief        IMU gyroscope and accelerometer bias of this frame.
     */
    IMU::Bias imuBias;

    /*!
     * @brief        IMU calibration, including the body-to-camera transform and
     *               noise model.
     */
    IMU::Calib imuCalibration;

    /*!
     * @brief        IMU measurements preintegrated since the last key frame;
     *               borrowed, shared with Tracking, null when there is none.
     */
    IMU::Preintegrated *p_imuPreintegrated;
    /*!
     * @brief        Last key frame at the time the IMU was preintegrated;
     *               borrowed.
     */
    KeyFrame           *p_lastKeyFrame;

    /*!
     * @brief        Previous frame in the sequence; borrowed, null for the
     *               first frame.
     */
    Frame                              *p_previousFrame;
    /*!
     * @brief        IMU measurements preintegrated since the previous frame;
     *               null until the IMU is preintegrated.
     */
    std::shared_ptr<IMU::Preintegrated> p_imuPreintegratedFrame;

    /*!
     * @brief        Id that the next constructed frame takes; shared counter
     *               for all frames.
     */
    static long unsigned int nextId;
    /*!
     * @brief        Id of this frame, taken from nextId at construction and
     *               kept by copies.
     */
    long unsigned int        id;

    /*!
     * @brief        Key frame that tracking uses as the reference for this
     *               frame; borrowed, null until Tracking assigns one.
     */
    KeyFrame *p_referenceKeyFrame;

    /*!
     * @brief        Number of levels in the ORB image pyramid, from the left
     *               extractor.
     */
    int                scaleLevelCount;
    /*!
     * @brief        Scale ratio between consecutive pyramid levels.
     */
    float              scaleFactor;
    /*!
     * @brief        Natural logarithm of scaleFactor.
     */
    float              logScaleFactor;
    /*!
     * @brief        Scale of each pyramid level relative to level 0.
     */
    std::vector<float> scaleFactors;
    /*!
     * @brief        Inverse of each entry of scaleFactors.
     */
    std::vector<float> invScaleFactors;
    /*!
     * @brief        Squared scale (variance factor) of each pyramid level.
     */
    std::vector<float> levelSigmaSquared;
    /*!
     * @brief        Inverse of each entry of levelSigmaSquared.
     */
    std::vector<float> invLevelSigmaSquared;

    /*!
     * @brief        Smallest undistorted image column, pixels; shared by all
     *               frames.
     */
    static float gridMinX;
    /*!
     * @brief        Largest undistorted image column, pixels; shared by all
     *               frames.
     */
    static float gridMaxX;
    /*!
     * @brief        Smallest undistorted image row, pixels; shared by all
     *               frames.
     */
    static float gridMinY;
    /*!
     * @brief        Largest undistorted image row, pixels; shared by all
     *               frames.
     */
    static float gridMaxY;

    /*!
     * @brief        True until the first frame with key points has computed the
     *               image bounds, grid scale and intrinsics; false afterwards.
     */
    static bool areInitialComputationsDone;

    /*!
     * @brief        Pixel position where each visible local map point projects
     *               in this frame, keyed by map point id; filled by the local
     *               point search and used for drawing.
     */
    std::map<long unsigned int, cv::Point2f> projectedPoints;
    /*!
     * @brief        Cleared by the constructors and copied by the copy
     *               constructor; nothing fills or reads it.
     */
    std::map<long unsigned int, cv::Point2f> matchedPoints;

    /*!
     * @brief        Name of the image file this frame came from, set by
     *               Tracking.
     */
    std::string fileName;

    /*!
     * @brief        Index of the dataset this frame belongs to, copied from
     *               Tracking and carried to the key frame.
     */
    int datasetId;

#ifdef REGISTER_TIMES
    double orbExtractionTime;
    double stereoMatchTime;
#endif

  private:
    /*!
     * @brief        Fills keyPointsUndistorted from keyPoints using the OpenCV
     *               distortion coefficients; a zero first coefficient copies
     *               them unchanged. Stereo images must already be rectified.
     *               Called by the constructors.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus undistortKeyPoints();

    /*!
     * @brief        Computes the static undistorted image bounds gridMinX to
     *               gridMaxY from the image corners. Called by the
     *               constructors.
     *
     * @param[in]    imageLeft_in
     *               Image whose width and height give the corners.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus computeImageBounds(const cv::Mat &imageLeft_in);

    /*!
     * @brief        Puts every key point into its grid cell (grid, and
     *               gridRight for fisheye stereo) to speed up matching. Called
     *               by the constructors.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus assignFeaturesToGrid();

    /*!
     * @brief        True once a pose has been assigned; read by isSet.
     */
    bool isFrameSet;

    /*!
     * @brief        True once setIntegrated has been called; guarded by
     *               p_imuMutex.
     */
    bool hasImuPreintegration;

    /*!
     * @brief Synchronizes access to this frame's IMU preintegration state.
     *
     * Frame copies share the preintegration state, so they also share its
     * mutex. Declaration-time initialization keeps frames safe when a
     * constructor returns early because an image contains no features.
     */
    std::shared_ptr<std::mutex> p_imuMutex = std::make_shared<std::mutex>();

  public:
    /*!
     * @brief        Camera model of the left (or only) camera; borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera;
    /*!
     * @brief        Camera model of the right camera; borrowed, null unless the
     *               frame is a fisheye stereo pair.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera2;

    /*!
     * @brief        Number of key points of the left image in a fisheye stereo
     *               frame; -1 for all other frames.
     */
    int leftKeyPointCount = -1;
    /*!
     * @brief        Number of key points of the right image in a fisheye stereo
     *               frame; -1 for all other frames.
     */
    int rightKeyPointCount = -1;
    /*!
     * @brief        Number of left key points outside the stereo overlap area;
     *               -1 for frames that did not extract with an overlap.
     */
    int monoLeft = -1;
    /*!
     * @brief        Number of right key points outside the stereo overlap area;
     *               -1 for frames that did not extract with an overlap.
     */
    int monoRight = -1;

    /*!
     * @brief        For each left key point, the index of its matched right key
     *               point; -1 when unmatched. Fisheye stereo only.
     */
    std::vector<int> leftToRightMatches;
    /*!
     * @brief        For each right key point, the index of its matched left key
     *               point; -1 when unmatched. Fisheye stereo only.
     */
    std::vector<int> rightToLeftMatches;

    /*!
     * @brief        Brute-force Hamming matcher shared by all frames for
     *               fisheye stereo matching.
     */
    static cv::BFMatcher bfMatcher;

    /*!
     * @brief        Point triangulated for each left key point from its right
     *               match, in the left camera frame, metres; filled by
     *               computeStereoFishEyeMatches.
     */
    std::vector<Eigen::Vector3f> stereoPoints3D;

    /*!
     * @brief        Right-image key point indices bucketed by grid cell,
     *               indexed [column][row]; fisheye stereo only.
     */
    std::vector<std::size_t> gridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /*!
     * @brief        Matches left and right key points in the overlap area by
     *               descriptor and triangulates them. Stores the matches,
     *               depths and 3D points.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus computeStereoFishEyeMatches();

    /*!
     * @brief        Tests one camera of a fisheye stereo frame for the
     *               visibility of a map point and, on success, fills that
     *               camera's tracking fields on the map point.
     *
     * @param[in,out] p_mapPoint_inout
     *               Map point to test; borrowed, not null. Its track* fields
     *               for the tested camera are written on success.
     * @param[in]    viewingCosLimit_in
     *               Smallest allowed cosine between the viewing ray and the map
     *               point mean normal.
     * @param[out]   isInFrustumChecks_out
     *               True when the point is in front of the camera, inside the
     *               image bounds, inside its scale-invariance distance range
     *               and within the viewing angle limit.
     * @param[in]    isRightCamera_in
     *               True to test the right camera instead of the left.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus isInFrustumChecks(MapPoint *p_mapPoint_inout,
                                                float     viewingCosLimit_in,
                                                bool     &isInFrustumChecks_out,
                                                bool isRightCamera_in = false);

    /*!
     * @brief        Moves the triangulated point of a left key point into the
     *               world frame. The key point must have a stereo match; this
     *               is not checked.
     *
     * @param[in]    index_in
     *               Index of the left key point.
     * @param[out]   stereoFishEye_out
     *               Point in the world frame, metres.
     *
     * @return       FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameStatus
        unprojectStereoFishEye(const int       &index_in,
                               Eigen::Vector3f &stereoFishEye_out);

    /*!
     * @brief        Colour image, kept for semantic segmentation.
     */
    cv::Mat colorImg;
    /*!
     * @brief        Copy of the left grayscale image; set only by the fisheye
     *               stereo constructor.
     */
    cv::Mat imgLeft;
    /*!
     * @brief        Copy of the right grayscale image; set only by the fisheye
     *               stereo constructor.
     */
    cv::Mat imgRight;

    /*!
     * @brief        Never used.
     */
    Sophus::SE3<double> T_test;
};

} // namespace core
} // namespace vs_graphs

#endif // FRAME_H
