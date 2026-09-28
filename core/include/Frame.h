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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

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

#include "Semantic/Marker.h"

namespace vs_graphs
{
namespace core
{
#define FRAME_GRID_ROWS 48
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

class Frame
{
  public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame_in);

    /*! Preserve the legacy member-wise assignment semantics explicitly. */
    Frame &operator=(const Frame &frame_in) = default;

    // Constructor for stereo cameras (with or without IMU) #1
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

    // Constructor for stereo cameras (with or without IMU) #2
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

    // Constructor for RGB-D cameras (with or without IMU)
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

    // Constructor for Monocular cameras (with or without IMU)
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
    void extractOrbFeatures(int            flag_in,
                            const cv::Mat &imageGray_in,
                            const int      x0_in,
                            const int      x1_in);

    // Compute Bag of Words representation.
    void computeBagOfWords();

    // Set the camera pose. (Imu pose is not modified!)
    void setPose(const Sophus::SE3<float> &Tcw_in);

    // Set IMU velocity
    void setVelocity(Eigen::Vector3f Vw_in);

    Eigen::Vector3f getVelocity() const;

    // Set IMU pose and velocity (implicitly changes camera pose)
    void setImuPoseVelocity(const Eigen::Matrix3f &Rwb_in,
                            const Eigen::Vector3f &twb_in,
                            const Eigen::Vector3f &Vwb_in);

    Eigen::Matrix<float, 3, 1> getImuPosition() const;
    Eigen::Matrix<float, 3, 3> getImuRotation();
    Sophus::SE3<float>         getImuPose();

    Sophus::SE3f    getRelativePoseTrl();
    Sophus::SE3f    getRelativePoseTlr();
    Eigen::Matrix3f getRelativePoseTlrRotation();
    Eigen::Vector3f getRelativePoseTlrTranslation();

    void setNewBias(const IMU::Bias &b_in);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint *p_mapPoint_inout, float viewingCosLimit_in);

    bool projectPointDistort(MapPoint    *p_mapPoint_in,
                             cv::Point2f &keyPoint_out,
                             float       &u_out,
                             float       &v_out);

    Eigen::Vector3f inReferenceCoordinates(Eigen::Vector3f pCw_in);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool isPositionInGrid(const cv::KeyPoint &keyPoint_in,
                          int                &positionX_out,
                          int                &positionY_out);

    vector<size_t> getFeaturesInArea(const float &x_in,
                                     const float &y_in,
                                     const float &r_in,
                                     const int    minimumLevel_in = -1,
                                     const int    maximumLevel_in = -1,
                                     const bool isRightCamera_in = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the
    // right image. If there is a match, depth is computed and the right
    // coordinate associated to the left keypoint is stored.
    void computeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in
    // the depthmap.
    void computeStereoFromRGBD(const cv::Mat &imageDepth_in);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world
    // coordinates.
    bool unprojectStereo(const int &index_in, Eigen::Vector3f &x3D_out);

    ConstraintPoseImu *p_poseImuConstraint;

    bool isImuPreintegrated();
    void setIntegrated();

    bool isSet() const;

    // Computes rotation, translation and camera center matrices from the camera
    // pose.
    void updatePoseMatrices();

    // Returns the camera center.
    inline Eigen::Vector3f getCameraCenter()
    {
        return centerOw;
    }

    // Returns inverse of rotation
    inline Eigen::Matrix3f getRotationInverse()
    {
        return rotationRwc;
    }

    inline Sophus::SE3<float> getPose() const
    {
        // TODO: can the Frame pose be accsessed from several threads? should
        // this be protected somehow?
        return poseTcw;
    }

    inline Eigen::Matrix3f getRotationRwc() const
    {
        return rotationRwc;
    }

    inline Eigen::Vector3f getCenterOw() const
    {
        return centerOw;
    }

    inline bool hasPose() const
    {
        return isPoseAvailable;
    }

    inline bool hasVelocity() const
    {
        return isVelocityAvailable;
    }

  private:
    // Sophus/Eigen migration
    Sophus::SE3<float>         poseTcw;
    Eigen::Matrix<float, 3, 3> rotationRwc;
    Eigen::Matrix<float, 3, 1> centerOw;
    Eigen::Matrix<float, 3, 3> rotationRcw;
    Eigen::Matrix<float, 3, 1> translationTcw;
    bool                       isPoseAvailable;

    // Rcw_ not necessary as Sophus has a method for extracting the rotation
    // matrix: Tcw_.rotationMatrix() tcw_ not necessary as Sophus has a method
    // for extracting the translation vector: Tcw_.translation() Twc_ not
    // necessary as Sophus has a method for easily computing the inverse pose:
    // Tcw_.inverse()

    Sophus::SE3<float>         poseTlr, poseTrl;
    Eigen::Matrix<float, 3, 3> rotationRlr;
    Eigen::Vector3f            translationTlr;

    // IMU linear velocity
    Eigen::Vector3f velocityVw;
    bool            isVelocityAvailable;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Vocabulary used for relocalization.
    ORBVocabulary *p_orbVocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor *p_orbExtractorLeft, *p_orbExtractorRight;

    // Frame timestamp.
    double timeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat         calibrationMatrix;
    Eigen::Matrix3f calibrationMatrixEigen;
    static float    fx;
    static float    fy;
    static float    cx;
    static float    cy;
    static float    invfx;
    static float    invfy;
    cv::Mat         distortionCoefficients;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb = 0.0f;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float depthThreshold;

    // Number of KeyPoints.
    int keyPointCount;

    // Vector of keypoints (original for visualization) and undistorted
    // (actually used by the system). In the stereo case, keyPointsUndistorted
    // is redundant as images must be rectified. In the RGB-D case, RGB images
    // can be distorted.
    std::vector<cv::KeyPoint> keyPoints, keyPointsRight;
    std::vector<cv::KeyPoint> keyPointsUndistorted;

    // Corresponding point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointClouds;

    // Corresponding stereo coordinate and depth for each keypoint.
    std::vector<MapPoint *> mapPoints;

    // List of Markers found in the frame
    std::vector<semantic::Marker *> mapMarkers;

    // "Monocular" keypoints have a negative value.
    std::vector<float> uRight;
    std::vector<float> depths;

    // Bag of Words Vector structures.
    DBoW2::BowVector     bowVector;
    DBoW2::FeatureVector featureVector;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat descriptors, descriptorsRight;

    // MapPoints associated to keypoints, nullptr pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> outlierFlags;
    int               closeMapPointCount = 0;

    // Keypoints are assigned to cells in a grid to reduce matching complexity
    // when projecting MapPoints.
    static float             gridElementWidthInverse;
    static float             gridElementHeightInverse;
    std::vector<std::size_t> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    IMU::Bias predictedBias;

    // IMU bias
    IMU::Bias imuBias;

    // Imu calibration
    IMU::Calib imuCalibration;

    // Imu preintegration from last keyframe
    IMU::Preintegrated *p_imuPreintegrated;
    KeyFrame           *p_lastKeyFrame;

    // Pointer to previous frame
    Frame                              *p_previousFrame;
    std::shared_ptr<IMU::Preintegrated> p_imuPreintegratedFrame;

    // Current and Next Frame id.
    static long unsigned int nextId;
    long unsigned int        id;

    // Reference Keyframe.
    KeyFrame *p_referenceKeyFrame;

    // Scale pyramid info.
    int           scaleLevelCount;
    float         scaleFactor;
    float         logScaleFactor;
    vector<float> scaleFactors;
    vector<float> invScaleFactors;
    vector<float> levelSigmaSquared;
    vector<float> invLevelSigmaSquared;

    // Undistorted Image Bounds (computed once).
    static float gridMinX;
    static float gridMaxX;
    static float gridMinY;
    static float gridMaxY;

    static bool areInitialComputationsDone;

    map<long unsigned int, cv::Point2f> projectedPoints;
    map<long unsigned int, cv::Point2f> matchedPoints;

    string fileName;

    int datasetId;

#ifdef REGISTER_TIMES
    double orbExtractionTime;
    double stereoMatchTime;
#endif

  private:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void undistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the
    // constructor).
    void computeImageBounds(const cv::Mat &imageLeft_in);

    // Assign keypoints to the grid for speed up feature matching (called in the
    // constructor).
    void assignFeaturesToGrid();

    bool isFrameSet;

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
    camera_models::geometriccamera::GeometricCamera *p_camera, *p_camera2;

    // Number of KeyPoints extracted in the left and right images
    int leftKeyPointCount = -1, rightKeyPointCount = -1;
    // Number of Non Lapping Keypoints
    int monoLeft = -1, monoRight = -1;

    // For stereo matching
    std::vector<int> leftToRightMatches, rightToLeftMatches;

    // For stereo fisheye matching
    static cv::BFMatcher bfMatcher;

    // Triangulated stereo observations using as reference the left camera.
    // These are computed during computeStereoFishEyeMatches
    std::vector<Eigen::Vector3f> stereoPoints3D;

    // Grid for the right image
    std::vector<std::size_t> gridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Stereo fisheye
    void computeStereoFishEyeMatches();

    bool isInFrustumChecks(MapPoint *p_mapPoint_inout,
                           float     viewingCosLimit_in,
                           bool      isRightCamera_in = false);

    Eigen::Vector3f unprojectStereoFishEye(const int &index_in);

    cv::Mat colorImg; // To get the color image for sending to the Semantic
                      // Segmentation
    cv::Mat imgLeft, imgRight;

    void printPointDistribution()
    {
        int left = 0, right = 0;
        int limCount =
            (leftKeyPointCount != -1) ? leftKeyPointCount : keyPointCount;
        for (int keyPointIndex = 0; keyPointIndex < keyPointCount;
             keyPointIndex++)
        {
            if (mapPoints[keyPointIndex] && !outlierFlags[keyPointIndex])
            {
                if (keyPointIndex < limCount)
                    left++;
                else
                    right++;
            }
        }
        cout << "Point distribution in Frame: left-> " << left
             << " --- right-> " << right << endl;
    }

    Sophus::SE3<double> T_test;
};

} // namespace core
} // namespace vs_graphs

#endif // FRAME_H
