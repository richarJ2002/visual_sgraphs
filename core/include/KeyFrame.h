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
 * @file            KeyFrame.h
 *
 * @brief           Declares KeyFrame, a frame kept in the map: its pose,
 *                  features, observed map points and links to other key frames.
 */

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "ImuTypes.h"
#include "KeyFrameStatus.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"

#include <boost/serialization/access.hpp>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vs_graphs
{
namespace core
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
namespace camera_models
{
class GeometricCamera;
}

namespace semantic
{
class Marker;
}
namespace geometric
{
class Plane;
}
namespace semantic
{
class Passage;
}

class KeyFrame
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame            &F_inout,
             Map              *p_map_in,
             KeyFrameDatabase *p_keyFrameDatabase_in);

    // Pose functions
    [[nodiscard]] KeyFrameStatus
        setPose(const Sophus::SE3f &pose_worldToCamera_in);
    [[nodiscard]] KeyFrameStatus setVelocity(const Eigen::Vector3f &Vw_in);

    [[nodiscard]] KeyFrameStatus getPose(Sophus::SE3f &pose_out);

    [[nodiscard]] KeyFrameStatus getPoseInverse(Sophus::SE3f &poseInverse_out);
    [[nodiscard]] KeyFrameStatus
        getCameraCenter(Eigen::Vector3f &cameraCenter_out);

    [[nodiscard]] KeyFrameStatus
        getImuPosition(Eigen::Vector3f &imuPosition_out);
    [[nodiscard]] KeyFrameStatus
        getImuRotation(Eigen::Matrix3f &imuRotation_out);
    [[nodiscard]] KeyFrameStatus getImuPose(Sophus::SE3f &imuPose_out);
    [[nodiscard]] KeyFrameStatus getRotation(Eigen::Matrix3f &rotation_out);
    [[nodiscard]] KeyFrameStatus
        getTranslation(Eigen::Vector3f &translation_out);
    [[nodiscard]] KeyFrameStatus getVelocity(Eigen::Vector3f &velocity_out);
    [[nodiscard]] KeyFrameStatus isVelocitySet(bool &isVelocitySet_out);

    // Bag of Words Representation
    [[nodiscard]] KeyFrameStatus computeBagOfWords();

    // Covisibility graph functions
    [[nodiscard]] KeyFrameStatus addConnection(KeyFrame  *p_keyFrame_inout,
                                               const int &weight_in);
    [[nodiscard]] KeyFrameStatus eraseConnection(KeyFrame *p_keyFrame_in);

    [[nodiscard]] KeyFrameStatus updateConnections(bool upParent_in = true);
    [[nodiscard]] KeyFrameStatus updateBestCovisibles();
    [[nodiscard]] KeyFrameStatus
        getConnectedKeyFrames(std::set<KeyFrame *> &connectedKeyFrames_out);
    [[nodiscard]] KeyFrameStatus getVectorCovisibleKeyFrames(
        std::vector<KeyFrame *> &vectorCovisibleKeyFrames_out);
    [[nodiscard]] KeyFrameStatus getBestCovisibilityKeyFrames(
        const int               &N_in,
        std::vector<KeyFrame *> &bestCovisibilityKeyFrames_out);
    [[nodiscard]] KeyFrameStatus
                                 getCovisiblesByWeight(const int               &w_in,
                                                       std::vector<KeyFrame *> &covisiblesByWeight_out);
    [[nodiscard]] KeyFrameStatus getWeight(KeyFrame *p_keyFrame_in,
                                           int      &weight_out);

    // Spanning tree functions
    [[nodiscard]] KeyFrameStatus addChild(KeyFrame *p_keyFrame_in);
    [[nodiscard]] KeyFrameStatus eraseChild(KeyFrame *p_keyFrame_in);
    [[nodiscard]] KeyFrameStatus changeParent(KeyFrame *p_keyFrame_inout);
    [[nodiscard]] KeyFrameStatus getChilds(std::set<KeyFrame *> &childs_out);
    [[nodiscard]] KeyFrameStatus getParent(KeyFrame *&p_parent_out);
    [[nodiscard]] KeyFrameStatus hasChild(KeyFrame *p_keyFrame_in,
                                          bool     &hasChild_out);
    [[nodiscard]] KeyFrameStatus setFirstConnection(bool isFirst_in);

    // Loop Edges
    [[nodiscard]] KeyFrameStatus addLoopEdge(KeyFrame *p_keyFrame_in);
    [[nodiscard]] KeyFrameStatus
        getLoopEdges(std::set<KeyFrame *> &loopEdges_out);

    // Merge Edges
    [[nodiscard]] KeyFrameStatus addMergeEdge(KeyFrame *p_keyFrame_in);
    [[nodiscard]] KeyFrameStatus
        getMergeEdges(std::set<KeyFrame *> &mergeEdges_out);

    // MapPoint observation functions
    [[nodiscard]] KeyFrameStatus getMapPointCount(int &mapPointCount_out);
    [[nodiscard]] KeyFrameStatus addMapPoint(MapPoint     *p_mapPoint_in,
                                             const size_t &index_in);
    [[nodiscard]] KeyFrameStatus eraseMapPointMatch(const int &index_in);
    [[nodiscard]] KeyFrameStatus eraseMapPointMatch(MapPoint *p_mapPoint_in);
    [[nodiscard]] KeyFrameStatus replaceMapPointMatch(const int &index_in,
                                                      MapPoint  *p_mapPoint_in);
    [[nodiscard]] KeyFrameStatus
        getMapPoints(std::set<MapPoint *> &mapPoints_out);
    [[nodiscard]] KeyFrameStatus
        getMapPointMatches(std::vector<MapPoint *> &mapPointMatches_out);
    [[nodiscard]] KeyFrameStatus
        getTrackedMapPointCount(const int &minimumObservation_in,
                                int       &trackedMapPointCount_out);
    [[nodiscard]] KeyFrameStatus getMapPoint(const size_t &index_in,
                                             MapPoint    *&p_mapPoint_out);

    // MapMarker observation functions
    [[nodiscard]] KeyFrameStatus addMapMarker(semantic::Marker *p_marker_in);
    [[nodiscard]] KeyFrameStatus
        getMapMarkers(std::vector<semantic::Marker *> &mapMarkers_out);

    // MapPlane observation functions
    [[nodiscard]] KeyFrameStatus addMapPlane(geometric::Plane *p_plane_in);
    [[nodiscard]] KeyFrameStatus getMapPlanes(
        std::vector<geometric::Plane *>
            &mapPlanes_out); // After getting planes, need to check for NULLs
    [[nodiscard]] KeyFrameStatus removeMapPlane(geometric::Plane *p_plane_in);

    /*!
     * @brief       Replaces a retired mapped plane in this keyframe.
     *
     *              Duplicate retained-plane entries are removed while the
     *              feature mutex is held.
     *
     * @param[in]   p_retiredPlane_in
     *              Plane hypothesis which is being retired.
     *
     * @param[in]   p_retainedPlane_in
     *              Plane hypothesis which owns the fused observations.
     *
     * @param[out] wasReplaced_out True when the retired plane was present.
     * @return KEY_FRAME_STATUS_SUCCESS, or KEY_FRAME_STATUS_INVALID_ARGUMENT
     * when an input is rejected.
     */
    [[nodiscard]] KeyFrameStatus
        replaceMapPlane(geometric::Plane *p_retiredPlane_in,
                        geometric::Plane *p_retainedPlane_in,
                        bool             &wasReplaced_out);

    /*!
     * @brief       Adds a mapped passage association.
     *
     * @param[in]   p_passage_in Passage observed by this keyframe.
     */
    [[nodiscard]] KeyFrameStatus
        addMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);

    /*!
     * @brief       Replaces a retired passage association after fusion.
     *
     * @param[in]   p_retiredPassage_in Duplicate passage being retired.
     * @param[in]   p_retainedPassage_in Passage retaining the associations.
     *
     * @param[out] wasReplaced_out True when the retired passage was present.
     * @return KEY_FRAME_STATUS_SUCCESS, or KEY_FRAME_STATUS_INVALID_ARGUMENT
     * when an input is rejected.
     */
    [[nodiscard]] KeyFrameStatus replaceMapPassage(
        vs_graphs::core::semantic::Passage *p_retiredPassage_in,
        vs_graphs::core::semantic::Passage *p_retainedPassage_in,
        bool                               &wasReplaced_out);

    [[nodiscard]] KeyFrameStatus getMapPassages(
        std::vector<vs_graphs::core::semantic::Passage *> &mapPassages_out);

    // KeyPoint functions
    [[nodiscard]] KeyFrameStatus
                                 getFeaturesInArea(const float         &x_in,
                                                   const float         &y_in,
                                                   const float         &r_in,
                                                   std::vector<size_t> &featuresInArea_out,
                                                   const bool           isRightCamera_in = false) const;
    [[nodiscard]] KeyFrameStatus unprojectStereo(int              index_in,
                                                 Eigen::Vector3f &x3D_out,
                                                 bool &isUnprojected_out);

    // Image
    [[nodiscard]] KeyFrameStatus isInImage(const float &x_in,
                                           const float &y_in,
                                           bool        &isInImage_out) const;

    // Enable/Disable bad flag changes
    [[nodiscard]] KeyFrameStatus setNotErase();
    [[nodiscard]] KeyFrameStatus setErase();

    // Set/check bad flag
    [[nodiscard]] KeyFrameStatus setBadFlag();
    [[nodiscard]] KeyFrameStatus isBad(bool &isBad_out);

    // Compute Scene Depth (q=2 median). Used in monocular.
    [[nodiscard]] KeyFrameStatus
        computeSceneMedianDepth(const int q_in, float &sceneMedianDepth_out);

    static bool weightComp(int a_in, int b_in)
    {
        return a_in > b_in;
    }

    static bool lId(KeyFrame *p_keyFrame1_inout, KeyFrame *p_keyFrame2_inout)
    {
        return p_keyFrame1_inout->id < p_keyFrame2_inout->id;
    }

    [[nodiscard]] KeyFrameStatus getMap(Map *&p_map_out);
    [[nodiscard]] KeyFrameStatus updateMap(Map *p_map_in);

    [[nodiscard]] KeyFrameStatus setNewBias(const IMU::Bias &b_in);
    [[nodiscard]] KeyFrameStatus getGyroBias(Eigen::Vector3f &gyroBias_out);

    [[nodiscard]] KeyFrameStatus getAccBias(Eigen::Vector3f &accBias_out);

    [[nodiscard]] KeyFrameStatus getImuBias(IMU::Bias &imuBias_out);

    [[nodiscard]] KeyFrameStatus projectPointDistort(MapPoint    *p_mapPoint_in,
                                                     cv::Point2f &keyPoint_out,
                                                     float       &u_out,
                                                     float       &v_out,
                                                     bool &isProjected_out);
    [[nodiscard]] KeyFrameStatus
        projectPointUnDistort(MapPoint    *p_mapPoint_in,
                              cv::Point2f &keyPoint_out,
                              float       &u_out,
                              float       &v_out,
                              bool        &isProjected_out);

    [[nodiscard]] KeyFrameStatus
        preSave(std::set<KeyFrame *> &keyFrames_in,
                std::set<MapPoint *> &mapPoints_in,
                std::set<camera_models::geometriccamera::GeometricCamera *>
                    &cameras_in);
    [[nodiscard]] KeyFrameStatus
        postLoad(std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
                 std::map<long unsigned int, MapPoint *> &mapPointId_in,
                 std::map<unsigned int,
                          camera_models::geometriccamera::GeometricCamera *>
                     &cameraId_in);

    [[nodiscard]] KeyFrameStatus
        setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);
    [[nodiscard]] KeyFrameStatus
        setKeyFrameDatabase(KeyFrameDatabase *p_keyFrameDatabase_in);

    bool isImu;

    // The following variables are accesed from only 1 thread or never change
    // (no mutex needed).
  public:
    static long unsigned int nextId;
    long unsigned int        id;
    const long unsigned int  frameId;

    const double timeStamp;

    // Grid (to speed up feature matching)
    const int   gridCols;
    const int   gridRows;
    const float gridElementWidthInverse;
    const float gridElementHeightInverse;

    // Variables used by the tracking
    long unsigned int trackReferenceFrameId;
    long unsigned int fuseTargetKeyFrameId;

    // Variables used by the local mapping
    long unsigned int baLocalKeyFrameId;
    long unsigned int baFixedKeyFrameId;

    // Number of optimizations by BA(amount of iterations in BA)
    long unsigned int optimizationCount;

    // Variables used by the keyframe database
    long unsigned int loopQuery;
    int               loopWords;
    float             loopScore;
    long unsigned int relocQuery;
    int               relocWords;
    float             relocScore;
    long unsigned int mergeQuery;
    int               mergeWords;
    float             mergeScore;
    long unsigned int placeRecognitionQuery;
    int               placeRecognitionWords;
    float             placeRecognitionScore;

    bool isInCurrentPlaceRecognition;

    // Variables used by loop closing
    Sophus::SE3f      tcwGBA;
    Sophus::SE3f      tcwBefGBA;
    Eigen::Vector3f   vwbGBA;
    Eigen::Vector3f   vwbBefGBA;
    IMU::Bias         biasGBA;
    long unsigned int baGlobalKeyFrameId;

    // Variables used by merging
    Sophus::SE3f      tcwMerge;
    Sophus::SE3f      tcwBefMerge;
    Sophus::SE3f      twcBefMerge;
    Eigen::Vector3f   vwbMerge;
    Eigen::Vector3f   vwbBefMerge;
    IMU::Bias         biasMerge;
    long unsigned int mergeCorrectedKeyFrameId;
    long unsigned int mergeKeyFrameId;
    float             scaleMerge;
    long unsigned int baLocalMergeId;

    float correctedScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, depthThreshold;
    cv::Mat     distortionCoefficients;

    // Number of KeyPoints
    const int keyPointCount;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> keyPoints;
    const std::vector<cv::KeyPoint> keyPointsUndistorted;
    const std::vector<float> uRight; // negative value for monocular points
    const std::vector<float> depths; // negative value for monocular points
    const cv::Mat            descriptors;

    // BoW
    DBoW2::BowVector     bowVector;
    DBoW2::FeatureVector featureVector;

    // Pose relative to parent (this is computed when bad flag is activated)
    Sophus::SE3f tcp;

    // Scale
    const int                scaleLevelCount;
    const float              scaleFactor;
    const float              logScaleFactor;
    const std::vector<float> scaleFactors;
    const std::vector<float> levelSigmaSquared;
    const std::vector<float> invLevelSigmaSquared;

    // Image bounds and calibration
    const int gridMinX;
    const int gridMinY;
    const int gridMaxX;
    const int gridMaxY;

    // Preintegrated IMU measurements from previous keyframe
    KeyFrame *p_prevKF;
    KeyFrame *p_nextKF;

    IMU::Preintegrated *p_imuPreintegrated;
    IMU::Calib          imuCalibration;

    unsigned int originMapId;

    std::string fileName;

    int datasetId;

    std::vector<KeyFrame *> loopCandKFs;
    std::vector<KeyFrame *> mergeCandKFs;

    // bool mbHasHessian;
    // cv::Mat mHessianPose;

    // For Semantic Segmentation
    cv::Mat colorImg;
    bool    isPublished;

    // The following variables need to be accessed trough a mutex to be thread
    // safe.
  protected:
    // sophus poses
    Sophus::SE3<float> poseTcw;
    Eigen::Matrix3f    rotationRcw;
    Sophus::SE3<float> twc;
    Eigen::Matrix3f    rotationRwc;

    // IMU position
    Eigen::Vector3f owb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f velocityVw;
    bool            isVelocityAvailable;

    // Transformation matrix between cameras in stereo fisheye
    Sophus::SE3<float> poseTlr;
    Sophus::SE3<float> poseTrl;

    // Imu bias
    IMU::Bias imuBias;

    // MapPoints associated to keypoints
    std::vector<MapPoint *> mapPoints;

    // Markers available in each keyframe
    std::vector<semantic::Marker *> mapMarkers;

    // Planes available in each keyframe
    std::vector<geometric::Plane *> mapPlanes;

    // Doorways available in each keyframe
    std::vector<vs_graphs::core::semantic::Passage *> mapPassages;

    // For save relation without pointer, this is necessary for save/load
    // function
    std::vector<long long int> backupMapPointsId;

    // BoW
    KeyFrameDatabase *p_keyFrameDatabase;
    ORBVocabulary    *p_orbVocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> grid;

    std::map<KeyFrame *, int>        connectedKeyFrameWeights;
    std::vector<KeyFrame *>          orderedConnectedKeyFrames;
    std::vector<int>                 orderedWeights;
    // For save relation without pointer, this is necessary for save/load
    // function
    std::map<long unsigned int, int> backupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    bool                           isFirstConnection;
    KeyFrame                      *p_parent;
    std::set<KeyFrame *>           childrens;
    std::set<KeyFrame *>           loopEdges;
    std::set<KeyFrame *>           mergeEdges;
    // For save relation without pointer, this is necessary for save/load
    // function
    long long int                  backupParentId;
    std::vector<long unsigned int> backupChildrensId;
    std::vector<long unsigned int> backupLoopEdgesId;
    std::vector<long unsigned int> backupMergeEdgesId;

    // Bad flags
    bool isEraseProtected;
    bool isPendingErase;
    bool isFlaggedBad;

    float halfBaseline; // Only for visualization

    // Variables to be passed to GeometricSegmentation
    std::vector<semantic::Marker *> currentFrameMarkers;
    std::vector<MapPoint *>         currentFrameMapPoints;

    // point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentFramePointClouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> currentClsCloudPtrs;

    Map *p_map;

    // Backup variables for inertial
    long long int      backupPrevKFId;
    long long int      backupNextKFId;
    IMU::Preintegrated backupImuPreintegrated;

    // Backup for Cameras
    unsigned int backupCameraId, backupCamera2Id;

    // Calibration
    Eigen::Matrix3f calibrationMatrixEigen;

    // Mutex
    std::mutex poseMutex; // for pose, velocity and biases
    std::mutex connectionsMutex;
    std::mutex featuresMutex;
    std::mutex mapMutex;

  public:
    camera_models::geometriccamera::GeometricCamera *p_camera, *p_camera2;

    // Indexes of stereo observations correspondences
    std::vector<int> leftToRightMatches, rightToLeftMatches;

    [[nodiscard]] KeyFrameStatus
        getRelativePoseTrl(Sophus::SE3f &relativePoseTrl_out);
    [[nodiscard]] KeyFrameStatus
        getRelativePoseTlr(Sophus::SE3f &relativePoseTlr_out);

    // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> keyPointsRight;

    const int leftKeyPointCount, rightKeyPointCount;

    std::vector<std::vector<std::vector<size_t>>> gridRight;

    [[nodiscard]] KeyFrameStatus
        getRightPose(Sophus::SE3<float> &rightPose_out);
    [[nodiscard]] KeyFrameStatus
        getRightPoseInverse(Sophus::SE3<float> &rightPoseInverse_out);

    [[nodiscard]] KeyFrameStatus
        getRightCameraCenter(Eigen::Vector3f &rightCameraCenter_out);
    [[nodiscard]] KeyFrameStatus
        getRightRotation(Eigen::Matrix<float, 3, 3> &rightRotation_out);
    [[nodiscard]] KeyFrameStatus
        getRightTranslation(Eigen::Vector3f &rightTranslation_out);

    [[nodiscard]] KeyFrameStatus getCurrentFrameMarkers(
        std::vector<semantic::Marker *> &getCurrentFrameMarkers_out) const;
    [[nodiscard]] KeyFrameStatus getCurrentFrameMapPoints(
        std::vector<MapPoint *> &getCurrentFrameMapPoints_out) const;

    // getters and setter for point clouds
    [[nodiscard]] KeyFrameStatus getCurrentFramePointCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCurrentFramePointCloud_out)
        const;
    [[nodiscard]] KeyFrameStatus clearPointCloud();
    [[nodiscard]] KeyFrameStatus
        getClsCloudPtrs(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
                            &getClsCloudPtrs_out) const;
    [[nodiscard]] KeyFrameStatus setCurrentClsCloudPtrs(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_clsCloudPtrs_in);
    [[nodiscard]] KeyFrameStatus clearClsClouds();
};

} // namespace core
} // namespace vs_graphs

#endif // KEYFRAME_H
