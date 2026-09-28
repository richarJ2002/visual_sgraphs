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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "Frame.h"
#include "Geometric/Plane.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

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
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & id;
        ar &const_cast<long unsigned int &>(frameId);
        ar &const_cast<double &>(timeStamp);
        // Grid
        ar &const_cast<int &>(gridCols);
        ar &const_cast<int &>(gridRows);
        ar &const_cast<float &>(gridElementWidthInverse);
        ar &const_cast<float &>(gridElementHeightInverse);

        // Variables of tracking
        // ar & trackReferenceFrameId;
        // ar & fuseTargetKeyFrameId;
        // Variables of local mapping
        // ar & baLocalKeyFrameId;
        // ar & baFixedKeyFrameId;
        // ar & optimizationCount;
        // Variables used by KeyFrameDatabase
        // ar & mnLoopQuery;
        // ar & mnLoopWords;
        // ar & mLoopScore;
        // ar & mnRelocQuery;
        // ar & mnRelocWords;
        // ar & mRelocScore;
        // ar & mnMergeQuery;
        // ar & mnMergeWords;
        // ar & mMergeScore;
        // ar & mnPlaceRecognitionQuery;
        // ar & mnPlaceRecognitionWords;
        // ar & mPlaceRecognitionScore;
        // ar & mbCurrentPlaceRecognition;
        // Variables of loop closing
        // serializeMatrix(ar,mTcwGBA,version);
        // serializeMatrix(ar,mTcwBefGBA,version);
        // serializeMatrix(ar,mVwbGBA,version);
        // serializeMatrix(ar,mVwbBefGBA,version);
        // ar & mBiasGBA;
        // ar & baGlobalKeyFrameId;
        // Variables of Merging
        // serializeMatrix(ar,mTcwMerge,version);
        // serializeMatrix(ar,mTcwBefMerge,version);
        // serializeMatrix(ar,mTwcBefMerge,version);
        // serializeMatrix(ar,mVwbMerge,version);
        // serializeMatrix(ar,mVwbBefMerge,version);
        // ar & mBiasMerge;
        // ar & mergeCorrectedKeyFrameId;
        // ar & mergeKeyFrameId;
        // ar & mfScaleMerge;
        // ar & baLocalMergeId;

        // Scale
        ar & correctedScale;
        // Calibration parameters
        ar &const_cast<float &>(fx);
        ar &const_cast<float &>(fy);
        ar &const_cast<float &>(invfx);
        ar &const_cast<float &>(invfy);
        ar &const_cast<float &>(cx);
        ar &const_cast<float &>(cy);
        ar &const_cast<float &>(mbf);
        ar &const_cast<float &>(mb);
        ar &const_cast<float &>(depthThreshold);
        serializeMatrix(ar, distortionCoefficients, version);
        // Number of Keypoints
        ar &const_cast<int &>(keyPointCount);
        // KeyPoints
        serializeVectorKeyPoints<Archive>(ar, keyPoints, version);
        serializeVectorKeyPoints<Archive>(ar, keyPointsUndistorted, version);
        ar &const_cast<vector<float> &>(uRight);
        ar &const_cast<vector<float> &>(depths);
        serializeMatrix<Archive>(ar, descriptors, version);
        // BOW
        ar & bowVector;
        ar & featureVector;
        // Pose relative to parent
        serializeSophusSE3<Archive>(ar, tcp, version);
        // Scale
        ar &const_cast<int &>(scaleLevelCount);
        ar &const_cast<float &>(scaleFactor);
        ar &const_cast<float &>(logScaleFactor);
        ar &const_cast<vector<float> &>(scaleFactors);
        ar &const_cast<vector<float> &>(levelSigmaSquared);
        ar &const_cast<vector<float> &>(invLevelSigmaSquared);
        // Image bounds and calibration
        ar &const_cast<int &>(gridMinX);
        ar &const_cast<int &>(gridMinY);
        ar &const_cast<int &>(gridMaxX);
        ar &const_cast<int &>(gridMaxY);
        ar &boost::serialization::make_array(calibrationMatrixEigen.data(),
                                             calibrationMatrixEigen.size());
        // Pose
        serializeSophusSE3<Archive>(ar, poseTcw, version);
        // MapPointsId associated to keypoints
        ar & backupMapPointsId;
        // Grid
        ar & grid;
        // Connected KeyFrameWeight
        ar & backupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges
        ar & isFirstConnection;
        ar & backupParentId;
        ar & backupChildrensId;
        ar & backupLoopEdgesId;
        ar & backupMergeEdgesId;
        // Bad flags
        ar & isEraseProtected;
        ar & isPendingErase;
        ar & isFlaggedBad;

        ar & halfBaseline;

        ar & originMapId;

        // Camera variables
        ar & backupCameraId;
        ar & backupCamera2Id;

        // Fisheye variables
        ar & leftToRightMatches;
        ar & rightToLeftMatches;
        ar &const_cast<int &>(leftKeyPointCount);
        ar &const_cast<int &>(rightKeyPointCount);
        serializeSophusSE3<Archive>(ar, poseTlr, version);
        serializeVectorKeyPoints<Archive>(ar, keyPointsRight, version);
        ar & gridRight;

        // Inertial variables
        ar & imuBias;
        ar & backupImuPreintegrated;
        ar & imuCalibration;
        ar & backupPrevKFId;
        ar & backupNextKFId;
        ar & isImu;
        ar &boost::serialization::make_array(velocityVw.data(),
                                             velocityVw.size());
        ar &boost::serialization::make_array(owb.data(), owb.size());
        ar & isVelocityAvailable;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame            &F_inout,
             Map              *p_map_in,
             KeyFrameDatabase *p_keyFrameDatabase_in);

    // Pose functions
    void setPose(const Sophus::SE3f &Tcw_in);
    void setVelocity(const Eigen::Vector3f &Vw_in);

    Sophus::SE3f getPose();

    Sophus::SE3f    getPoseInverse();
    Eigen::Vector3f getCameraCenter();

    Eigen::Vector3f getImuPosition();
    Eigen::Matrix3f getImuRotation();
    Sophus::SE3f    getImuPose();
    Eigen::Matrix3f getRotation();
    Eigen::Vector3f getTranslation();
    Eigen::Vector3f getVelocity();
    bool            isVelocitySet();

    // Bag of Words Representation
    void computeBagOfWords();

    // Covisibility graph functions
    void addConnection(KeyFrame *p_keyFrame_inout, const int &weight_in);
    void eraseConnection(KeyFrame *p_keyFrame_in);

    void                    updateConnections(bool upParent_in = true);
    void                    updateBestCovisibles();
    std::set<KeyFrame *>    getConnectedKeyFrames();
    std::vector<KeyFrame *> getVectorCovisibleKeyFrames();
    std::vector<KeyFrame *> getBestCovisibilityKeyFrames(const int &N_in);
    std::vector<KeyFrame *> getCovisiblesByWeight(const int &w_in);
    int                     getWeight(KeyFrame *p_keyFrame_in);

    // Spanning tree functions
    void                 addChild(KeyFrame *p_keyFrame_in);
    void                 eraseChild(KeyFrame *p_keyFrame_in);
    void                 changeParent(KeyFrame *p_keyFrame_inout);
    std::set<KeyFrame *> getChilds();
    KeyFrame            *getParent();
    bool                 hasChild(KeyFrame *p_keyFrame_in);
    void                 setFirstConnection(bool isFirst_in);

    // Loop Edges
    void                 addLoopEdge(KeyFrame *p_keyFrame_in);
    std::set<KeyFrame *> getLoopEdges();

    // Merge Edges
    void            addMergeEdge(KeyFrame *p_keyFrame_in);
    set<KeyFrame *> getMergeEdges();

    // MapPoint observation functions
    int  getMapPointCount();
    void addMapPoint(MapPoint *p_mapPoint_in, const size_t &index_in);
    void eraseMapPointMatch(const int &index_in);
    void eraseMapPointMatch(MapPoint *p_mapPoint_in);
    void replaceMapPointMatch(const int &index_in, MapPoint *p_mapPoint_in);
    std::set<MapPoint *>    getMapPoints();
    std::vector<MapPoint *> getMapPointMatches();
    int       getTrackedMapPointCount(const int &minimumObservation_in);
    MapPoint *getMapPoint(const size_t &index_in);

    // MapMarker observation functions
    void                            addMapMarker(semantic::Marker *p_marker_in);
    std::vector<semantic::Marker *> getMapMarkers();

    // MapPlane observation functions
    void addMapPlane(geometric::Plane *p_plane_in);
    std::vector<geometric::Plane *>
         getMapPlanes(); // After getting planes, need to check for NULLs
    void removeMapPlane(geometric::Plane *p_plane_in);

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
     * @return      True when the retired plane was present.
     */
    bool replaceMapPlane(geometric::Plane *p_retiredPlane_in,
                         geometric::Plane *p_retainedPlane_in);

    /*!
     * @brief       Adds a mapped passage association.
     *
     * @param[in]   p_passage_in Passage observed by this keyframe.
     */
    void addMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);

    /*!
     * @brief       Replaces a retired passage association after fusion.
     *
     * @param[in]   p_retiredPassage_in Duplicate passage being retired.
     * @param[in]   p_retainedPassage_in Passage retaining the associations.
     *
     * @return      True when the retired passage was present.
     */
    bool replaceMapPassage(
        vs_graphs::core::semantic::Passage *p_retiredPassage_in,
        vs_graphs::core::semantic::Passage *p_retainedPassage_in);

    std::vector<vs_graphs::core::semantic::Passage *> getMapPassages();

    // KeyPoint functions
    std::vector<size_t>
         getFeaturesInArea(const float &x_in,
                           const float &y_in,
                           const float &r_in,
                           const bool   isRightCamera_in = false) const;
    bool unprojectStereo(int index_in, Eigen::Vector3f &x3D_out);

    // Image
    bool isInImage(const float &x_in, const float &y_in) const;

    // Enable/Disable bad flag changes
    void setNotErase();
    void setErase();

    // Set/check bad flag
    void setBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float computeSceneMedianDepth(const int q_in);

    static bool weightComp(int a_in, int b_in)
    {
        return a_in > b_in;
    }

    static bool lId(KeyFrame *p_keyFrame1_inout, KeyFrame *p_keyFrame2_inout)
    {
        return p_keyFrame1_inout->id < p_keyFrame2_inout->id;
    }

    Map *getMap();
    void updateMap(Map *p_map_in);

    void            setNewBias(const IMU::Bias &b_in);
    Eigen::Vector3f getGyroBias();

    Eigen::Vector3f getAccBias();

    IMU::Bias getImuBias();

    bool projectPointDistort(MapPoint    *p_mapPoint_in,
                             cv::Point2f &keyPoint_out,
                             float       &u_out,
                             float       &v_out);
    bool projectPointUnDistort(MapPoint    *p_mapPoint_in,
                               cv::Point2f &keyPoint_out,
                               float       &u_out,
                               float       &v_out);

    void preSave(
        set<KeyFrame *>                                        &keyFrames_in,
        set<MapPoint *>                                        &mapPoints_in,
        set<camera_models::geometriccamera::GeometricCamera *> &cameras_in);
    void postLoad(
        map<long unsigned int, KeyFrame *> &keyFrameId_in,
        map<long unsigned int, MapPoint *> &mapPointId_in,
        map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
            &cameraId_in);

    void setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);
    void setKeyFrameDatabase(KeyFrameDatabase *p_keyFrameDatabase_in);

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

    string fileName;

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

    Sophus::SE3f getRelativePoseTrl();
    Sophus::SE3f getRelativePoseTlr();

    // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> keyPointsRight;

    const int leftKeyPointCount, rightKeyPointCount;

    std::vector<std::vector<std::vector<size_t>>> gridRight;

    Sophus::SE3<float> getRightPose();
    Sophus::SE3<float> getRightPoseInverse();

    Eigen::Vector3f            getRightCameraCenter();
    Eigen::Matrix<float, 3, 3> getRightRotation();
    Eigen::Vector3f            getRightTranslation();

    std::vector<semantic::Marker *> getCurrentFrameMarkers() const;
    std::vector<MapPoint *>         getCurrentFrameMapPoints() const;

    // getters and setter for point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentFramePointCloud() const;
    void                                   clearPointCloud();
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
         getClsCloudPtrs() const;
    void setCurrentClsCloudPtrs(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_clsCloudPtrs_in);
    void clearClsClouds();

    void printPointDistribution()
    {
        int left = 0, right = 0;
        int limCount =
            (leftKeyPointCount != -1) ? leftKeyPointCount : keyPointCount;
        for (int keyPointIndex = 0; keyPointIndex < keyPointCount;
             keyPointIndex++)
        {
            if (mapPoints[keyPointIndex])
            {
                if (keyPointIndex < limCount)
                    left++;
                else
                    right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left
             << " --- right-> " << right << endl;
    }
};

} // namespace core
} // namespace vs_graphs

#endif // KEYFRAME_H
