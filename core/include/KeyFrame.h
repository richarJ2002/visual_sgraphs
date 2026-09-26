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
        ar & mnId;
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
        ar &const_cast<int &>(N);
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
        ar & firstConnection;
        ar & backupParentId;
        ar & backupChildrensId;
        ar & backupLoopEdgesId;
        ar & backupMergeEdgesId;
        // Bad flags
        ar & notErase;
        ar & toBeErased;
        ar & mbBad;

        ar & halfBaseline;

        ar & originMapId;

        // Camera variables
        ar & backupCameraId;
        ar & backupCamera2Id;

        // Fisheye variables
        ar & leftToRightMatches;
        ar & rightToLeftMatches;
        ar &const_cast<int &>(Nleft);
        ar &const_cast<int &>(Nright);
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
        ar & velocityAvailable;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

    // Pose functions
    void setPose(const Sophus::SE3f &Tcw);
    void setVelocity(const Eigen::Vector3f &Vw_);

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
    void addConnection(KeyFrame *pKF, const int &weight);
    void eraseConnection(KeyFrame *pKF);

    void                    updateConnections(bool upParent = true);
    void                    updateBestCovisibles();
    std::set<KeyFrame *>    getConnectedKeyFrames();
    std::vector<KeyFrame *> getVectorCovisibleKeyFrames();
    std::vector<KeyFrame *> getBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame *> getCovisiblesByWeight(const int &w);
    int                     getWeight(KeyFrame *pKF);

    // Spanning tree functions
    void                 addChild(KeyFrame *pKF);
    void                 eraseChild(KeyFrame *pKF);
    void                 changeParent(KeyFrame *pKF);
    std::set<KeyFrame *> getChilds();
    KeyFrame            *getParent();
    bool                 hasChild(KeyFrame *pKF);
    void                 setFirstConnection(bool bFirst);

    // Loop Edges
    void                 addLoopEdge(KeyFrame *pKF);
    std::set<KeyFrame *> getLoopEdges();

    // Merge Edges
    void            addMergeEdge(KeyFrame *pKF);
    set<KeyFrame *> getMergeEdges();

    // MapPoint observation functions
    int                     getMapPointCount();
    void                    addMapPoint(MapPoint *pMP, const size_t &idx);
    void                    eraseMapPointMatch(const int &idx);
    void                    eraseMapPointMatch(MapPoint *pMP);
    void                    replaceMapPointMatch(const int &idx, MapPoint *pMP);
    std::set<MapPoint *>    getMapPoints();
    std::vector<MapPoint *> getMapPointMatches();
    int                     getTrackedMapPointCount(const int &minObs);
    MapPoint               *getMapPoint(const size_t &idx);

    // MapMarker observation functions
    void                            addMapMarker(semantic::Marker *marker);
    std::vector<semantic::Marker *> getMapMarkers();

    // MapPlane observation functions
    void addMapPlane(geometric::Plane *plane);
    std::vector<geometric::Plane *>
         getMapPlanes(); // After getting planes, need to check for NULLs
    void removeMapPlane(geometric::Plane *plane);

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
    std::vector<size_t> getFeaturesInArea(const float &x,
                                          const float &y,
                                          const float &r,
                                          const bool   bRight = false) const;
    bool                unprojectStereo(int i, Eigen::Vector3f &x3D);

    // Image
    bool isInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void setNotErase();
    void setErase();

    // Set/check bad flag
    void setBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float computeSceneMedianDepth(const int q);

    static bool weightComp(int a, int b)
    {
        return a > b;
    }

    static bool lId(KeyFrame *pKF1, KeyFrame *pKF2)
    {
        return pKF1->mnId < pKF2->mnId;
    }

    Map *getMap();
    void updateMap(Map *pMap);

    void            setNewBias(const IMU::Bias &b);
    Eigen::Vector3f getGyroBias();

    Eigen::Vector3f getAccBias();

    IMU::Bias getImuBias();

    bool
        projectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);
    bool projectPointUnDistort(MapPoint    *pMP,
                               cv::Point2f &kp,
                               float       &u,
                               float       &v);

    void PreSave(set<KeyFrame *>                                        &spKF,
                 set<MapPoint *>                                        &spMP,
                 set<camera_models::geometriccamera::GeometricCamera *> &spCam);
    void PostLoad(
        map<long unsigned int, KeyFrame *> &mpKFid,
        map<long unsigned int, MapPoint *> &mpMPid,
        map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
            &mpCamId);

    void setORBVocabulary(ORBVocabulary *pORBVoc);
    void setKeyFrameDatabase(KeyFrameDatabase *pKFDB);

    bool isImu;

    // The following variables are accesed from only 1 thread or never change
    // (no mutex needed).
  public:
    static long unsigned int nNextId;
    long unsigned int        mnId;
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

    bool currentPlaceRecognition;

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
    const int N;

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
    bool            velocityAvailable;

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
    bool                           firstConnection;
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
    bool notErase;
    bool toBeErased;
    bool mbBad;

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
    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

  public:
    camera_models::geometriccamera::GeometricCamera *p_camera, *p_camera2;

    // Indexes of stereo observations correspondences
    std::vector<int> leftToRightMatches, rightToLeftMatches;

    Sophus::SE3f getRelativePoseTrl();
    Sophus::SE3f getRelativePoseTlr();

    // KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> keyPointsRight;

    const int Nleft, Nright;

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
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs);
    void clearClsClouds();

    void printPointDistribution()
    {
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for (int i = 0; i < N; i++)
        {
            if (mapPoints[i])
            {
                if (i < Nlim)
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
