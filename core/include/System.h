/**
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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <atomic>
#include <cstdint>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>

// JSON library
#include "Thirdparty/nlohmann/json.hpp"

#include "Atlas.h"
#include "DatabaseParser.h"
#include "FrameDrawer.h"
#include "Geometric/Plane.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticSegmentation.h"
#include "SemanticsManager.h"
#include "Settings.h"
#include "Tracking.h"
#include "Types/SystemParams.h"
#include "Viewer.h"

namespace ORB_SLAM3
{

class Verbose
{
  public:
    enum eLevel
    {
        VERBOSITY_QUIET        = 0,
        VERBOSITY_NORMAL       = 1,
        VERBOSITY_VERBOSE      = 2,
        VERBOSITY_VERY_VERBOSE = 3,
        VERBOSITY_DEBUG        = 4
    };

    static eLevel th;

  public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if (lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class SemanticSegmentation;
class SemanticsManager;

class System
{
  public:
    // Input sensor
    enum eSensor
    {
        NOT_SET       = -1,
        MONOCULAR     = 0,
        STEREO        = 1,
        RGBD          = 2,
        IMU_MONOCULAR = 3,
        IMU_STEREO    = 4,
        IMU_RGBD      = 5,
    };

    // File type
    enum FileType
    {
        TEXT_FILE   = 0,
        BINARY_FILE = 1,
    };

    struct PassageHealth
    {
        int           id{-1};
        bool          passable{false};
        std::uint64_t knownToFarCount{0U};
        std::uint64_t farToKnownCount{0U};
        std::uint64_t unknownCount{0U};
        int           knownSideRoomId{-1};
        int           farSideRoomId{-1};
    };

    struct RoomHealth
    {
        int              id{-1};
        std::vector<int> passageIds;
    };

    struct FloorHealth
    {
        int              id{-1};
        std::vector<int> roomIds;
    };

    struct MissionHealthSnapshot
    {
        double                     frameTimestamp{0.0};
        int                        trackingState{-1};
        int                        trackingInliers{0};
        bool                       inertial{false};
        bool                       inertialInitialized{false};
        bool                       poseValid{false};
        Sophus::SE3f               cameraPose_World;
        std::uint64_t              mapId{0U};
        std::uint32_t              mapCount{0U};
        std::uint32_t              keyFrameCount{0U};
        std::uint64_t              resetCount{0U};
        bool                       latestKeyFramePoseValid{false};
        double                     latestKeyFrameTimestamp{0.0};
        Sophus::SE3f               latestKeyFramePose_World;
        int                        currentRoomId{-1};
        int                        lastKnownRoomId{-1};
        std::uint32_t              confirmedRoomCount{0U};
        std::uint32_t              unresolvedRoomCount{0U};
        std::uint32_t              floorRoomLinkCount{0U};
        std::vector<RoomHealth>    rooms;
        std::vector<FloorHealth>   floors;
        std::vector<PassageHealth> passages;
        std::uint64_t              loopSequence{0U};
        std::uint32_t              acceptedLoopCount{0U};
        std::uint32_t              rejectedLoopCount{0U};
        bool                       hasLoopEvent{false};
        bool                       lastLoopAccepted{false};
        std::uint64_t              lastLoopMapId{0U};
        std::uint64_t              lastLoopCurrentKeyFrameId{0U};
        std::uint64_t              lastLoopMatchedKeyFrameId{0U};
        double                     lastLoopCurrentTimestamp{0.0};
        double                     lastLoopMatchedTimestamp{0.0};
        std::string                lastLoopReason;
    };

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief       Initialize the SLAM system. It launches the Local Mapping,
     *              Loop Closing and Viewer threads.
     *
     * @param[in]   strVocFile
     *              TODO
     *
     * @param[in]   strSettingsFile
     *              TODO
     *
     * @param[in]   strSysParamsFile
     *              TODO
     *
     * @param[in]   sensor
     *              TODO
     *
     * @param[in]   bUseViewer
     *              TODO
     *
     * @param[in]   initFr
     *              TODO
     *
     * @param[in]   strSequence
     *              TODO
     */
    System(const string &strVocFile,
           const string &strSettingsFile,
           const string &strSysParamsFile,
           const eSensor sensor,
           const bool    bUseViewer  = true,
           const int     initFr      = 0,
           const string &strSequence = std::string());

    /*!
     * @brief       Process the given stereo frame for tracking. Images must be
     *              synchronized and rectified.
     *
     * @param       imLeft
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              left camera.
     *
     * @param       imRight
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              right camera.
     *
     * @param       timestamp
     *              the timestamp of the frame.
     *
     * @param       vImuMeas
     *              the vector of IMU measurements.
     *
     * @param       filename
     *              the name of the file.
     *
     * @param       markers
     *              the vector of fiducial markers.
     *
     * @return      The camera pose (empty if tracking fails)
     */
    Sophus::SE3f
        TrackStereo(const cv::Mat            &imLeft,
                    const cv::Mat            &imRight,
                    const double             &timestamp,
                    const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                    string                    filename = "",
                    const vector<Marker *>    markers  = vector<Marker *>{});

    /**
     * @brief       Process the given rgbd frame for tracking. The DepthMap must
     *              be registered to the RGB frame.
     *
     * @param       im
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U).
     *
     * @param       depthmap
     *              The input DepthMap (CV_32F).
     *
     * @param       mainCloud
     *              The main input PointCloud before filtering.
     *
     * @param       timestamp
     *              The timestamp of the frame.
     *
     * @param       vImuMeas
     *              The vector of IMU measurements.
     *
     * @param       filename
     *              The name of the file.
     *
     * @param       markers
     *              The vector of fiducial markers.
     *
     * @return      The camera pose (empty if tracking fails)
     */
    Sophus::SE3f
        TrackRGBD(const cv::Mat                                &im,
                  const cv::Mat                                &depthmap,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mainCloud,
                  const double                                 &timestamp,
                  const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                  string                    filename = "",
                  const vector<Marker *>    markers  = vector<Marker *>{});

    /**
     * @brief       Process the given stereo frame for tracking. Images must be
     *              synchronized and rectified.
     *
     * @param       im
     *              The input RGB image (CV_8UC3) or grayscale (CV_8U) from the
     *              left camera.
     *
     * @param       timestamp
     *              The timestamp of the frame.
     *
     * @param       vImuMeas
     *              The vector of IMU measurements.
     *
     * @param       filename
     *              The name of the file.
     *
     * @param       markers
     *              The vector of fiducial markers.
     *
     * @return      The camera pose (empty if tracking fails)
     */
    Sophus::SE3f TrackMonocular(
        const cv::Mat            &im,
        const double             &timestamp,
        const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
        string                    filename = "",
        const vector<Marker *>    markers  = vector<Marker *>{});

    /*!
     * @brief       This stops local mapping thread (map building) and performs
     *              only camera tracking.
     */
    void ActivateLocalizationMode();

    /*!
     * @brief        This resumes local mapping thread and performs SLAM again.
     */
    void DeactivateLocalizationMode();

    /**
     * @brief       Get the current active map in Atlas.
     *
     * @return      The current active map.
     */
    ORB_SLAM3::Map *GetCurrentMap();

    /*!
     * @brief       Returns true if there have been a big map change (loop
     *              closure, global BA) since last call to this function.
     */
    bool MapChanged();

    MissionHealthSnapshot
        GetMissionHealthSnapshot(bool includeSemantics = true);

    /*!
     * @brief       Reset the system (clear Atlas or the active map).
     */
    void Reset();

    /*!
     * @brief       TODO
     */
    void ResetActiveMap();

    /*!
     * @brief       All threads will be requested to finish. It waits until all
     *              threads have finished. This function must be called before
     *              saving the trajectory.
     */
    void Shutdown();

    /*!
     * @brief       Reset the system (clear Atlas or the active map).
     */
    bool isShutDown();

    /*!
     * @brief       Save camera trajectory in the TUM RGB-D dataset format.
     *              Only for stereo and RGB-D. This method does not work for
     *              monocular.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    void SaveTrajectoryTUM(const string &filename);

    /*!
     * @brief       Save keyframe poses in the TUM RGB-D dataset format. This
     *              method works for all sensor input.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    /*!
     * @brief       TODO
     */
    void SaveTrajectoryEuRoC(const string &filename);

    /*!
     * @brief       TODO
     */
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    /*!
     * @brief       TODO
     */
    void SaveTrajectoryEuRoC(const string &filename, Map *pMap);

    /*!
     * @brief       TODO
     */
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);

    /*!
     * @brief       Save data used for initialization debug.
     */
    void SaveDebugData(const int &iniIdx);

    //
    // Call first Shutdown()
    // See format details at:
    // http://www.cvlibs.net/datasets/kitti/eval_odometry.php

    /*!
     * @brief       Save camera trajectory in the KITTI dataset format. Only for
     *              stereo and RGB-D. This method does not work for monocular.
     *
     * @note        Call first Shutdown()
     *
     * @note        See format details at:
     *              http://vision.in.tum.de/data/datasets/rgbd-dataset
     */
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    bool SaveMap(const string &filename);
    bool SaveMapPointsAsPCD(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int                                GetTrackingState();
    cv::Mat                            GetCurrentFrame();
    std::vector<ORB_SLAM3::Room *>     GetAllRooms();
    std::vector<ORB_SLAM3::Floor *>    GetAllFloors();
    std::vector<ORB_SLAM3::Plane *>    GetAllPlanes();
    std::vector<ORB_SLAM3::Door *>     GetAllDoors();
    std::vector<ORB_SLAM3::Marker *>   GetAllMarkers();
    std::vector<ORB_SLAM3::Passage *>  GetAllPassages();
    std::vector<ORB_SLAM3::KeyFrame *> GetAllKeyFrames();
    std::vector<ORB_SLAM3::MapPoint *> GetAllMapPoints();
    std::vector<ORB_SLAM3::MapPoint *> GetTrackedMapPoints();
    std::vector<Sophus::SE3f>          GetAllKeyframePoses();
    std::vector<cv::KeyPoint>          GetTrackedKeyPointsUn();

    // singular version of GetAllKeyFrames
    Sophus::SE3f GetKeyFramePose(KeyFrame *pKF);

    Sophus::SE3f    GetCamTwc();
    Sophus::SE3f    GetImuTwb();
    Eigen::Vector3f GetImuVwb();
    bool            isImuPreintegrated();

    // For debugging
    double GetTimeFromIMUInit();
    bool   isLost();
    bool   isFinished();

    void ChangeDataset();

    float GetImageScale();

    /*!
     * @brief       Parse the JSON file containing the environment data
     *
     * @param       jsonFilePath
     *              The path to the JSON file
     */
    void parseJsonDatabase(string jsonFilePath);

    /*!
     * @brief       Add the segmented image to the buffer in the
     *              SemanticSegmentation
     *
     * @param       tuple
     *              The address of the tuple of segmented image and pointcloud
     */
    void addSegmentedImage(
        std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple);

    /*!
     * @brief       Get the skeleton cluster coming from the current map
     */
    std::vector<std::vector<Eigen::Vector3d>> getSkeletonCluster();

    /*!
     * @brief       Update the skeleton cluster coming from `voxblox_skeleton`
     *              in the map.
     *
     * @param[in]   skeletonClusterPoints_World_m_in
     *              the skeleton cluster points
     */
    void setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>>
                                &skeletonClusterPoints_World_m_in);

    /*!
     * @brief       Stores the latest connected Voxblox skeleton edges.
     *
     * @param[in]   skeletonEdges_World_m_in
     *              Start and end points of each connected skeleton edge.
     */
    void setSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &skeletonEdges_World_m_in);

    /*!
     * @brief       Update the GNN room candidates list
     */
    void setGNNRoomCandidates(
        const std::vector<ORB_SLAM3::Room *> &gnnRoomCandidates);

#ifdef REGISTER_TIMES
    void InsertRectTime(double &time);
    void InsertResizeTime(double &time);
    void InsertTrackTime(double &time);
#endif

  private:
    bool SaveAtlas(int type);
    bool LoadAtlas(int type);

    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary *mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop
    // detection).
    KeyFrameDatabase *mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    // Map* mpMap;
    Atlas *mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints
    // and performs relocalization if tracking fails.
    Tracking *mpTracker;

    // Local Mapper. It manages the local map and performs local bundle
    // adjustment.
    LocalMapping *mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a
    // loop it performs a pose graph optimization and full bundle adjustment (in
    // a new thread) afterwards.
    LoopClosing *mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer *mpViewer;

    FrameDrawer *mpFrameDrawer;
    MapDrawer   *mpMapDrawer;

    // Geometric & Semantic Segmentation
    SemanticSegmentation *mpSemanticSegmentation;
    SemanticsManager     *mpSemanticsManager;

    // List of rooms in the environment
    std::vector<ORB_SLAM3::Room *> envRooms;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // 🚀 [vS-Graphs v.2.0] Two new threads: Geometric Segmentation and Semantic
    // Segmentation The Tracking thread "lives" in the main execution thread
    // that creates the System object.
    std::thread *mptViewer;
    std::thread *mptLoopClosing;
    std::thread *mptLocalMapping;
    std::thread *mptSemanticSegmentation;
    std::thread *mptSemanticsManager;
    std::thread *mptGeometricSegmentation;

    // Reset flag
    std::mutex mMutexReset;
    bool       mbReset;
    bool       mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool       mbActivateLocalizationMode;
    bool       mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int                        mTrackingState{-1};
    int                        mTrackingInliers{0};
    double                     mLastFrameTimestamp{0.0};
    Sophus::SE3f               mCurrentCameraPose_World;
    bool                       mCurrentCameraPoseValid{false};
    std::atomic<std::uint64_t> mResetCount{0U};
    std::vector<MapPoint *>    mTrackedMapPoints;
    std::vector<cv::KeyPoint>  mTrackedKeyPointsUn;
    std::mutex                 mMutexState;

    /*!
     * @brief Map ID of the most recently processed frame, used to detect
     *        map restarts for room-context carryover (WP1).
     */
    long unsigned int mLastProcessedMapId{0};
    bool              mFirstMapInit{true};

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    Settings *settings_;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_H
