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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "Atlas.h"
#include "LocalMappingStatus.h"
#include "Types/objects/SystemParams.h"
#include "Utils/Settings/objects/Settings.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
class KeyFrame;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System       *p_system_in,
                 Atlas        *p_atlas_in,
                 const float   monocular_in,
                 bool          inertial_in,
                 const string &sequenceName_in = std::string());

    [[nodiscard]] LocalMappingStatus
        setLoopCloser(LoopClosing *p_loopCloser_in);

    [[nodiscard]] LocalMappingStatus setTracker(Tracking *p_tracker_in);

    // Main function
    void run();

    [[nodiscard]] LocalMappingStatus insertKeyFrame(KeyFrame *p_keyFrame_in);
    [[nodiscard]] LocalMappingStatus emptyQueue();

    // Thread Synch
    [[nodiscard]] LocalMappingStatus requestStop();
    [[nodiscard]] LocalMappingStatus requestReset();
    [[nodiscard]] LocalMappingStatus requestResetActiveMap(Map *p_map_in);
    [[nodiscard]] LocalMappingStatus stop(bool &isStopped_out);
    [[nodiscard]] LocalMappingStatus release();
    [[nodiscard]] LocalMappingStatus isStopped(bool &isStopped_out);
    [[nodiscard]] LocalMappingStatus stopRequested(bool &isStopRequested_out);
    [[nodiscard]] LocalMappingStatus
        isAcceptingKeyFrames(bool &isAcceptingKeyFrames_out);
    [[nodiscard]] LocalMappingStatus
        setAcceptKeyFrames(bool shouldAcceptKeyFrames_in);
    [[nodiscard]] LocalMappingStatus setNotStop(bool  shouldPreventStop_in,
                                                bool &wasSet_out);

    [[nodiscard]] LocalMappingStatus interruptBA();

    [[nodiscard]] LocalMappingStatus requestFinish();
    [[nodiscard]] LocalMappingStatus isFinished(bool &isFinished_out);

    [[nodiscard]] LocalMappingStatus keyframesInQueue(int &keyFrameCount_out)
    {
        unique_lock<std::mutex> lock(newKeyFramesMutex);
        keyFrameCount_out = newKeyFrames.size();
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    }

    [[nodiscard]] LocalMappingStatus isInitializing(bool &isInitializing_out);
    [[nodiscard]] LocalMappingStatus
        getCurrentKeyFrameTime(double &currentKeyFrameTime_out);
    [[nodiscard]] LocalMappingStatus
        getCurrentKeyFrame(KeyFrame *&p_currentKeyFrame_out);

    std::mutex imuInitMutex;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double          scale;
    double          initTime;
    double          costTime;

    unsigned int initSection;
    unsigned int initIndex;
    unsigned int keyFrameCount;
    double       firstTimestamp;
    int          matchesInliers;

    // For debugging (erase in normal mode)
    int    initFrame;
    int    iterationIndex;
    string sequence;

    bool isFirstImuBaPending;
    bool isSecondImuBaPending;
    bool isImuBad;

    bool shouldWriteStats;

    // not consider far points (clouds)
    bool  shouldSkipFarPoints;
    float farPointsThreshold;

#ifdef REGISTER_TIMES
    vector<double> keyFrameInsertTimes_ms;
    vector<double> mapPointCullingTimes_ms;
    vector<double> mapPointCreationTimes_ms;
    vector<double> localBaTimes_ms;
    vector<double> keyFrameCullingTimes_ms;
    vector<double> localMappingTotalTimes_ms;

    vector<double> localBaSyncTimes_ms;
    vector<double> keyFrameCullingSyncTimes_ms;
    vector<int>    localBaEdgeCounts;
    vector<int>    localBaOptimizedKeyFrameCounts;
    vector<int>    localBaFixedKeyFrameCounts;
    vector<int>    localBaMapPointCounts;
    int            localBaExecutionCount;
    int            localBaAbortCount;
#endif
  protected:
    [[nodiscard]] LocalMappingStatus
        checkNewKeyFrames(bool &hasNewKeyFrames_out);
    [[nodiscard]] LocalMappingStatus processNewKeyFrame();
    [[nodiscard]] LocalMappingStatus createNewMapPoints();

    [[nodiscard]] LocalMappingStatus mapPointCulling();
    [[nodiscard]] LocalMappingStatus searchInNeighbors();
    [[nodiscard]] LocalMappingStatus keyFrameCulling();

    System *p_system;

    bool isMonocular;
    bool isInertial;

    [[nodiscard]] LocalMappingStatus resetIfRequested();
    bool                             isResetRequested;
    bool                             isResetActiveMapRequested;
    Map                             *p_mapToReset;
    std::mutex                       resetMutex;

    [[nodiscard]] LocalMappingStatus checkFinish(bool &isFinishRequested_out);
    [[nodiscard]] LocalMappingStatus setFinish();
    bool                             isFinishRequested;
    bool                             hasFinished;
    std::mutex                       finishMutex;

    Atlas *p_atlas;

    LoopClosing *p_loopCloser;
    Tracking    *p_tracker;

    std::list<KeyFrame *> newKeyFrames;

    KeyFrame *p_currentKeyFrame;

    std::list<MapPoint *> recentAddedMapPoints;

    std::mutex newKeyFramesMutex;
    std::mutex newRoomsMutex;

    bool shouldAbortBa;

    bool       hasStopped;
    bool       isStopRequested;
    bool       isStopBlocked;
    std::mutex stopMutex;

    bool       shouldAcceptKeyFrames;
    std::mutex acceptMutex;

    [[nodiscard]] LocalMappingStatus
        initializeIMU(float gyroPriorWeight_in         = 1e2,
                      float accelPriorWeight_in        = 1e6,
                      bool  shouldRunFullInertialBa_in = false);
    [[nodiscard]] LocalMappingStatus scaleRefinement();

    bool isInitializationInProgress;

    Eigen::MatrixXd infoInertial;
    int             localMappingCount;
    int             keyFrameCullingCount;

    float initializationStartTime;

    int countRefinement;

    // DEBUG
    ofstream localMappingStatsFile;
};

} // namespace core
} // namespace vs_graphs
#endif
