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

    void setLoopCloser(LoopClosing *p_loopCloser_in);

    void setTracker(Tracking *p_tracker_in);

    // Main function
    void run();

    void insertKeyFrame(KeyFrame *p_keyFrame_in);
    void emptyQueue();

    // Thread Synch
    void requestStop();
    void requestReset();
    void requestResetActiveMap(Map *p_map_in);
    bool stop();
    void release();
    bool isStopped();
    bool stopRequested();
    bool isAcceptingKeyFrames();
    void setAcceptKeyFrames(bool shouldAcceptKeyFrames_in);
    bool setNotStop(bool shouldPreventStop_in);

    void interruptBA();

    void requestFinish();
    bool isFinished();

    int keyframesInQueue()
    {
        unique_lock<std::mutex> lock(newKeyFramesMutex);
        return newKeyFrames.size();
    }

    bool      isInitializing();
    double    getCurrentKeyFrameTime();
    KeyFrame *getCurrentKeyFrame();

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
    bool checkNewKeyFrames();
    void processNewKeyFrame();
    void createNewMapPoints();

    void mapPointCulling();
    void searchInNeighbors();
    void keyFrameCulling();

    System *p_system;

    bool isMonocular;
    bool isInertial;

    void       resetIfRequested();
    bool       isResetRequested;
    bool       isResetActiveMapRequested;
    Map       *p_mapToReset;
    std::mutex resetMutex;

    bool       checkFinish();
    void       setFinish();
    bool       isFinishRequested;
    bool       hasFinished;
    std::mutex finishMutex;

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

    void initializeIMU(float gyroPriorWeight_in         = 1e2,
                       float accelPriorWeight_in        = 1e6,
                       bool  shouldRunFullInertialBa_in = false);
    void scaleRefinement();

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
