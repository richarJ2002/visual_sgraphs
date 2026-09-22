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
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "Utils/Settings/objects/Settings.h"
#include "Tracking.h"
#include "Types/objects/SystemParams.h"

#include <mutex>

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
    LocalMapping(System       *pSys,
                 Atlas        *pAtlas,
                 const float   bMonocular,
                 bool          bInertial,
                 const string &_strSeqName = std::string());

    void setLoopCloser(LoopClosing *pLoopCloser);

    void setTracker(Tracking *pTracker);

    // Main function
    void run();

    void insertKeyFrame(KeyFrame *pKF);
    void emptyQueue();

    // Thread Synch
    void requestStop();
    void requestReset();
    void requestResetActiveMap(Map *pMap);
    bool stop();
    void release();
    bool isStopped();
    bool stopRequested();
    bool isAcceptingKeyFrames();
    void setAcceptKeyFrames(bool flag);
    bool setNotStop(bool flag);

    void interruptBA();

    void requestFinish();
    bool isFinished();

    int keyframesInQueue()
    {
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return newKeyFrames.size();
    }

    bool      isInitializing();
    double    getCurrentKeyFrameTime();
    KeyFrame *getCurrentKeyFrame();

    std::mutex mMutexImuInit;

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

    bool notBA1;
    bool notBA2;
    bool badImu;

    bool writeStats;

    // not consider far points (clouds)
    bool  farPoints;
    float farPointsThreshold;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;

    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int>    vnLBA_edges;
    vector<int>    vnLBA_KFopt;
    vector<int>    vnLBA_KFfixed;
    vector<int>    vnLBA_MPs;
    int            nLBA_exec;
    int            nLBA_abort;
#endif
  protected:
    bool checkNewKeyFrames();
    void processNewKeyFrame();
    void createNewMapPoints();

    void mapPointCulling();
    void searchInNeighbors();
    void keyFrameCulling();

    System *p_system;

    bool monocular;
    bool inertial;

    void       resetIfRequested();
    bool       resetRequested;
    bool       resetActiveMapRequested;
    Map       *p_mapToReset;
    std::mutex mMutexReset;

    bool       checkFinish();
    void       setFinish();
    bool       finishRequested;
    bool       finished;
    std::mutex mMutexFinish;

    Atlas *p_atlas;

    LoopClosing *p_loopCloser;
    Tracking    *p_tracker;

    std::list<KeyFrame *> newKeyFrames;

    KeyFrame *p_currentKeyFrame;

    std::list<MapPoint *> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;
    std::mutex mMutexNewRooms;

    bool abortBA;

    bool       stopped;
    bool       stopRequestedFlag;
    bool       notStop;
    std::mutex mMutexStop;

    bool       acceptKeyFrames;
    std::mutex mMutexAccept;

    void initializeIMU(float priorG = 1e2,
                       float priorA = 1e6,
                       bool  bFirst = false);
    void scaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int             localMappingCount;
    int             keyFrameCullingCount;

    float initializationStartTime;

    int countRefinement;

    // DEBUG
    ofstream f_lm;
};

} // namespace core
} // namespace vs_graphs
#endif
