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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <atomic>
#include <boost/algorithm/string.hpp>
#include <cstdint>
#include <mutex>
#include <thread>

#include "Atlas.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Tracking.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;

class LoopClosing
{
  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE CONSTANT
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The main Run() function runs every runInterval_s seconds.
     */
    const double runInterval_s = 3.0;

  public:
    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Thread-safe summary of the latest validated loop event.
     */
    struct LoopCorrectionStatus
    {
        std::uint64_t sequence{0U};
        std::uint32_t acceptedCount{0U};
        std::uint32_t rejectedCount{0U};
        bool          hasEvent{false};
        bool          lastAccepted{false};
        unsigned long lastMapId{0U};
        unsigned long lastCurrentKeyFrameId{0U};
        unsigned long lastMatchedKeyFrameId{0U};
        double        lastCurrentTimestamp{0.0};
        double        lastMatchedTimestamp{0.0};
        std::string   lastReason;
    };

    typedef pair<set<KeyFrame *>, int> ConsistentGroup;

    typedef map<KeyFrame *,
                g2o::Sim3,
                std::less<KeyFrame *>,
                Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>
        KeyFrameAndPose;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       TODO
     *
     * @param[in]   pAtlas
     *              TODO
     *
     * @param[in]   pDB
     *              TODO
     *
     * @param[in]   pVoc
     *              TODO
     *
     * @param[in]   bFixScale
     *              TODO
     *
     * @param[in]   bActiveLC
     *              TODO
     */
    LoopClosing(Atlas            *pAtlas,
                KeyFrameDatabase *pDB,
                ORBVocabulary    *pVoc,
                const bool        bFixScale,
                const bool        bActiveLC);

    /*!
     * @brief       TODO
     *
     * @param[in]   pTracker
     *              TODO
     */
    void SetTracker(Tracking *pTracker);

    /*!
     * @brief       TODO
     *
     * @param[in]   pLocalMapper
     *              TODO
     */
    void SetLocalMapper(LocalMapping *pLocalMapper);

    /*!
     * @brief       TODO
     *
     * @param[in]   mergeStatus_in
     *              TODO
     */
    void SetMergeStatus(bool mergeStatus_in);

    /*!
     * @brief       TODO
     */
    void Run(void);

    /*!
     * @brief       TODO
     *
     * @param[in]   pKF
     *              TODO
     */
    void InsertKeyFrame(KeyFrame *pKF);

    /*!
     * @brief       TODO
     */
    void RequestReset();

    /*!
     * @brief       TODO
     *
     * @param[in]   pMap
     *              TODO
     */
    void RequestResetActiveMap(Map *pMap);

    /*!
     * @brief       TODO
     *
     * @note        This function will run in a separate thread
     *
     * @param[in]   pActiveMap
     *              TODO
     *
     * @param[in]   nLoopKF
     *              TODO
     *
     * @param[in]   generation_in
     *              TODO
     */
    void RunGlobalBundleAdjustment(Map          *pActiveMap,
                                   unsigned long nLoopKF,
                                   unsigned int  generation_in);

    /*!
     * @brief       TODO
     */
    bool isRunningGBA(void)
    {
        /* Lock mutext */
        unique_lock<std::mutex> lock(mMutexGBA);

        /* Return flag to indicate if global bundal adjustemnt is running */
        return mbRunningGBA;
    }

    /*!
     * @brief       TODO
     */
    bool isFinishedGBA(void)
    {
        /* Lock mutext */
        unique_lock<std::mutex> lock(mMutexGBA);

        /* Return flag to indicate if global bundal adjustemnt is finished */
        return mbFinishedGBA;
    }

    /*!
     * @brief       TODO
     */
    void RequestFinish(void);

    /*!
     * @brief       TODO
     */
    bool isFinished(void);

    /*!
     * @brief       TODO
     */
    bool isMergeInProgress(void);

    /*!
     * @brief       TODO
     */
    LoopCorrectionStatus GetLoopCorrectionStatus() const;

    Viewer *mpViewer;

#ifdef REGISTER_TIMES

    vector<double> vdDataQuery_ms;
    vector<double> vdEstSim3_ms;
    vector<double> vdPRTotal_ms;

    vector<double> vdMergeMaps_ms;
    vector<double> vdWeldingBA_ms;
    vector<double> vdMergeOptEss_ms;
    vector<double> vdMergeTotal_ms;
    vector<int>    vnMergeKFs;
    vector<int>    vnMergeMPs;
    int            nMerges;

    vector<double> vdLoopFusion_ms;
    vector<double> vdLoopOptEss_ms;
    vector<double> vdLoopTotal_ms;
    vector<int>    vnLoopKFs;
    int            nLoop;

    vector<double> vdGBA_ms;
    vector<double> vdUpdateMap_ms;
    vector<double> vdFGBATotal_ms;
    vector<int>    vnGBAKFs;
    vector<int>    vnGBAMPs;
    int            nFGBA_exec;
    int            nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    /* ---------------------------------------------------------------------- *
     * PROTECTED MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief      TODO
     */
    bool mbResetRequested;

    /*!
     * @brief      TODO
     */
    bool mbResetActiveMapRequested;

    /*!
     * @brief      TODO
     */
    Map *mpMapToReset;

    /*!
     * @brief      TODO
     */
    std::mutex mMutexReset;

    /*!
     * @brief      TODO
     */
    bool mbFinishRequested;

    /*!
     * @brief      TODO
     */
    bool mbFinished;

    /*!
     * @brief      TODO
     */
    std::mutex mMutexFinish;

    /*!
     * @brief      TODO
     */
    Atlas *mpAtlas;

    /*!
     * @brief      TODO
     */
    Tracking *mpTracker;

    /*!
     * @brief      TODO
     */
    KeyFrameDatabase *mpKeyFrameDB;

    /*!
     * @brief      TODO
     */
    ORBVocabulary *mpORBVocabulary;

    /*!
     * @brief      TODO
     */
    LocalMapping *mpLocalMapper;

    /*!
     * @brief      TODO
     */
    std::list<KeyFrame *> mlpLoopKeyFrameQueue;

    /*!
     * @brief      TODO
     */
    std::mutex mMutexLoopQueue;

    /*!
     * @brief      Loop detector parameters
     */
    float mnCovisibilityConsistencyTh;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpCurrentKF;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpLastCurrentKF;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpMatchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<ConsistentGroup> mvConsistentGroups;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> mvpEnoughConsistentCandidates;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> mvpCurrentConnectedKFs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpCurrentMatchedPoints;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpLoopMapPoints;

    /*!
     * @brief      TODO
     */
    cv::Mat mScw;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oScw;

    /*!
     * @brief      TODO
     */
    Map *mpLastMap;

    /*!
     * @brief      TODO
     */
    bool mbLoopDetected;

    /*!
     * @brief      TODO
     */
    int mnLoopNumCoincidences;

    /*!
     * @brief      TODO
     */
    int mnLoopNumNotFound;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpLoopLastCurrentKF;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oLoopSlw;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oLoopScw;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpLoopMatchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpLoopMPs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpLoopMatchedMPs;

    /*!
     * @brief      TODO
     */
    bool mbMergeDetected;

    /*!
     * @brief      TODO
     */
    std::atomic_bool mbMergeInProgress;

    /*!
     * @brief      TODO
     */
    int mnMergeNumCoincidences;

    /*!
     * @brief      TODO
     */
    int mnMergeNumNotFound;

    /*!
     * @brief      TODO
     */
    KeyFrame *mpMergeLastCurrentKF;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oMergeSlw;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oMergeSmw;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oMergeScw;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oMergeSw1w2;

    /*!
     * @brief      TODO
     */

    /*!
     * @brief      TODO
     */
    KeyFrame *mpMergeMatchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpMergeMPs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mvpMergeMatchedMPs;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> mvpMergeConnectedKFs;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mSold_new;

    /*!
     * @brief      TODO
     */
    long unsigned int mLastLoopKFid;

    /*!
     * @brief      TODO
     */
    bool mbRunningGBA;

    /*!
     * @brief      TODO
     */
    bool mbFinishedGBA;

    /*!
     * @brief      TODO
     */
    std::mutex mMutexGBA;

    /*!
     * @brief      TODO
     */
    std::thread *mpThreadGBA;

    /*!
     * @brief      TODO
     */
    std::atomic_bool globalBundleAdjustmentStopRequested{false};

    /*!
     * @brief      TODO
     *
     * @note        Fix scale in the stereo/RGB-D case
     */
    bool mbFixScale;

    /*!
     * @brief      TODO
     */
    unsigned int mnFullBAIdx;

    /*!
     * @brief      TODO
     */
    vector<double> vdPR_CurrentTime;

    /*!
     * @brief      TODO
     */
    vector<double> vdPR_MatchedTime;

    /*!
     * @brief      TODO
     */
    vector<int> vnPR_TypeRecogn;

    /*!
     * @brief      TODO
     */
    string mstrFolderSubTraj;

    /*!
     * @brief      TODO
     */
    int mnNumCorrection;

    /*!
     * @brief      TODO
     */
    int mnCorrectionGBA;

    /*!
     * @brief      TODO
     */
    mutable std::mutex mMutexLoopCorrectionStatus;

    /*!
     * @brief      TODO
     */
    LoopCorrectionStatus mLoopCorrectionStatus;

    /*!
     * @brief      TODO
     */
    bool mbActiveLC = true;

    /*!
     * @brief      TODO
     */
    SystemParams *sysParams;

    /* ---------------------------------------------------------------------- *
     * PROTECTED METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       TODO
     */
    bool CheckNewKeyFrames(void);

    /*!
     * @brief       TODO
     */
    bool NewDetectCommonRegions(void);

    /*!
     * @brief       TODO
     *
     * @param[in]   pCurrentKF
     *              TODO
     *
     * @param[in]   pMatchedKF
     *              TODO
     *
     * @param[in]   gScw
     *              TODO
     *
     * @param[in]   nNumProjMatches
     *              TODO
     *
     * @param[in]   vpMPs
     *              TODO
     *
     * @param[in]   vpMatchedMPs
     *              TODO
     */
    bool DetectAndReffineSim3FromLastKF(KeyFrame  *pCurrentKF,
                                        KeyFrame  *pMatchedKF,
                                        g2o::Sim3 &gScw,
                                        int       &nNumProjMatches,
                                        std::vector<MapPoint *> &vpMPs,
                                        std::vector<MapPoint *> &vpMatchedMPs);

    /*!
     * @brief       TODO
     *
     * @param[in]   vpBowCand
     *              TODO
     *
     * @param[in]   pMatchedKF
     *              TODO
     *
     * @param[in]   pLastCurrentKF
     *              TODO
     *
     * @param[in]   nNumProjMatches
     *              TODO
     *
     * @param[in]   g2oScw
     *              TODO
     *
     * @param[in]   nNumCoincidences
     *              TODO
     *
     * @param[in]   vpMPs
     *              TODO
     *
     * @param[in]   vpMatchedMPs
     *              TODO
     */
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand,
                                    KeyFrame               *&pMatchedKF,
                                    KeyFrame               *&pLastCurrentKF,
                                    g2o::Sim3               &g2oScw,
                                    int                     &nNumCoincidences,
                                    std::vector<MapPoint *> &vpMPs,
                                    std::vector<MapPoint *> &vpMatchedMPs);

    /*!
     * @brief       TODO
     *
     * @param[in]   pCurrentKF
     *              TODO
     *
     * @param[in]   pMatchedKF
     *              TODO
     *
     * @param[in]   gScw
     *              TODO
     *
     * @param[in]   nNumProjMatches
     *              TODO
     *
     * @param[in]   vpMPs
     *              TODO
     *
     * @param[in]   vpMatchedMPs
     *              TODO
     */
    bool DetectCommonRegionsFromLastKF(KeyFrame                *pCurrentKF,
                                       KeyFrame                *pMatchedKF,
                                       g2o::Sim3               &gScw,
                                       int                     &nNumProjMatches,
                                       std::vector<MapPoint *> &vpMPs,
                                       std::vector<MapPoint *> &vpMatchedMPs);

    /*!
     * @brief       TODO
     *
     * @param[in]   pCurrentKF
     *              TODO
     *
     * @param[in]   pMatchedKF
     *              TODO
     *
     * @param[in]   g2oScw
     *              TODO
     *
     * @param[in]   spMatchedMPinOrigin
     *              TODO
     *
     * @param[in]   vpMapPoints
     *              TODO
     *
     * @param[in]   vpMatchedMapPoints
     *              TODO
     */
    int FindMatchesByProjection(KeyFrame           *pCurrentKF,
                                KeyFrame           *pMatchedKFw,
                                g2o::Sim3          &g2oScw,
                                set<MapPoint *>    &spMatchedMPinOrigin,
                                vector<MapPoint *> &vpMapPoints,
                                vector<MapPoint *> &vpMatchedMapPoints);

    /*!
     * @brief       TODO
     *
     * @param[in]   CorrectedPosesMap
     *              TODO
     *
     * @param[in]   vpMapPoints
     *              TODO
     */
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                       vector<MapPoint *>    &vpMapPoints);

    /*!
     * @brief       TODO
     *
     * @param[in]   vConectedKFs
     *              TODO
     *
     * @param[in]   vpMapPoints
     *              TODO
     */
    void SearchAndFuse(const vector<KeyFrame *> &vConectedKFs,
                       vector<MapPoint *>       &vpMapPoints);

    /*!
     * @brief       TODO
     */
    void CorrectLoop(void);

    /*!
     * @brief       Stops and joins the owned global bundle-adjustment worker.
     *
     * @return      True when an active optimization was interrupted; false when
     *              the method only reclaimed an already-completed worker.
     */
    bool stopGlobalBundleAdjustment(void);

    /*!
     * @brief       TODO
     *
     * @param[in]   accepted_in
     *              TODO
     *
     * @param[in]   reason_in
     *              TODO
     */
    void recordLoopCorrectionEvent(bool               accepted_in,
                                   const std::string &reason_in);
    /*!
     * @brief       Starts a replacement GBA worker after an interrupted merge
     *              attempt.
     *
     * @param[in]   accepted_in
     *              TODO
     *
     * @param[in]   reason_in
     *              TODO
     */
    void relaunchGlobalBundleAdjustment(Map *p_activeMap_in);

    /*!
     * @brief       TODO
     */
    void MergeLocal(void);

    /*!
     * @brief       TODO
     */
    void MergeLocalInertial(void);

    /*!
     * @brief       TODO
     *
     * @param[in]   spKFsMap1
     *              TODO
     *
     * @param[in]   spKFsMap2
     *              TODO
     */
    void CheckObservations(set<KeyFrame *> &spKFsMap1,
                           set<KeyFrame *> &spKFsMap2);

    /*!
     * @brief       TODO
     */
    void ResetIfRequested(void);

    /*!
     * @brief      TODO
     */
    bool CheckFinish(void);

    /*!
     * @brief      TODO
     */
    void SetFinish(void);
#ifdef REGISTER_LOOP
    string mstrFolderLoop;
#endif
};

} // namespace ORB_SLAM3

#endif
