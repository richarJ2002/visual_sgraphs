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
#include "LoopClosingStatus.h"
#include "ORBVocabulary.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

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
namespace types
{
class SystemParams;
}

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;
namespace semantic
{
enum class SemanticMergeDecision;
}

/*!
 * @brief        Confirms both maps have observed floors with matching
 *               plane identity before a loop-merge is allowed to
 *               proceed.
 *
 *               Shared by the legacy place-recognition merge path
 *               (LoopClosing.cc call sites) and the plane-gated
 *               semantic verifier (SemanticVerify::runFloorGate),
 *               which is why this declaration lives here rather
 *               than staying local to LoopClosing.cc's anonymous
 *               namespace.
 *
 * @param[in]    p_survivingMap_in
 *               The map that remains active after the merge.
 *
 * @param[in]    p_absorbedMap_in
 *               The map being merged into the surviving map.
 *
 * @param[in]    transform_absorbedWorldToSurvivingWorld_in
 *               Verified Sim3 transform from the absorbed map's
 *               world frame to the surviving map's world frame.
 *
 * @param[out]   result_out
 *               One of "ACCEPTED", "REJECTED", or "DEFERRED".
 *
 * @param[out] isVerified_out True only when both floors are observed and their
 * plane identities match within Floor's merge thresholds.
 * @return LOOP_CLOSING_STATUS_SUCCESS.
 */
[[nodiscard]] LoopClosingStatus verifyLoopMergeFloors(
    Map             *p_survivingMap_in,
    Map             *p_absorbedMap_in,
    const g2o::Sim3 &transform_absorbedWorldToSurvivingWorld_in,
    std::string     &result_out,
    bool            &isVerified_out);

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
        bool          wasLastAccepted{false};
        unsigned long lastMapId{0U};
        unsigned long lastCurrentKeyFrameId{0U};
        unsigned long lastMatchedKeyFrameId{0U};
        double        lastCurrentTimestamp{0.0};
        double        lastMatchedTimestamp{0.0};
        std::string   lastReason;
    };

    typedef std::pair<std::set<KeyFrame *>, int> ConsistentGroup;

    typedef std::map<
        KeyFrame *,
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
    LoopClosing(Atlas            *p_atlas_in,
                KeyFrameDatabase *p_database_in,
                ORBVocabulary    *p_vocabulary_in,
                const bool        isScaleFixed_in,
                const bool        isActiveLc_in);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_tracker_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus setTracker(Tracking *p_tracker_in);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_localMapper_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        setLocalMapper(LocalMapping *p_localMapper_in);

    /*!
     * @brief       TODO
     *
     * @param[in]   mergeStatus_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus setMergeStatus(bool mergeStatus_in);

    /*!
     * @brief       TODO
     */
    void run(void);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_keyFrame_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus insertKeyFrame(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus requestReset();

    /*!
     * @brief       TODO
     *
     * @param[in]   p_map_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus requestResetActiveMap(Map *p_map_in);

    /*!
     * @brief       TODO
     *
     * @note        This function will run in a separate thread
     *
     * @param[in,out] p_activeMap_inout
     *              TODO
     *
     * @param[in]   loopKeyFrameCount_in
     *              TODO
     *
     * @param[in]   generation_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        runGlobalBundleAdjustment(Map          *p_activeMap_inout,
                                  unsigned long loopKeyFrameCount_in,
                                  unsigned int  generation_in);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus isRunningGBA(bool &isRunningGBA_out)
    {
        /* Lock mutext */
        std::unique_lock<std::mutex> lock(gbaMutex);

        /* Return flag to indicate if global bundal adjustemnt is running */
        isRunningGBA_out = isGbaRunning;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus isFinishedGBA(bool &isFinishedGBA_out)
    {
        /* Lock mutext */
        std::unique_lock<std::mutex> lock(gbaMutex);

        /* Return flag to indicate if global bundal adjustemnt is finished */
        isFinishedGBA_out = hasGbaFinished;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus requestFinish(void);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus isFinished(bool &isFinished_out);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus
        isMergeInProgress(bool &isMergeInProgress_out);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus getLoopCorrectionStatus(
        LoopClosing::LoopCorrectionStatus &getLoopCorrectionStatus_out) const;

    Viewer *p_viewer;

#ifdef REGISTER_TIMES

    std::vector<double> dataQueryTimes_ms;
    std::vector<double> sim3EstimationTimes_ms;
    std::vector<double> placeRecognitionTotalTimes_ms;

    std::vector<double> mergeMapsTimes_ms;
    std::vector<double> weldingBaTimes_ms;
    std::vector<double> mergeEssentialGraphTimes_ms;
    std::vector<double> mergeTotalTimes_ms;
    std::vector<int>    mergeKeyFrameCounts;
    std::vector<int>    mergeMapPointCounts;
    int                 mergeCount;

    std::vector<double> loopFusionTimes_ms;
    std::vector<double> loopEssentialGraphTimes_ms;
    std::vector<double> loopTotalTimes_ms;
    std::vector<int>    loopKeyFrameCounts;
    int                 loopCount;

    std::vector<double> gbaTimes_ms;
    std::vector<double> updateMapTimes_ms;
    std::vector<double> fullGbaTotalTimes_ms;
    std::vector<int>    gbaKeyFrameCounts;
    std::vector<int>    gbaMapPointCounts;
    int                 fullGbaExecutionCount;
    int                 fullGbaAbortCount;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    /* ---------------------------------------------------------------------- *
     * PROTECTED MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief      TODO
     */
    bool isResetRequested;

    /*!
     * @brief      TODO
     */
    bool isResetActiveMapRequested;

    /*!
     * @brief      TODO
     */
    Map *p_mapToReset;

    /*!
     * @brief      TODO
     */
    std::mutex resetMutex;

    /*!
     * @brief      TODO
     */
    bool isFinishRequested;

    /*!
     * @brief      TODO
     */
    bool hasFinished;

    /*!
     * @brief      TODO
     */
    std::mutex finishMutex;

    /*!
     * @brief      TODO
     */
    Atlas *p_atlas;

    /*!
     * @brief      TODO
     */
    Tracking *p_tracker;

    /*!
     * @brief      TODO
     */
    KeyFrameDatabase *p_keyFrameDatabase;

    /*!
     * @brief      TODO
     */
    ORBVocabulary *p_orbVocabulary;

    /*!
     * @brief      TODO
     */
    LocalMapping *p_localMapper;

    /*!
     * @brief      TODO
     */
    std::list<KeyFrame *> loopKeyFrameQueue;

    /*!
     * @brief      TODO
     */
    std::mutex loopQueueMutex;

    /*!
     * @brief      Loop detector parameters
     */
    float covisibilityConsistencyThreshold;

    /*!
     * @brief      TODO
     */
    KeyFrame *p_currentKF;

    /*!
     * @brief      TODO
     */
    KeyFrame *p_lastCurrentKF;

    /*!
     * @brief      TODO
     */
    KeyFrame *p_matchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<ConsistentGroup> consistentGroups;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> enoughConsistentCandidates;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> currentConnectedKFs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> currentMatchedPoints;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> loopMapPoints;

    /*!
     * @brief      TODO
     */
    cv::Mat correctedPose;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 mg2oScw;

    /*!
     * @brief      TODO
     */
    Map *p_lastMap;

    /*!
     * @brief      TODO
     */
    bool isLoopDetected;

    /*!
     * @brief      TODO
     */
    int loopNumCoincidences;

    /*!
     * @brief      TODO
     */
    int loopNumNotFound;

    /*!
     * @brief      TODO
     */
    KeyFrame *p_loopLastCurrentKF;

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
    KeyFrame *p_loopMatchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> loopMPs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> loopMatchedMPs;

    /*!
     * @brief      TODO
     */
    bool isMergeDetected;

    /*!
     * @brief      TODO
     */
    std::atomic_bool hasMergeInProgress;

    /*!
     * @brief      TODO
     */
    int mergeNumCoincidences;

    /*!
     * @brief      TODO
     */
    int mergeNumNotFound;

    /*!
     * @brief      TODO
     */
    KeyFrame *p_mergeLastCurrentKF;

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
    KeyFrame *p_mergeMatchedKF;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mergeMPs;

    /*!
     * @brief      TODO
     */
    std::vector<MapPoint *> mergeMatchedMPs;

    /*!
     * @brief      TODO
     */
    std::vector<KeyFrame *> mergeConnectedKFs;

    /*!
     * @brief      TODO
     */
    g2o::Sim3 oldCorrectedPose;

    /*!
     * @brief      TODO
     */
    long unsigned int lastLoopKeyFrameId;

    /*!
     * @brief      TODO
     */
    bool isGbaRunning;

    /*!
     * @brief      TODO
     */
    bool hasGbaFinished;

    /*!
     * @brief      TODO
     */
    std::mutex gbaMutex;

    /*!
     * @brief      TODO
     */
    std::thread *p_threadGBA;

    /*!
     * @brief      TODO
     */
    std::atomic_bool isGlobalBundleAdjustmentStopRequested{false};

    /*!
     * @brief      TODO
     *
     * @note        Fix scale in the stereo/RGB-D case
     */
    bool isScaleFixed;

    /*!
     * @brief      TODO
     */
    unsigned int fullBundleAdjustmentIndex;

    /*!
     * @brief      TODO
     */
    std::vector<double> placeRecognitionCurrentTimes;

    /*!
     * @brief      TODO
     */
    std::vector<double> placeRecognitionMatchedTimes;

    /*!
     * @brief      TODO
     */
    std::vector<int> placeRecognitionTypes;

    /*!
     * @brief      TODO
     */
    std::string mstrFolderSubTraj;

    /*!
     * @brief      TODO
     */
    int numCorrection;

    /*!
     * @brief      TODO
     */
    int correctionGBA;

    /*!
     * @brief      TODO
     */
    mutable std::mutex loopCorrectionStatusMutex;

    /*!
     * @brief      TODO
     */
    LoopCorrectionStatus loopCorrectionStatus;

    /*!
     * @brief      TODO
     */
    bool isLoopClosingActive = true;

    /*!
     * @brief      TODO
     */
    types::SystemParams *p_sysParams;

    /* ---------------------------------------------------------------------- *
     * PROTECTED METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus
        checkNewKeyFrames(bool &hasNewKeyFrames_out);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus
        newDetectCommonRegions(bool &isDetected_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_currentKeyFrame_in
     *              TODO
     *
     * @param[in]   p_matchedKeyFrame_in
     *              TODO
     *
     * @param[in,out] gScw_inout
     *              TODO
     *
     * @param[out]  countProjectionMatchCount_out
     *              TODO
     *
     * @param[in,out] mapPoints_inout
     *              TODO
     *
     * @param[in,out] matchedMapPoints_inout
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus detectAndReffineSim3FromLastKF(
        KeyFrame                *p_currentKeyFrame_in,
        KeyFrame                *p_matchedKeyFrame_in,
        g2o::Sim3               &gScw_inout,
        int                     &countProjectionMatchCount_out,
        std::vector<MapPoint *> &mapPoints_inout,
        std::vector<MapPoint *> &matchedMapPoints_inout,
        bool                    &isDetected_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   bowCandidates_in
     *              TODO
     *
     * @param[out]  matchedKeyFrame_out
     *              TODO
     *
     * @param[out]  lastCurrentKeyFrame_out
     *              TODO
     *
     * @param[in]   nNumProjMatches
     *              TODO
     *
     * @param[out]  g2oScw_out
     *              TODO
     *
     * @param[out]  countCoincidenceCount_out
     *              TODO
     *
     * @param[out]  mapPoints_out
     *              TODO
     *
     * @param[out]  matchedMapPoints_out
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus detectCommonRegionsFromBoW(
        std::vector<KeyFrame *> &bowCandidates_in,
        KeyFrame               *&matchedKeyFrame_out,
        KeyFrame               *&lastCurrentKeyFrame_out,
        g2o::Sim3               &g2oScw_out,
        int                     &countCoincidenceCount_out,
        std::vector<MapPoint *> &mapPoints_out,
        std::vector<MapPoint *> &matchedMapPoints_out,
        bool                    &isDetected_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_currentKeyFrame_in
     *              TODO
     *
     * @param[in]   p_matchedKeyFrame_in
     *              TODO
     *
     * @param[in,out] gScw_inout
     *              TODO
     *
     * @param[out]  countProjectionMatchCount_out
     *              TODO
     *
     * @param[in,out] mapPoints_inout
     *              TODO
     *
     * @param[in,out] matchedMapPoints_inout
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus detectCommonRegionsFromLastKF(
        KeyFrame                *p_currentKeyFrame_in,
        KeyFrame                *p_matchedKeyFrame_in,
        g2o::Sim3               &gScw_inout,
        int                     &countProjectionMatchCount_out,
        std::vector<MapPoint *> &mapPoints_inout,
        std::vector<MapPoint *> &matchedMapPoints_inout,
        bool                    &isDetected_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   p_currentKeyFrame_in
     *              TODO
     *
     * @param[in]   pMatchedKF
     *              TODO
     *
     * @param[in]   g2oScw_in
     *              TODO
     *
     * @param[in]   matchedMPinOrigins_in
     *              TODO
     *
     * @param[out]  mapPoints_out
     *              TODO
     *
     * @param[out]  matchedMapPoints_out
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        findMatchesByProjection(KeyFrame                *p_currentKeyFrame_in,
                                KeyFrame                *p_matchedKFw_in,
                                g2o::Sim3               &g2oScw_in,
                                std::set<MapPoint *>    &matchedMPinOrigins_in,
                                std::vector<MapPoint *> &mapPoints_out,
                                std::vector<MapPoint *> &matchedMapPoints_out,
                                int                     &matches_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   correctedPosesMap_in
     *              TODO
     *
     * @param[in]   mapPoints_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        searchAndFuse(const KeyFrameAndPose   &correctedPosesMap_in,
                      std::vector<MapPoint *> &mapPoints_in);

    /*!
     * @brief       TODO
     *
     * @param[in]   conectedKeyFrames_in
     *              TODO
     *
     * @param[in]   mapPoints_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        searchAndFuse(const std::vector<KeyFrame *> &conectedKeyFrames_in,
                      std::vector<MapPoint *>       &mapPoints_in);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus correctLoop(void);

    /*!
     * @brief       Stops and joins the owned global bundle-adjustment worker.
     *
     * @param[out] wasRunning_out True when an active optimization was
     * interrupted; false when the method only reclaimed an already-completed
     * worker.
     * @return LOOP_CLOSING_STATUS_SUCCESS.
     */
    [[nodiscard]] LoopClosingStatus
        stopGlobalBundleAdjustment(bool &wasRunning_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   accepted_in
     *              TODO
     *
     * @param[in]   reason_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        recordLoopCorrectionEvent(bool               accepted_in,
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
    [[nodiscard]] LoopClosingStatus
        relaunchGlobalBundleAdjustment(Map *p_activeMap_inout);

    /*!
     * @brief       TODO
     */
    /*!
     * @brief       Attempts a visual map merge after all preconditions pass.
     *
     * @param[out] local_out ACCEPT only after the merge is committed, DEFER
     * when semantic evidence is incomplete and the candidate remains retryable,
     * or REJECT when the attempt is invalid.
     * @return LOOP_CLOSING_STATUS_SUCCESS.
     */
    [[nodiscard]] LoopClosingStatus
        mergeLocal(semantic::SemanticMergeDecision &local_out);

    /*!
     * @brief       TODO
     */
    /*!
     * @brief       Attempts an inertial map merge after all preconditions pass.
     *
     * @param[out] localInertial_out ACCEPT only after the merge is committed,
     * DEFER when semantic evidence is incomplete and the candidate remains
     * retryable, or REJECT when the attempt is invalid.
     * @return LOOP_CLOSING_STATUS_SUCCESS.
     */
    [[nodiscard]] LoopClosingStatus
        mergeLocalInertial(semantic::SemanticMergeDecision &localInertial_out);

    /*!
     * @brief       TODO
     *
     * @param[in]   keyFramesMap1_in
     *              TODO
     *
     * @param[in]   keyFramesMap2_in
     *              TODO
     */
    [[nodiscard]] LoopClosingStatus
        checkObservations(std::set<KeyFrame *> &keyFramesMap1_in,
                          std::set<KeyFrame *> &keyFramesMap2_in);

    /*!
     * @brief       TODO
     */
    [[nodiscard]] LoopClosingStatus resetIfRequested(void);

    /*!
     * @brief      TODO
     */
    [[nodiscard]] LoopClosingStatus checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief      TODO
     */
    [[nodiscard]] LoopClosingStatus setFinish(void);
#ifdef REGISTER_LOOP
    std::string mstrFolderLoop;
#endif
};

} // namespace core
} // namespace vs_graphs

#endif
