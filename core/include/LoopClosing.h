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
 * @file            LoopClosing.h
 *
 * @brief           Declares LoopClosing, the thread that recognises revisited
 *                  places, closes loops and merges maps.
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
     * @brief       Creates the loop closer. Connect it with setTracker() and
     *              setLocalMapper() before starting run() on its own thread.
     *
     * @param[in]   p_atlas_in
     *              Atlas holding every map; non-owning, shall outlive the loop
     *              closer.
     *
     * @param[in]   p_database_in
     *              Key-frame database used for place recognition; non-owning,
     *              shall outlive the loop closer.
     *
     * @param[in]   p_vocabulary_in
     *              ORB vocabulary; non-owning, shall outlive the loop closer.
     *
     * @param[in]   isScaleFixed_in
     *              True when the map scale is known (stereo, RGB-D or
     *              inertial), so optimisations keep the scale fixed.
     *
     * @param[in]   isActiveLc_in
     *              False switches loop closing off: key frames are queued but
     *              no loop or merge is detected.
     */
    LoopClosing(Atlas            *p_atlas_in,
                KeyFrameDatabase *p_database_in,
                ORBVocabulary    *p_vocabulary_in,
                const bool        isScaleFixed_in,
                const bool        isActiveLc_in);

    /*!
     * @brief       Connects the tracking thread, which is told about loop
     *              corrections.
     *
     * @param[in]   p_tracker_in
     *              Tracking thread; non-owning, shall outlive the loop closer.
     */
    [[nodiscard]] LoopClosingStatus setTracker(Tracking *p_tracker_in);

    /*!
     * @brief       Connects the local-mapping thread, which is paused while a
     *              loop or merge is corrected.
     *
     * @param[in]   p_localMapper_in
     *              Local-mapping thread; non-owning, shall outlive the loop
     *              closer.
     */
    [[nodiscard]] LoopClosingStatus
        setLocalMapper(LocalMapping *p_localMapper_in);

    /*!
     * @brief       Marks whether a map merge is in progress.
     *
     * @param[in]   mergeStatus_in
     *              True while a merge is being carried out.
     */
    [[nodiscard]] LoopClosingStatus setMergeStatus(bool mergeStatus_in);

    /*!
     * @brief       Main loop of the loop-closing thread: takes queued key
     *              frames, looks for loops and map merges, and corrects them,
     *              until requestFinish() is called.
     */
    void run(void);

    /*!
     * @brief       Queues a key frame to be checked for loops and merges. The
     *              very first key frame (id 0) is ignored.
     *
     * @param[in]   p_keyFrame_in
     *              Key frame to check; non-owning.
     */
    [[nodiscard]] LoopClosingStatus insertKeyFrame(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       Asks the thread to reset and waits, polling every 5 ms,
     *              until it has done so.
     */
    [[nodiscard]] LoopClosingStatus requestReset();

    /*!
     * @brief       Asks the thread to reset its state for one map.
     *
     * @param[in]   p_map_in
     *              Map being reset; non-owning.
     */
    [[nodiscard]] LoopClosingStatus requestResetActiveMap(Map *p_map_in);

    /*!
     * @brief       Optimises the whole map after a loop closure, then applies
     *              the correction to every key frame and map point.
     *
     * @note        Runs on its own thread (p_threadGBA).
     *
     * @param[in,out] p_activeMap_inout
     *              Map to optimise; updated in place.
     *
     * @param[in]   loopKeyFrameCount_in
     *              Id of the key frame that closed the loop.
     *
     * @param[in]   generation_in
     *              Value of fullBundleAdjustmentIndex when the run was
     *              started; the run gives up once a newer run has been
     *              requested.
     */
    [[nodiscard]] LoopClosingStatus
        runGlobalBundleAdjustment(Map          *p_activeMap_inout,
                                  unsigned long loopKeyFrameCount_in,
                                  unsigned int  generation_in);

    /*!
     * @brief       Tells whether a global bundle adjustment is running.
     *
     * @param[out]  isRunningGBA_out
     *              True while one runs.
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
     * @brief       Tells whether the last global bundle adjustment has
     *              finished.
     *
     * @param[out]  isFinishedGBA_out
     *              True once it has finished.
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
     * @brief       Asks the thread to stop after its current iteration.
     */
    [[nodiscard]] LoopClosingStatus requestFinish(void);

    /*!
     * @brief       Tells whether the thread has stopped.
     *
     * @param[out]  isFinished_out
     *              True once it has stopped.
     */
    [[nodiscard]] LoopClosingStatus isFinished(bool &isFinished_out);

    /*!
     * @brief       Tells whether a map merge is being carried out.
     *
     * @param[out]  isMergeInProgress_out
     *              True while a merge runs.
     */
    [[nodiscard]] LoopClosingStatus
        isMergeInProgress(bool &isMergeInProgress_out);

    /*!
     * @brief       Copies the loop-correction counters and the last outcome,
     *              for health reporting.
     *
     * @param[out]  getLoopCorrectionStatus_out
     *              Copy of the status.
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
     * @brief       True while a full reset is requested; guarded by
     *              resetMutex.
     */
    bool isResetRequested;

    /*!
     * @brief       True while a reset of one map is requested; guarded by
     *              resetMutex.
     */
    bool isResetActiveMapRequested;

    /*!
     * @brief       Map whose reset is requested; non-owning, guarded by
     *              resetMutex.
     */
    Map *p_mapToReset;

    /*!
     * @brief       Guards the reset requests and p_mapToReset.
     */
    std::mutex resetMutex;

    /*!
     * @brief       True once the thread has been asked to stop; guarded by
     *              finishMutex.
     */
    bool isFinishRequested;

    /*!
     * @brief       True once the thread has stopped; guarded by finishMutex.
     */
    bool hasFinished;

    /*!
     * @brief       Guards isFinishRequested and hasFinished.
     */
    std::mutex finishMutex;

    /*!
     * @brief       Atlas holding every map; non-owning.
     */
    Atlas *p_atlas;

    /*!
     * @brief       Tracking thread, told about loop corrections; non-owning.
     */
    Tracking *p_tracker;

    /*!
     * @brief       Key-frame database used for place recognition; non-owning.
     */
    KeyFrameDatabase *p_keyFrameDatabase;

    /*!
     * @brief       ORB vocabulary; non-owning.
     */
    ORBVocabulary *p_orbVocabulary;

    /*!
     * @brief       Local-mapping thread, paused during corrections; non-
     *              owning.
     */
    LocalMapping *p_localMapper;

    /*!
     * @brief       Key frames waiting to be checked; guarded by
     *              loopQueueMutex.
     */
    std::list<KeyFrame *> loopKeyFrameQueue;

    /*!
     * @brief       Guards loopKeyFrameQueue.
     */
    std::mutex loopQueueMutex;

    /*!
     * @brief       Key frame being checked now.
     */
    KeyFrame *p_currentKF;

    /*!
     * @brief       Key frame checked in the previous iteration.
     */
    KeyFrame *p_lastCurrentKF;

    /*!
     * @brief       Key frames connected to the current key frame in the
     *              covisibility graph.
     */
    std::vector<KeyFrame *> currentConnectedKFs;

    /*!
     * @brief       Map points around the loop candidate, fused into the map
     *              when the loop is corrected.
     */
    std::vector<MapPoint *> loopMapPoints;

    /*!
     * @brief       Map that the previously checked key frame belonged to.
     */
    Map *p_lastMap;

    /*!
     * @brief       True when a loop is confirmed and waits to be corrected.
     */
    bool isLoopDetected;

    /*!
     * @brief       Consecutive key frames that confirmed the current loop
     *              candidate.
     */
    int loopNumCoincidences;

    /*!
     * @brief       Consecutive key frames that failed to confirm the current
     *              loop candidate.
     */
    int loopNumNotFound;

    /*!
     * @brief       Last key frame that confirmed the loop candidate.
     */
    KeyFrame *p_loopLastCurrentKF;

    /*!
     * @brief       Similarity transform estimated against the loop candidate
     *              for p_loopLastCurrentKF.
     *
     * @frame       World to the camera of p_loopLastCurrentKF.
     */
    g2o::Sim3 mg2oLoopSlw;

    /*!
     * @brief       Similarity transform used to close the loop.
     *
     * @frame       World to current key-frame camera (similarity: rotation,
     *              translation, scale).
     */
    g2o::Sim3 mg2oLoopScw;

    /*!
     * @brief       Earlier key frame recognised as the same place (the loop
     *              candidate).
     */
    KeyFrame *p_loopMatchedKF;

    /*!
     * @brief       Map points around the loop candidate.
     */
    std::vector<MapPoint *> loopMPs;

    /*!
     * @brief       Matched loop map point for each key point of the current
     *              key frame, or null.
     */
    std::vector<MapPoint *> loopMatchedMPs;

    /*!
     * @brief       True when a map merge is confirmed and waits to be carried
     *              out.
     */
    bool isMergeDetected;

    /*!
     * @brief       True while a map merge is being carried out; read by other
     *              threads.
     */
    std::atomic_bool hasMergeInProgress;

    /*!
     * @brief       Consecutive key frames that confirmed the current merge
     *              candidate.
     */
    int mergeNumCoincidences;

    /*!
     * @brief       Consecutive key frames that failed to confirm the current
     *              merge candidate.
     */
    int mergeNumNotFound;

    /*!
     * @brief       Last key frame that confirmed the merge candidate.
     */
    KeyFrame *p_mergeLastCurrentKF;

    /*!
     * @brief       Similarity transform estimated against the merge candidate
     *              for p_mergeLastCurrentKF.
     *
     * @frame       World to the camera of p_mergeLastCurrentKF.
     */
    g2o::Sim3 mg2oMergeSlw;

    /*!
     * @brief       Similarity transform used to carry out the merge.
     *
     * @frame       World to current key-frame camera (similarity: rotation,
     *              translation, scale).
     */
    g2o::Sim3 mg2oMergeScw;

    /*!
     * @brief       Key frame of another map recognised as the same place (the
     *              merge candidate).
     */
    KeyFrame *p_mergeMatchedKF;

    /*!
     * @brief       Map points around the merge candidate.
     */
    std::vector<MapPoint *> mergeMPs;

    /*!
     * @brief       Matched merge map point for each key point of the current
     *              key frame, or null.
     */
    std::vector<MapPoint *> mergeMatchedMPs;

    /*!
     * @brief       Key frames connected to the merge candidate in the
     *              covisibility graph.
     */
    std::vector<KeyFrame *> mergeConnectedKFs;

    /*!
     * @brief       Similarity transform between the two maps of a confirmed
     *              merge.
     *
     * @frame       Current map's world to the matched map's world.
     */
    g2o::Sim3 oldCorrectedPose;

    /*!
     * @brief       Id of the key frame at which the last loop was closed.
     */
    long unsigned int lastLoopKeyFrameId;

    /*!
     * @brief       True while a global bundle adjustment runs; guarded by
     *              gbaMutex.
     */
    bool isGbaRunning;

    /*!
     * @brief       True once the last global bundle adjustment has finished;
     *              guarded by gbaMutex.
     */
    bool hasGbaFinished;

    /*!
     * @brief       Guards the global bundle-adjustment flags and p_threadGBA.
     */
    std::mutex gbaMutex;

    /*!
     * @brief       Thread running the global bundle adjustment; owned, joined
     *              by stopGlobalBundleAdjustment().
     */
    std::thread *p_threadGBA;

    /*!
     * @brief       Asks the running global bundle adjustment to stop early.
     */
    std::atomic_bool isGlobalBundleAdjustmentStopRequested{false};

    /*!
     * @brief       True when the map scale is known, so optimisations keep it
     *              fixed.
     *
     * @note        Fix scale in the stereo/RGB-D case
     */
    bool isScaleFixed;

    /*!
     * @brief       Generation of the global bundle adjustment; raised when a
     *              run is stopped, so a superseded run can tell and give up.
     */
    unsigned int fullBundleAdjustmentIndex;

    /*!
     * @brief       Time stamp of the current key frame at each recognised
     *              place.
     *
     * @units       seconds
     */
    std::vector<double> placeRecognitionCurrentTimes;

    /*!
     * @brief       Time stamp of the matched key frame at each recognised
     *              place.
     *
     * @units       seconds
     */
    std::vector<double> placeRecognitionMatchedTimes;

    /*!
     * @brief       Kind of each recognised place: 0 = loop closure, 1 = map
     *              merge.
     */
    std::vector<int> placeRecognitionTypes;

    /*!
     * @brief       Number of loop corrections so far.
     */
    int numCorrection;

    /*!
     * @brief       Value of numCorrection when the last global bundle
     *              adjustment started.
     */
    int correctionGBA;

    /*!
     * @brief       Guards loopCorrectionStatus.
     */
    mutable std::mutex loopCorrectionStatusMutex;

    /*!
     * @brief       Counters and last outcome of the loop-correction attempts,
     *              for health reporting; guarded by loopCorrectionStatusMutex.
     */
    LoopCorrectionStatus loopCorrectionStatus;

    /*!
     * @brief       False when loop closing is switched off; the thread then
     *              detects nothing.
     */
    bool isLoopClosingActive = true;

    /*!
     * @brief       System parameters; non-owning.
     */
    types::SystemParams *p_sysParams;

    /* ---------------------------------------------------------------------- *
     * PROTECTED METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Tells whether key frames are waiting in the queue.
     *
     * @param[out]  hasNewKeyFrames_out
     *              True when at least one is waiting.
     */
    [[nodiscard]] LoopClosingStatus
        checkNewKeyFrames(bool &hasNewKeyFrames_out);

    /*!
     * @brief       Looks for a loop or merge for the current key frame: first
     *              re-checks the last candidate, then searches the key-frame
     *              database by bag of words.
     *
     * @param[out]  isDetected_out
     *              True when a loop or merge has been confirmed.
     */
    [[nodiscard]] LoopClosingStatus
        newDetectCommonRegions(bool &isDetected_out);

    /*!
     * @brief       Refines the similarity transform to a candidate key frame
     *              and counts the map points that match by projection.
     *
     * @param[in]   p_currentKeyFrame_in
     *              Key frame being checked.
     *
     * @param[in]   p_matchedKeyFrame_in
     *              Candidate key frame.
     *
     * @param[in,out] gScw_inout
     *              World to current key-frame camera (similarity: rotation,
     *              translation, scale). Refined in place.
     *
     * @param[out]  countProjectionMatchCount_out
     *              Number of map points matched by projection.
     *
     * @param[in,out] mapPoints_inout
     *              Map points around the candidate.
     *
     * @param[in,out] matchedMapPoints_inout
     *              Matched map point for each key point of the current key
     *              frame, or null.
     *
     * @param[out]  isDetected_out
     *              True when enough map points match.
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
     * @brief       Searches the bag-of-words candidates for a place the
     *              current key frame has seen before and estimates the
     *              similarity transform to it.
     *
     * @param[in]   bowCandidates_in
     *              Candidate key frames from the key-frame database.
     *
     * @param[out]  matchedKeyFrame_out
     *              Best candidate key frame.
     *
     * @param[out]  lastCurrentKeyFrame_out
     *              Key frame that confirmed the candidate.
     *
     * @param[out]  g2oScw_out
     *              World to current key-frame camera (similarity: rotation,
     *              translation, scale).
     *
     * @param[out]  countCoincidenceCount_out
     *              Number of key frames that confirmed the candidate.
     *
     * @param[out]  mapPoints_out
     *              Map points around the candidate.
     *
     * @param[out]  matchedMapPoints_out
     *              Matched map point for each key point of the current key
     *              frame, or null.
     *
     * @param[out]  isDetected_out
     *              True when a candidate was found.
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
     * @brief       Checks whether the current key frame still matches the last
     *              loop or merge candidate.
     *
     * @param[in]   p_currentKeyFrame_in
     *              Key frame being checked.
     *
     * @param[in]   p_matchedKeyFrame_in
     *              Last candidate key frame.
     *
     * @param[in,out] gScw_inout
     *              World to current key-frame camera (similarity: rotation,
     *              translation, scale). Refined in place.
     *
     * @param[out]  countProjectionMatchCount_out
     *              Number of map points matched by projection.
     *
     * @param[in,out] mapPoints_inout
     *              Map points around the candidate.
     *
     * @param[in,out] matchedMapPoints_inout
     *              Matched map point for each key point of the current key
     *              frame, or null.
     *
     * @param[out]  isDetected_out
     *              True when the candidate is confirmed again.
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
     * @brief       Projects the map points around a candidate key frame into
     *              the current key frame and matches them.
     *
     * @param[in]   p_currentKeyFrame_in
     *              Key frame being checked.
     *
     * @param[in]   p_matchedKFw_in
     *              Candidate key frame.
     *
     * @param[in]   g2oScw_in
     *              World to current key-frame camera (similarity: rotation,
     *              translation, scale).
     *
     * @param[out]  mapPoints_out
     *              Map points around the candidate.
     *
     * @param[out]  matchedMapPoints_out
     *              Matched map point for each key point of the current key
     *              frame, or null.
     *
     * @param[out]  matches_out
     *              Number of matches.
     */
    [[nodiscard]] LoopClosingStatus
        findMatchesByProjection(KeyFrame                *p_currentKeyFrame_in,
                                KeyFrame                *p_matchedKFw_in,
                                g2o::Sim3               &g2oScw_in,
                                std::vector<MapPoint *> &mapPoints_out,
                                std::vector<MapPoint *> &matchedMapPoints_out,
                                int                     &matches_out);

    /*!
     * @brief       Projects map points into each corrected key frame and fuses
     *              the duplicates.
     *
     * @param[in]   correctedPosesMap_in
     *              Corrected similarity transform (world to camera) of each
     *              key frame.
     *
     * @param[in]   mapPoints_in
     *              Map points to fuse.
     */
    [[nodiscard]] LoopClosingStatus
        searchAndFuse(const KeyFrameAndPose   &correctedPosesMap_in,
                      std::vector<MapPoint *> &mapPoints_in);

    /*!
     * @brief       Projects map points into the given key frames and fuses the
     *              duplicates (used by map merges).
     *
     * @param[in]   conectedKeyFrames_in
     *              Key frames to fuse into.
     *
     * @param[in]   mapPoints_in
     *              Map points to fuse.
     */
    [[nodiscard]] LoopClosingStatus
        searchAndFuse(const std::vector<KeyFrame *> &conectedKeyFrames_in,
                      std::vector<MapPoint *>       &mapPoints_in);

    /*!
     * @brief       Closes the confirmed loop: corrects the poses around the
     *              current key frame, fuses duplicate map points, optimises
     *              the essential graph and starts a global bundle adjustment.
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
     * @brief       Records the outcome of one loop-correction attempt in
     *              loopCorrectionStatus.
     *
     * @param[in]   accepted_in
     *              True when the correction was applied.
     *
     * @param[in]   reason_in
     *              Why it was accepted or rejected.
     */
    [[nodiscard]] LoopClosingStatus
        recordLoopCorrectionEvent(bool               accepted_in,
                                  const std::string &reason_in);
    /*!
     * @brief       Starts a replacement global bundle adjustment after an
     *              interrupted merge attempt.
     *
     * @param[in,out] p_activeMap_inout
     *              Map to optimise.
     */
    [[nodiscard]] LoopClosingStatus
        relaunchGlobalBundleAdjustment(Map *p_activeMap_inout);

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
     * @brief       Counts the map points that the key frames of the first set
     *              share with the second set, as a merge-debugging aid.
     *
     * @param[in]   keyFramesMap1_in
     *              Key frames of the first map.
     *
     * @param[in]   keyFramesMap2_in
     *              Key frames of the second map.
     */
    [[nodiscard]] LoopClosingStatus
        checkObservations(std::set<KeyFrame *> &keyFramesMap1_in,
                          std::set<KeyFrame *> &keyFramesMap2_in);

    /*!
     * @brief       Carries out a pending reset request, if any.
     */
    [[nodiscard]] LoopClosingStatus resetIfRequested(void);

    /*!
     * @brief       Tells whether the thread has been asked to stop.
     *
     * @param[out]  isFinishRequested_out
     *              True once requestFinish() was called.
     */
    [[nodiscard]] LoopClosingStatus checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief       Marks the thread as stopped.
     */
    [[nodiscard]] LoopClosingStatus setFinish(void);
#ifdef REGISTER_LOOP
    std::string mstrFolderLoop;
#endif
};

} // namespace core
} // namespace vs_graphs

#endif
