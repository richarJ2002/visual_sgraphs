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
 * @file            LocalMapping.h
 *
 * @brief           Declares LocalMapping, the thread that inserts new key
 *                  frames, creates map points and runs local bundle adjustment.
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

class Tracking;
class LoopClosing;
class Atlas;

/*!
 * @brief           The local-mapping thread: takes key frames from the tracker,
 *                  turns them into map points and keeps the nearby map
 *                  consistent. Connected to Tracking and LoopClosing by
 *                  borrowed pointers.
 */
class LocalMapping
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the local mapper. Connect it with setTracker()
     *                  and setLoopCloser() before starting run() on its own
     *                  thread.
     *
     * @param[in]       p_atlas_in
     *                  Atlas holding every map; borrowed, shall outlive the
     *                  mapper.
     *
     * @param[in]       monocular_in
     *                  Non-zero when the sensor is monocular.
     *
     * @param[in]       inertial_in
     *                  True when an IMU is used.
     */
    LocalMapping(Atlas *p_atlas_in, const float monocular_in, bool inertial_in);

    /*!
     * @brief           Tells the mapper which loop closer to use.
     *
     * @param[in]       p_loopCloser_in
     *                  Loop closer; borrowed, shall outlive the mapper.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        setLoopCloser(LoopClosing *p_loopCloser_in);

    /*!
     * @brief           Tells the mapper which tracker to use.
     *
     * @param[in]       p_tracker_in
     *                  Tracker; borrowed, shall outlive the mapper.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus setTracker(Tracking *p_tracker_in);

    // Main function
    /*!
     * @brief           Thread body: repeatedly takes the next queued key frame,
     *                  creates and culls map points, runs local bundle
     *                  adjustment and, with an IMU, the inertial
     *                  initialisation, until a finish is requested.
     */
    void run();

    /*!
     * @brief           Queues a key frame for processing by run() and asks any
     *                  running bundle adjustment to abort.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to process; borrowed from the caller.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus insertKeyFrame(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Processes every queued key frame immediately, without
     *                  the culling and optimisation steps of run().
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus emptyQueue();

    // Thread Synch
    /*!
     * @brief           Asks the thread to stop at its next safe point and
     *                  aborts a running bundle adjustment.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus requestStop();

    /*!
     * @brief           Asks the thread to clear its queue and reset, then
     *                  blocks until run() has done so.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus requestReset();

    /*!
     * @brief           Same as requestReset() but for one map; blocks until
     *                  run() has handled the request.
     *
     * @param[in]       p_map_in
     *                  Map to reset; borrowed. Stored in p_mapToReset.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus requestResetActiveMap(Map *p_map_in);

    /*!
     * @brief           Stops the thread if a stop was requested and nothing
     *                  blocks it (see setNotStop()).
     *
     * @param[out]      isStopped_out
     *                  True when the thread is now stopped.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus stop(bool &isStopped_out);

    /*!
     * @brief           Resumes a stopped thread and deletes the key frames
     *                  still queued. Does nothing once the thread has finished.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus release();

    /*!
     * @brief           Reports whether the thread is stopped.
     *
     * @param[out]      isStopped_out
     *                  True when stopped.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus isStopped(bool &isStopped_out);

    /*!
     * @brief           Reports whether a stop has been requested.
     *
     * @param[out]      isStopRequested_out
     *                  True when requestStop() was called and not yet released.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus stopRequested(bool &isStopRequested_out);

    /*!
     * @brief           Reports whether the tracker may send new key frames now.
     *
     * @param[out]      isAcceptingKeyFrames_out
     *                  True when the mapper is idle and accepts key frames.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        isAcceptingKeyFrames(bool &isAcceptingKeyFrames_out);

    /*!
     * @brief           Sets whether new key frames are accepted. run() clears
     *                  it while busy so the tracker sees the mapper as
     *                  occupied.
     *
     * @param[in]       shouldAcceptKeyFrames_in
     *                  True to accept key frames.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        setAcceptKeyFrames(bool shouldAcceptKeyFrames_in);

    /*!
     * @brief           Blocks or unblocks stopping of the thread, so a caller
     *                  can keep the mapper running while it uses the map.
     *
     * @param[in]       shouldPreventStop_in
     *                  True to block stopping, false to allow it again.
     *
     * @param[out]      wasSet_out
     *                  False only when blocking was refused because the thread
     *                  is already stopped; true otherwise.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus setNotStop(bool  shouldPreventStop_in,
                                                bool &wasSet_out);

    /*!
     * @brief           Asks a running local bundle adjustment to abort early.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus interruptBA();

    /*!
     * @brief           Asks run() to leave its loop and finish.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus requestFinish();

    /*!
     * @brief           Reports whether run() has finished (true before run()
     *                  starts as well).
     *
     * @param[out]      isFinished_out
     *                  True when the thread is finished.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus isFinished(bool &isFinished_out);

    /*!
     * @brief           Counts the key frames waiting to be processed.
     *
     * @param[out]      keyFrameCount_out
     *                  Number of queued key frames.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus keyframesInQueue(int &keyFrameCount_out)
    {
        std::unique_lock<std::mutex> lock(newKeyFramesMutex);
        keyFrameCount_out = newKeyFrames.size();
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    }

    /*!
     * @brief           Reports whether an IMU initialisation or scale
     *                  refinement is running.
     *
     * @param[out]      isInitializing_out
     *                  True while one is running.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        isInitializing(bool &isInitializing_out) const;

    /*!
     * @brief           Returns the timestamp of the key frame being processed.
     *
     * @param[out]      currentKeyFrameTime_out
     *                  Timestamp in seconds; 0.0 when there is no current key
     *                  frame yet.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        getCurrentKeyFrameTime(double &currentKeyFrameTime_out);

    /*!
     * @brief           Returns the key frame the thread is working on.
     *
     * @param[out]      p_currentKeyFrame_out
     *                  Borrowed pointer; null before the first key frame. Read
     *                  without a lock, so it may change while the thread runs.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        getCurrentKeyFrame(KeyFrame *&p_currentKeyFrame_out);

    /*!
     * @brief           Rotation from the gravity-aligned frame to the world
     *                  frame, estimated by IMU initialisation; identity before
     *                  that, and scale refinement resets it to identity first.
     */
    Eigen::Matrix3d mRwg;

    /*!
     * @brief           Gyroscope bias, rad/s, from the last IMU initialisation;
     *                  zero before the first one.
     */
    Eigen::Vector3d mbg;

    /*!
     * @brief           Accelerometer bias, m/s^2, from the last IMU
     *                  initialisation; zero before the first one.
     */
    Eigen::Vector3d mba;

    /*!
     * @brief           Metric scale factor found by the inertial optimisation;
     *                  1.0 means no correction.
     */
    double scale;

    /*!
     * @brief           Time span, seconds, between the first key frame and the
     *                  tracker's last frame at the last IMU initialisation; 0
     *                  before the first one.
     */
    double initTime;

    /*!
     * @brief           Number of IMU initialisations completed; set back to 0
     *                  on a full reset.
     */
    unsigned int initIndex;

    /*!
     * @brief           Number of key frames used by the last IMU
     *                  initialisation.
     */
    unsigned int keyFrameCount;

    /*!
     * @brief           Timestamp, seconds, of the first key frame of the
     *                  initialisation window; 0 before the first IMU
     *                  initialisation.
     */
    double firstTimestamp;

    /*!
     * @brief           Number of map points the tracker matched in the latest
     *                  frame; written by Tracking::trackLocalMap().
     */
    int matchesInliers;

    /*!
     * @brief           True when the IMU is flagged unusable; run() then skips
     *                  key-frame processing. Only a reset assigns it (false).
     */
    bool isImuBad;

    /*!
     * @brief           True to ignore far points (beyond farPointsThreshold)
     *                  when creating map points and matching local points. Set
     *                  by System::initialize().
     */
    bool shouldSkipFarPoints;

    /*!
     * @brief           Distance beyond which a point counts as far, in map
     *                  units; from the settings file (thFarPoints).
     */
    float farPointsThreshold;

#ifdef REGISTER_TIMES
    std::vector<double> keyFrameInsertTimes_ms;
    std::vector<double> mapPointCullingTimes_ms;
    std::vector<double> mapPointCreationTimes_ms;
    std::vector<double> localBaTimes_ms;
    std::vector<double> keyFrameCullingTimes_ms;
    std::vector<double> localMappingTotalTimes_ms;

    std::vector<double> localBaSyncTimes_ms;
    std::vector<double> keyFrameCullingSyncTimes_ms;
    std::vector<int>    localBaEdgeCounts;
    std::vector<int>    localBaOptimizedKeyFrameCounts;
    std::vector<int>    localBaFixedKeyFrameCounts;
    std::vector<int>    localBaMapPointCounts;
    int                 localBaExecutionCount;
    int                 localBaAbortCount;
#endif
  protected:
    /*!
     * @brief           Reports whether any key frame is queued.
     *
     * @param[out]      hasNewKeyFrames_out
     *                  True when the queue is not empty.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus
        checkNewKeyFrames(bool &hasNewKeyFrames_out);

    /*!
     * @brief           Takes the next queued key frame as the current one,
     *                  computes its bag of words and links its map points to
     *                  it.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus processNewKeyFrame();

    /*!
     * @brief           Triangulates new map points between the current key
     *                  frame and its covisible neighbours.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus createNewMapPoints();

    /*!
     * @brief           Removes recently created map points that too few key
     *                  frames observe.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus mapPointCulling();

    /*!
     * @brief           Matches map points between the current key frame and its
     *                  neighbours and fuses duplicates.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus searchInNeighbors();

    /*!
     * @brief           Marks as bad the neighbouring key frames whose map
     *                  points are almost all seen by other key frames.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus keyFrameCulling();

    /*!
     * @brief           True when the sensor is monocular (set from the
     *                  constructor's monocular_in).
     */
    bool isMonocular;

    /*!
     * @brief           True when an IMU is used.
     */
    bool isInertial;

    /*!
     * @brief           Performs a pending reset: clears the key-frame queue and
     *                  recent map points and restarts IMU initialisation state.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus resetIfRequested();

    /*!
     * @brief           True while a full reset is requested; guarded by
     *                  resetMutex.
     */
    bool isResetRequested;

    /*!
     * @brief           True while a reset of one map is requested; guarded by
     *                  resetMutex.
     */
    bool isResetActiveMapRequested;

    /*!
     * @brief           Map named by requestResetActiveMap(); borrowed. Guarded
     *                  by resetMutex.
     */
    Map *p_mapToReset;

    /*!
     * @brief           Guards the reset flags and p_mapToReset.
     */
    std::mutex resetMutex;

    /*!
     * @brief           Reports whether a finish was requested.
     *
     * @param[out]      isFinishRequested_out
     *                  True when requestFinish() was called.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief           Marks the thread finished and stopped.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always.
     */
    [[nodiscard]] LocalMappingStatus setFinish();

    /*!
     * @brief           True once requestFinish() was called; guarded by
     *                  finishMutex.
     */
    bool isFinishRequested;

    /*!
     * @brief           True when run() is not running (also before it starts);
     *                  guarded by finishMutex.
     */
    bool hasFinished;

    /*!
     * @brief           Guards the finish flags.
     */
    std::mutex finishMutex;

    /*!
     * @brief           Atlas holding every map; borrowed from System.
     */
    Atlas *p_atlas;

    /*!
     * @brief           Loop closer; borrowed, nullptr until setLoopCloser().
     */
    LoopClosing *p_loopCloser;

    /*!
     * @brief           Tracker; borrowed, nullptr until setTracker().
     */
    Tracking *p_tracker;

    /*!
     * @brief           Key frames waiting to be processed, oldest first;
     *                  guarded by newKeyFramesMutex.
     */
    std::list<KeyFrame *> newKeyFrames;

    /*!
     * @brief           Key frame being processed; borrowed, null until the
     *                  first key frame is taken from the queue.
     */
    KeyFrame *p_currentKeyFrame;

    /*!
     * @brief           Map points created recently, still checked by
     *                  mapPointCulling(); borrowed.
     */
    std::list<MapPoint *> recentAddedMapPoints;

    /*!
     * @brief           Guards newKeyFrames.
     */
    std::mutex newKeyFramesMutex;

    /*!
     * @brief           Set to true to make a running bundle adjustment give up;
     *                  cleared by run() before it starts one.
     */
    bool shouldAbortBa;

    /*!
     * @brief           True while the thread is stopped or finished; guarded by
     *                  stopMutex.
     */
    bool hasStopped;

    /*!
     * @brief           True after requestStop() until release(); guarded by
     *                  stopMutex.
     */
    bool isStopRequested;

    /*!
     * @brief           True while setNotStop() blocks stopping; guarded by
     *                  stopMutex.
     */
    bool isStopBlocked;

    /*!
     * @brief           Guards the stop flags.
     */
    std::mutex stopMutex;

    /*!
     * @brief           True when the tracker may queue a key frame; guarded by
     *                  acceptMutex.
     */
    bool shouldAcceptKeyFrames;

    /*!
     * @brief           Guards shouldAcceptKeyFrames.
     */
    std::mutex acceptMutex;

    /*!
     * @brief           Estimates gravity direction, biases and scale from the
     *                  recent key frames and applies them to the map, once
     *                  enough time and motion are available.
     *
     * @param[in]       gyroPriorWeight_in
     *                  Weight of the gyroscope-bias prior.
     *
     * @param[in]       accelPriorWeight_in
     *                  Weight of the accelerometer-bias prior.
     *
     * @param[in]       shouldRunFullInertialBa_in
     *                  True to also run the full inertial bundle adjustment.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always, including when it
     *                  returns early without initialising.
     */
    [[nodiscard]] LocalMappingStatus
        initializeIMU(float gyroPriorWeight_in         = 1e2,
                      float accelPriorWeight_in        = 1e6,
                      bool  shouldRunFullInertialBa_in = false);

    /*!
     * @brief           Re-estimates the metric scale and gravity rotation of
     *                  the current map from its key frames and applies the
     *                  result.
     *
     * @return          LOCAL_MAPPING_STATUS_SUCCESS always, including when it
     *                  returns early because the scale is too small.
     */
    [[nodiscard]] LocalMappingStatus scaleRefinement();

    /*!
     * @brief           True while initializeIMU() or scaleRefinement() is
     *                  running.
     */
    bool isInitializationInProgress;

    /*!
     * @brief           Seconds of usable motion since the IMU initialisation
     *                  window began; 0 after a reset.
     */
    float initializationStartTime;
};

} // namespace core
} // namespace vs_graphs
#endif
