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

#include "LoopClosing.h"

#include "Utils/Converter/objects/Converter.h"
#include "G2oTypes.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Semantic/SemanticVerify.h"
#include "Sim3Solver.h"

#include <chrono>
#include <limits>
#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

/* Declared in LoopClosing.h: shared with SemanticVerify. */
bool verifyLoopMergeFloors(
    Map             *p_survivingMap_in,
    Map             *p_absorbedMap_in,
    const g2o::Sim3 &transform_absorbedWorldToSurvivingWorld_in,
    std::string     &result_out)
{
    semantic::Floor *p_survivingFloor =
        semantic::Floor::selectBestObservedFloor(
            p_survivingMap_in->getAllFloors());
    semantic::Floor *p_absorbedFloor = semantic::Floor::selectBestObservedFloor(
        p_absorbedMap_in->getAllFloors());

    const std::optional<semantic::Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? p_survivingFloor->getPlaneIdentity()
                                    : std::nullopt;
    const std::optional<semantic::Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? p_absorbedFloor->getPlaneIdentity()
                                   : std::nullopt;

    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        result_out = "DEFERRED";
        std::cout << "[FloorVerify] Map#" << p_survivingMap_in->getId()
                  << " and Map#" << p_absorbedMap_in->getId()
                  << " floor verification deferred (current="
                  << (survivingIdentity.has_value() ? "valid" : "missing")
                  << ", merge="
                  << (absorbedIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0" << std::endl;
        return false;
    }

    const std::optional<semantic::Floor::PlaneIdentity>
        transformedAbsorbedIdentity = semantic::Floor::transformPlaneIdentity(
            *absorbedIdentity,
            transform_absorbedWorldToSurvivingWorld_in);
    double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
    double floorOffset_m        = std::numeric_limits<double>::infinity();

    const bool floorsMatch = transformedAbsorbedIdentity.has_value() &&
                             semantic::Floor::planeIdentitiesMatch(
                                 *survivingIdentity,
                                 *transformedAbsorbedIdentity,
                                 semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
                                 semantic::Floor::kMergeMaxPlaneOffset_m,
                                 floorNormalAngle_deg,
                                 floorOffset_m);

    if (!floorsMatch)
    {
        result_out = "REJECTED";
        std::cerr << "[FloorVerify] Rejecting loop merge: Map#"
                  << p_survivingMap_in->getId() << " and Map#"
                  << p_absorbedMap_in->getId()
                  << " floor planes mismatch (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m << " m; limits="
                  << semantic::Floor::kMergeMaxPlaneNormalAngle_deg << " deg/"
                  << semantic::Floor::kMergeMaxPlaneOffset_m
                  << " m). result=REJECTED committed=0" << std::endl;
        return false;
    }

    std::cout << "[FloorVerify] Map#" << p_survivingMap_in->getId()
              << " and Map#" << p_absorbedMap_in->getId()
              << " floor planes match (angle=" << floorNormalAngle_deg
              << " deg, offset=" << floorOffset_m
              << " m). result=ACCEPTED committed=0" << std::endl;
    result_out = "ACCEPTED";
    return true;
}

namespace
{

void mergeFloorEvidenceAndRooms(semantic::Floor *p_retainedFloor_inout,
                                semantic::Floor *p_duplicateFloor_in)
{
    if (p_retainedFloor_inout == nullptr || p_duplicateFloor_in == nullptr ||
        p_retainedFloor_inout == p_duplicateFloor_in)
    {
        return;
    }

    if (semantic::Floor::selectBestObservedFloor(
            {p_retainedFloor_inout, p_duplicateFloor_in}) ==
        p_duplicateFloor_in)
    {
        const std::optional<semantic::Floor::PlaneIdentity> betterIdentity =
            p_duplicateFloor_in->getPlaneIdentity();
        if (betterIdentity.has_value())
        {
            p_retainedFloor_inout->setPlaneIdentity(
                betterIdentity->equation_World,
                betterIdentity->finiteSupportCount,
                betterIdentity->observationCount);
        }
        const Eigen::Vector3d betterCentroid =
            p_duplicateFloor_in->getCentroid();
        if (betterCentroid.allFinite())
        {
            p_retainedFloor_inout->setCentroid(betterCentroid);
        }
    }

    for (semantic::Room *p_room : p_duplicateFloor_in->getRooms())
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_retainedFloor_inout->addRoom(p_room);
        }
    }
}

void collapseMergedFloors(Map *p_survivingMap_in)
{
    if (p_survivingMap_in == nullptr)
    {
        return;
    }

    const std::vector<semantic::Floor *> allFloors =
        p_survivingMap_in->getAllFloors();
    if (allFloors.size() <= 1U)
    {
        return;
    }

    semantic::Floor *p_keeperFloor =
        semantic::Floor::selectBestObservedFloor(allFloors);
    if (p_keeperFloor == nullptr)
    {
        return;
    }

    for (semantic::Floor *p_duplicateFloor : allFloors)
    {
        if (p_duplicateFloor == nullptr || p_duplicateFloor == p_keeperFloor)
        {
            continue;
        }

        for (semantic::Room *p_room : p_duplicateFloor->getRooms())
        {
            if (p_room != nullptr && !p_room->isBad())
            {
                p_keeperFloor->addRoom(p_room);
            }
        }

        p_survivingMap_in->eraseMapFloor(p_duplicateFloor);
        std::cout << "[LoopClosing] Fused duplicate semantic::Floor#"
                  << p_duplicateFloor->getId() << " into semantic::Floor#"
                  << p_keeperFloor->getId()
                  << " and retained the better-observed plane identity."
                  << std::endl;
    }

    for (semantic::Room *p_room : p_survivingMap_in->getAllDetectedMapRooms())
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_keeperFloor->addRoom(p_room);
        }
    }
}
} // namespace

LoopClosing::LoopClosing(Atlas            *pAtlas,
                         KeyFrameDatabase *pDB,
                         ORBVocabulary    *pVoc,
                         const bool        bFixScale,
                         const bool        bActiveLC) :
    resetRequested(false),
    resetActiveMapRequested(false),
    finishRequested(false),
    finished(true),
    p_atlas(pAtlas),
    p_keyFrameDatabase(pDB),
    p_orbVocabulary(pVoc),
    p_matchedKF(nullptr),
    loopDetected(false),
    loopNumCoincidences(0),
    loopNumNotFound(0),
    mergeDetected(false),
    mergeInProgress(false),
    mergeNumCoincidences(0),
    mergeNumNotFound(0),
    lastLoopKeyFrameId(0),
    runningGBA(false),
    finishedGBA(true),
    p_threadGBA(nullptr),
    fixScale(bFixScale),
    fullBundleAdjustmentIndex(0),
    activeLC(bActiveLC)
{
    covisibilityConsistencyThreshold = 3;
    p_lastCurrentKF                  = static_cast<KeyFrame *>(nullptr);

#ifdef REGISTER_TIMES

    vdDataQuery_ms.clear();
    vdEstSim3_ms.clear();
    vdPRTotal_ms.clear();

    vdMergeMaps_ms.clear();
    vdWeldingBA_ms.clear();
    vdMergeOptEss_ms.clear();
    vdMergeTotal_ms.clear();
    vnMergeKFs.clear();
    vnMergeMPs.clear();
    nMerges = 0;

    vdLoopFusion_ms.clear();
    vdLoopOptEss_ms.clear();
    vdLoopTotal_ms.clear();
    vnLoopKFs.clear();
    nLoop = 0;

    vdGBA_ms.clear();
    vdUpdateMap_ms.clear();
    vdFGBATotal_ms.clear();
    vnGBAKFs.clear();
    vnGBAMPs.clear();
    nFGBA_exec  = 0;
    nFGBA_abort = 0;

#endif

    mstrFolderSubTraj = "SubTrajectories/";
    numCorrection     = 0;
    correctionGBA     = 0;
}

void LoopClosing::setTracker(Tracking *pTracker)
{
    p_tracker = pTracker;
}

void LoopClosing::setLocalMapper(LocalMapping *pLocalMapper)
{
    p_localMapper = pLocalMapper;
}

void LoopClosing::setMergeStatus(bool mergeStatus_in)
{
    mergeInProgress.store(mergeStatus_in);
}

LoopClosing::LoopCorrectionStatus LoopClosing::getLoopCorrectionStatus() const
{
    std::lock_guard<std::mutex> lock(mMutexLoopCorrectionStatus);
    return loopCorrectionStatus;
}

void LoopClosing::recordLoopCorrectionEvent(bool               accepted_in,
                                            const std::string &reason_in)
{
    {
        std::lock_guard<std::mutex> lock(mMutexLoopCorrectionStatus);
        ++loopCorrectionStatus.sequence;
        loopCorrectionStatus.hasEvent     = true;
        loopCorrectionStatus.lastAccepted = accepted_in;
        loopCorrectionStatus.lastReason   = reason_in;

        if (accepted_in)
        {
            ++loopCorrectionStatus.acceptedCount;
        }
        else
        {
            ++loopCorrectionStatus.rejectedCount;
        }

        if (p_currentKF != nullptr)
        {
            loopCorrectionStatus.lastCurrentKeyFrameId = p_currentKF->mnId;
            loopCorrectionStatus.lastCurrentTimestamp  = p_currentKF->timeStamp;
            if (p_currentKF->getMap() != nullptr)
            {
                loopCorrectionStatus.lastMapId = p_currentKF->getMap()->getId();
            }
        }
        if (p_loopMatchedKF != nullptr)
        {
            loopCorrectionStatus.lastMatchedKeyFrameId = p_loopMatchedKF->mnId;
            loopCorrectionStatus.lastMatchedTimestamp =
                p_loopMatchedKF->timeStamp;
        }
    }
}

bool LoopClosing::stopGlobalBundleAdjustment()
{
    std::thread *p_globalBundleAdjustmentThread = nullptr;
    bool         optimizationWasRunning         = false;

    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);

        optimizationWasRunning = runningGBA;

        if (optimizationWasRunning)
        {
            /* Invalidate the result before waiting for the worker to finish. */
            ++fullBundleAdjustmentIndex;
            globalBundleAdjustmentStopRequested.store(
                true,
                std::memory_order_release);
        }

        /*
         * Move ownership out while holding the state mutex. The mutex must be
         * released before join() because the worker takes it while finishing.
         */
        p_globalBundleAdjustmentThread = p_threadGBA;
        p_threadGBA                    = nullptr;
    }

    if (p_globalBundleAdjustmentThread != nullptr)
    {
        if (p_globalBundleAdjustmentThread->joinable())
        {
            p_globalBundleAdjustmentThread->join();
        }

        delete p_globalBundleAdjustmentThread;
    }

    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
        runningGBA  = false;
        finishedGBA = true;
    }

    return optimizationWasRunning;
}

void LoopClosing::relaunchGlobalBundleAdjustment(Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr || p_currentKF == nullptr)
    {
        return;
    }

    std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
    runningGBA  = true;
    finishedGBA = false;
    globalBundleAdjustmentStopRequested.store(false, std::memory_order_release);
    p_threadGBA = new std::thread(&LoopClosing::runGlobalBundleAdjustment,
                                  this,
                                  p_activeMap_in,
                                  p_currentKF->mnId,
                                  fullBundleAdjustmentIndex);
}

void LoopClosing::run(void)
{
    /*!
     * Mark the LoopClosing worker as active. SetFinish() changes this back to
     * true when Run() exits.
     *
     * This flag is observed by other threads through isFinished().
     */
    finished = false;

    /*!
     * Keep the LoopClosing worker alive until another thread requests shutdown.
     * One iteration is one LoopClosing polling cycle.
     */
    while (true)
    {
        /* Find the current time of the loop */
        const std::chrono::_V2::system_clock::time_point start =
            std::chrono::high_resolution_clock::now();

        /*!
         * Do expensive place recognition only when the loop-keyframe queue
         * contains work.
         *
         * true  -> at least one queued keyframe exists; process one.
         * false -> skip directly to reset/finish handling and the timed sleep.
         */
        if (checkNewKeyFrames())
        {
            /*!
             * Check that the last keyframe to be added is valid.
             *
             * If so, then clear the buffer of candidate keyframes and buffer
             * of merged keyframes. he previous processed KF may still hold
             * debug/candidate lists from its last place-recognition query.
             */
            if (p_lastCurrentKF)
            {
                /*!
                 * Remove stale same-map loop candidates associated with the
                 * previous current KF.
                 */
                p_lastCurrentKF->loopCandKFs.clear();

                /*!
                 * Remove stale cross-map merge candidates associated with the
                 * previous current KF.
                 */
                p_lastCurrentKF->mergeCandKFs.clear();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPR =
                std::chrono::steady_clock::now();
#endif

            /*!
             * Pop/process the next queued keyframe and look for a same-map loop
             * or cross-map merge candidate. This function consumes the next KF,
             * queries the database, validates Sim3 geometry, and updates
             * mbLoopDetected / mbMergeDetected plus their matched-KF state.
             */
            bool bFindedRegion = newDetectCommonRegions();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPR =
                std::chrono::steady_clock::now();

            double timePRTotal = std::chrono::duration_cast<
                                     std::chrono::duration<double, std::milli>>(
                                     time_EndPR - time_StartPR)
                                     .count();
            vdPRTotal_ms.push_back(timePRTotal);
#endif

            /* If a detected region is found, perform loop closure */
            if (bFindedRegion)
            {
                /* Merge if NewDetectCommonRegions() indicates so */
                if (mergeDetected)
                {
                    semantic::SemanticMergeDecision mergeDecision =
                        semantic::SemanticMergeDecision::REJECT;

                    /* If required, confirm IMU is working */
                    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                         p_tracker->sensor == System::IMU_STEREO ||
                         p_tracker->sensor == System::IMU_RGBD) &&
                        (!p_currentKF->getMap()->isImuInitialized()))
                    {
                        cout << "IMU is not initilized, merge is aborted"
                             << endl;
                    }
                    else
                    {
                        /*!
                         * Get pose of the matched keyframe in the matched
                         * keyframes world frame.
                         */
                        Sophus::SE3d mTmw =
                            p_mergeMatchedKF->getPose().cast<double>();

                        /* Convert above keyframe pose into Sim3 datatype */
                        g2o::Sim3 gSmw2(mTmw.unit_quaternion(),
                                        mTmw.translation(),
                                        1.0);

                        /*!
                         * Get pose of the current keyframe in the current
                         * keyframes world frame.
                         */
                        Sophus::SE3d mTcw =
                            p_currentKF->getPose().cast<double>();

                        /* Convert above keyframe pose into Sim3 datatype */
                        g2o::Sim3 gScw1(mTcw.unit_quaternion(),
                                        mTcw.translation(),
                                        1.0);

                        /*!
                         * `mg2oMergeSlw` is the pose of the CURRENT camera from
                         * the current keyframe in the world frame of the
                         * MATCHED keyframe. This is effectively the pose which
                         * causes the loop closure.
                         *
                         * First, find the pose from the matched keyframes world
                         * frame to the current camera.
                         *
                         * Also store the current pose.
                         */
                        const g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();

                        /*!
                         * Find the transform from the current keyframes world
                         * map to the matched keyframes world frame
                         */
                        oldCorrectedPose = (gSw2c * gScw1);

                        /*!
                         * If in both frames an IMU is used, use IMU readings
                         * for inertial odometry map constraints.
                         */
                        if (p_currentKF->getMap()->isInertial() &&
                            p_mergeMatchedKF->getMap()->isInertial())
                        {
                            cout << "Merge check transformation with IMU"
                                 << endl;

                            /* Reject maps with bad scale */
                            if (oldCorrectedPose.scale() < 0.90 ||
                                oldCorrectedPose.scale() > 1.1)
                            {
                                p_mergeLastCurrentKF->setErase();
                                p_mergeMatchedKF->setErase();
                                mergeNumCoincidences = 0;
                                mergeMatchedMPs.clear();
                                mergeMPs.clear();
                                mergeNumNotFound = 0;
                                mergeDetected    = false;
                                Verbose::printMess(
                                    "scale bad estimated. Abort merging",
                                    Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                                 p_tracker->sensor == System::IMU_STEREO ||
                                 p_tracker->sensor == System::IMU_RGBD) &&
                                p_currentKF->getMap()->getInertialBA1())
                            {
                                Eigen::Vector3d phi =
                                    LogSO3(oldCorrectedPose.rotation()
                                               .toRotationMatrix());
                                phi(0) = 0;
                                phi(1) = 0;
                                oldCorrectedPose =
                                    g2o::Sim3(ExpSO3(phi),
                                              oldCorrectedPose.translation(),
                                              1.0);
                            }
                        }

                        /*!
                         * This creates a transform from the current keyframes
                         * world frame to the matched cameras keyframe. This
                         * is the critical transform to cause the loop closure.
                         *
                         * gScw1:   current keyframe world -> current camera
                         * gSw2c:   current camera -> matched keyframe world
                         * gWmw2:   matched keyframes world -> matched camera
                         *
                         *  - mg2oMergeScw:     (Primary) World frame to Camera
                         *                      frame transform
                         *
                         *  - mg2oMergew1m:     Matched Camera frame to Primary
                         *                      World frame
                         *
                         * @note        Note that w1 reffers to the primary
                         *              frame which is the current frame that
                         *              the camera is in. This means that the
                         *              map is appended onto the current map
                         *              and hence avoids teleporting the camera
                         *              position and allows for smooth
                         *              operation (which would be the case if
                         *              the primary map was the matched map)
                         */
                        mg2oMergeSmw   = gSmw2 * gSw2c * gScw1;
                        mg2oMergeScw   = mg2oMergeSlw;
                        mg2oMergeSw1w2 = (gSw2c * gScw1).inverse();

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartMerge =
                            std::chrono::steady_clock::now();
#endif

                        /* Set flag to indicate that mergins is happening */
                        setMergeStatus(true);

                        /* Choose merging method based on if IMU is used */
                        if (p_tracker->sensor == System::IMU_MONOCULAR ||
                            p_tracker->sensor == System::IMU_STEREO ||
                            p_tracker->sensor == System::IMU_RGBD)
                        {
                            /* Merge maps using IMU */
                            mergeDecision = mergeLocalInertial();
                        }
                        else
                        {
                            /* Merge maps */
                            mergeDecision = mergeLocal();
                        }

                        /* Set flag to indicate that mergins has finished */
                        setMergeStatus(false);

#ifdef REGISTER_TIMES
                        if (mergeDecision ==
                            semantic::SemanticMergeDecision::ACCEPT)
                        {
                            std::chrono::steady_clock::time_point
                                time_EndMerge =
                                    std::chrono::steady_clock::now();

                            double timeMergeTotal =
                                std::chrono::duration_cast<
                                    std::chrono::duration<double, std::milli>>(
                                    time_EndMerge - time_StartMerge)
                                    .count();
                            vdMergeTotal_ms.push_back(timeMergeTotal);
                            nMerges += 1;
                        }
#endif
                    }

                    if (mergeDecision ==
                        semantic::SemanticMergeDecision::ACCEPT)
                    {
                        std::cout
                            << "[LoopClosing] Map merge has been finished."
                            << std::endl;

                        /* Record only a merge that actually committed. */
                        vdPR_CurrentTime.push_back(p_currentKF->timeStamp);
                        vdPR_MatchedTime.push_back(p_mergeMatchedKF->timeStamp);
                        vnPR_TypeRecogn.push_back(1);

                        p_mergeLastCurrentKF->setErase();
                        p_mergeMatchedKF->setErase();
                        mergeNumCoincidences = 0;
                        mergeMatchedMPs.clear();
                        mergeMPs.clear();
                        mergeNumNotFound = 0;
                        mergeDetected    = false;

                        /* A committed merge invalidates any same-map loop
                         * candidate collected against the old topology. */
                        if (loopDetected)
                        {
                            recordLoopCorrectionEvent(
                                false,
                                "superseded_by_map_merge");
                            p_loopLastCurrentKF->setErase();
                            p_loopMatchedKF->setErase();
                            loopNumCoincidences = 0;
                            loopMatchedMPs.clear();
                            loopMPs.clear();
                            loopNumNotFound = 0;
                            loopDetected    = false;
                        }
                    }
                    else if (mergeDecision ==
                             semantic::SemanticMergeDecision::DEFER)
                    {
                        /* Retain the matched keyframe and accumulated
                         * coincidences. The next keyframe can refine the same
                         * candidate after semantic evidence advances. */
                        std::cout << "[LoopClosing] Map merge deferred; "
                                     "candidate retained for retry."
                                  << std::endl;
                    }
                    else
                    {
                        std::cout << "[LoopClosing] Map merge rejected; "
                                     "candidate discarded."
                                  << std::endl;
                        p_mergeLastCurrentKF->setErase();
                        p_mergeMatchedKF->setErase();
                        mergeNumCoincidences = 0;
                        mergeMatchedMPs.clear();
                        mergeMPs.clear();
                        mergeNumNotFound = 0;
                        mergeDetected    = false;
                    }
                }

                /*!
                 * A loop closure candidate has been successfully detected and
                 * geometrically validated. The matched keyframe belongs to the
                 * same map as the current keyframe, meaning the estimated Sim3
                 * transformation can be used to correct accumulated drift in
                 * the map.
                 */
                if (loopDetected)
                {
                    std::cout
                        << "[LoopClosing] Loop detected! Correcting the map ..."
                        << std::endl;

                    /* Init a variable to track of a good loop closure occurs */
                    bool bGoodLoop = true;

                    /*!
                     * Record the place recognition event for evaluation and
                     * debugging. The two timestamp vectors store corresponding
                     * keyframe pairs:
                     *
                     *      - Current keyframe: the keyframe that triggered
                     *        place recognition.
                     *
                     *      - Matched keyframe: the previously observed keyframe
                     *        that was recognised.
                     *
                     * vnPR_TypeRecogn identifies the recognition type:
                     *   0 -> loop closure
                     *   1 -> map merge
                     */
                    vdPR_CurrentTime.push_back(p_currentKF->timeStamp);
                    vdPR_MatchedTime.push_back(p_loopMatchedKF->timeStamp);
                    vnPR_TypeRecogn.push_back(0);

                    /*!
                     * The Sim3 transformation estimated during loop detection
                     * describes the relationship between the current keyframe
                     * and the matched keyframe. Store it as the loop correction
                     * transformation.
                     */
                    mg2oLoopScw = mg2oLoopSlw;

                    /*!
                     * In inertial systems additional validation is performed
                     * before applying the loop correction.
                     *
                     * IMU constraints provide an estimate of gravity alignment,
                     * therefore large rotational discrepancies or scale changes
                     * indicate that the detected loop is likely a false
                     * positive.
                     */
                    if (p_currentKF->getMap()->isInertial())
                    {
                        Sophus::SE3d Twc =
                            p_currentKF->getPoseInverse().cast<double>();

                        /*!
                         * Convert the current camera pose from SE3 into a Sim3
                         * representation so that it can be combined with the
                         * loop correction estimate.
                         */
                        g2o::Sim3 g2oTwc(Twc.unit_quaternion(),
                                         Twc.translation(),
                                         1.0);

                        /*!
                         * Compute the resulting world-to-world transformation
                         * after applying the proposed loop correction.
                         *
                         * This represents the global adjustment required to
                         * align the current map trajectory with the previously
                         * observed location.
                         */
                        g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

                        /*!
                         * Extract the rotational difference from the proposed
                         * correction. For inertial maps, excessive rotation
                         * indicates insufficient overlap between the two
                         * observations and the correction is rejected.
                         */
                        Eigen::Vector3d phi =
                            LogSO3(g2oSww_new.rotation().toRotationMatrix());

                        if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f &&
                            fabs(phi(2)) < 0.349f)
                        {
                            /*!
                             * After inertial initialization, roll and pitch are
                             * constrained by gravity. Therefore only the yaw
                             * component of the loop correction is allowed to
                             * modify the map orientation.
                             */
                            if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                                 p_tracker->sensor == System::IMU_STEREO ||
                                 p_tracker->sensor == System::IMU_RGBD) &&
                                p_currentKF->getMap()->getInertialBA2())
                            {
                                phi(0)     = 0;
                                phi(1)     = 0;
                                g2oSww_new = g2o::Sim3(ExpSO3(phi),
                                                       g2oSww_new.translation(),
                                                       1.0);

                                /*!
                                 * Convert the constrained correction back into
                                 * the frame used by the loop closing module.
                                 */
                                mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                            }
                        }
                        else
                        {
                            /*!
                             * Reject the loop closure if the required
                             * correction is too large. A large rotation
                             * normally indicates an incorrect place recognition
                             * match.
                             */
                            std::cout
                                << "[LoopClosing] The loop lacks sufficient "
                                   "overlap! Skipping correction ..."
                                << std::endl;
                            bGoodLoop = false;
                            recordLoopCorrectionEvent(false,
                                                      "inertial_overlap");
                        }
                    }

                    /*!
                     * Apply the loop correction only after all validation
                     * checks have passed. CorrectLoop() performs the map
                     * optimisation and updates keyframe/map point poses to
                     * remove accumulated drift.
                     */
                    if (bGoodLoop)
                    {
                        loopMapPoints = loopMPs;

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartLoop =
                            std::chrono::steady_clock::now();

                        nLoop += 1;

#endif
                        /*!
                         * Apply the loop closure correction.
                         *
                         * CorrectLoop() performs the global map adjustment
                         * required after a loop has been detected. It uses the
                         * estimated Sim3 transformation and matched map points
                         * to:
                         *
                         *  - correct accumulated pose drift,
                         *  - update affected keyframe poses,
                         *  - update map point positions,
                         *  - optimise the essential graph,
                         *  - run bundle adjustment to refine the corrected map.
                         *
                         * After this operation, the trajectory should become
                         * globally consistent.
                         */
                        correctLoop();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndLoop =
                            std::chrono::steady_clock::now();

                        double timeLoopTotal =
                            std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndLoop - time_StartLoop)
                                .count();
                        vdLoopTotal_ms.push_back(timeLoopTotal);
#endif

                        /*!
                         * Track the number of successful loop closure
                         * corrections applied.
                         *
                         * This counter is incremented only after CorrectLoop()
                         * has completed, indicating that the detected loop was
                         * accepted and used to modify the map.
                         */
                        numCorrection += 1;
                        recordLoopCorrectionEvent(true, "corrected");
                    }

                    /*!
                     * Release all temporary loop closure state.
                     *
                     * The candidate keyframes are no longer required because
                     * the correction has either been applied or rejected.
                     * Resetting these variables allows future loop closure
                     * attempts to start from a clean state.
                     */
                    p_loopLastCurrentKF->setErase();
                    p_loopMatchedKF->setErase();
                    loopNumCoincidences = 0;
                    loopMatchedMPs.clear();
                    loopMPs.clear();
                    loopNumNotFound = 0;
                    loopDetected    = false;
                }
            }
            p_lastCurrentKF = p_currentKF;
        }

        resetIfRequested();

        if (checkFinish())
        {
            break;
        }

        /* Find the time after it took to run the loop */
        const auto end = std::chrono::high_resolution_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Find how much longer in the loop is left */
        const double remainingSeconds = runInterval_s - elapsed.count();

        /* If there is remaining time, sleep until next loop cycle */
        if (remainingSeconds > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remainingSeconds));
        }
    }

    /* No optimizer may outlive LoopClosing or race Atlas serialization. */
    stopGlobalBundleAdjustment();
    setFinish();
}

void LoopClosing::insertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if (pKF->mnId != 0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::checkNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::newDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be
    // performed
    if (!activeLC)
    {
        return false;
    }

    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        p_currentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this
        // thread
        p_currentKF->setNotErase();
        p_currentKF->currentPlaceRecognition = true;

        p_lastMap = p_currentKF->getMap();
    }

    if (p_lastMap->isInertial() && !p_lastMap->getInertialBA2())
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    if (p_tracker->sensor == System::STEREO &&
        p_lastMap->getAllKeyFrames().size() < 5) // 12
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    if (p_lastMap->getAllKeyFrames().size() < 12)
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    // Check the last candidates with geometric validation
    //  Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial     = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_1 =
        std::chrono::steady_clock::now();
#endif
    if (loopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (p_currentKF->getPose() * p_loopLastCurrentKF->getPoseInverse())
                .cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw           = gScl * mg2oLoopSlw;
        int       numProjMatches = 0;
        vector<MapPoint *> vpMatchedMPs;
        bool bCommonRegion = detectAndReffineSim3FromLastKF(p_currentKF,
                                                            p_loopMatchedKF,
                                                            gScw,
                                                            numProjMatches,
                                                            loopMPs,
                                                            vpMatchedMPs);
        if (bCommonRegion)
        {

            bLoopDetectedInKF = true;

            loopNumCoincidences++;
            p_loopLastCurrentKF->setErase();
            p_loopLastCurrentKF = p_currentKF;
            mg2oLoopSlw         = gScw;
            loopMatchedMPs      = vpMatchedMPs;

            loopDetected    = loopNumCoincidences >= 3;
            loopNumNotFound = 0;
        }
        else
        {
            bLoopDetectedInKF = false;

            loopNumNotFound++;
            if (loopNumNotFound >= 2)
            {
                recordLoopCorrectionEvent(false, "geometric_validation");
                p_loopLastCurrentKF->setErase();
                p_loopMatchedKF->setErase();
                loopNumCoincidences = 0;
                loopMatchedMPs.clear();
                loopMPs.clear();
                loopNumNotFound = 0;
            }
        }
    }

    // Merge candidates
    bool bMergeDetectedInKF = false;
    if (mergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (p_currentKF->getPose() * p_mergeLastCurrentKF->getPoseInverse())
                .cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw           = gScl * mg2oMergeSlw;
        int       numProjMatches = 0;
        vector<MapPoint *> vpMatchedMPs;
        bool bCommonRegion = detectAndReffineSim3FromLastKF(p_currentKF,
                                                            p_mergeMatchedKF,
                                                            gScw,
                                                            numProjMatches,
                                                            mergeMPs,
                                                            vpMatchedMPs);
        if (bCommonRegion)
        {
            bMergeDetectedInKF = true;

            mergeNumCoincidences++;
            p_mergeLastCurrentKF->setErase();
            p_mergeLastCurrentKF = p_currentKF;
            mg2oMergeSlw         = gScw;
            mergeMatchedMPs      = vpMatchedMPs;

            mergeDetected = mergeNumCoincidences >= 3;
        }
        else
        {
            mergeDetected      = false;
            bMergeDetectedInKF = false;

            mergeNumNotFound++;
            if (mergeNumNotFound >= 2)
            {
                p_mergeLastCurrentKF->setErase();
                p_mergeMatchedKF->setErase();
                mergeNumCoincidences = 0;
                mergeMatchedMPs.clear();
                mergeMPs.clear();
                mergeNumNotFound = 0;
            }
        }
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndEstSim3_1 =
        std::chrono::steady_clock::now();

    double timeEstSim3 =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndEstSim3_1 - time_StartEstSim3_1)
            .count();
#endif

    if (mergeDetected || loopDetected)
    {
#ifdef REGISTER_TIMES
        vdEstSim3_ms.push_back(timeEstSim3);
#endif
        p_keyFrameDatabase->add(p_currentKF);
        return true;
    }

    // TODO: This is only necessary if we use a minimun score for pick the best
    // candidates
    const vector<KeyFrame *> vpConnectedKeyFrames =
        p_currentKF->getVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
    if (!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartQuery =
            std::chrono::steady_clock::now();
#endif
        p_keyFrameDatabase->detectNBestCandidates(p_currentKF,
                                                  vpLoopBowCand,
                                                  vpMergeBowCand,
                                                  3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndQuery =
            std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   time_EndQuery - time_StartQuery)
                                   .count();
        vdDataQuery_ms.push_back(timeDataQuery);
#endif
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_2 =
        std::chrono::steady_clock::now();
#endif
    // Check the BoW candidates if the geometric candidate list is empty
    // Loop candidates
    if (!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        loopDetected = detectCommonRegionsFromBoW(vpLoopBowCand,
                                                  p_loopMatchedKF,
                                                  p_loopLastCurrentKF,
                                                  mg2oLoopSlw,
                                                  loopNumCoincidences,
                                                  loopMPs,
                                                  loopMatchedMPs);
    }
    // Merge candidates
    if (!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mergeDetected = detectCommonRegionsFromBoW(vpMergeBowCand,
                                                   p_mergeMatchedKF,
                                                   p_mergeLastCurrentKF,
                                                   mg2oMergeSlw,
                                                   mergeNumCoincidences,
                                                   mergeMPs,
                                                   mergeMatchedMPs);
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndEstSim3_2 =
        std::chrono::steady_clock::now();

    timeEstSim3 +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndEstSim3_2 - time_StartEstSim3_2)
            .count();
    vdEstSim3_ms.push_back(timeEstSim3);
#endif

    p_keyFrameDatabase->add(p_currentKF);

    if (mergeDetected || loopDetected)
    {
        return true;
    }

    p_currentKF->setErase();
    p_currentKF->currentPlaceRecognition = false;

    return false;
}

bool LoopClosing::detectAndReffineSim3FromLastKF(
    KeyFrame                *pCurrentKF,
    KeyFrame                *pMatchedKF,
    g2o::Sim3               &gScw,
    int                     &nNumProjMatches,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    set<MapPoint *> spAlreadyMatchedMPs;
    nNumProjMatches = findMatchesByProjection(pCurrentKF,
                                              pMatchedKF,
                                              gScw,
                                              spAlreadyMatchedMPs,
                                              vpMPs,
                                              vpMatchedMPs);

    int nProjMatches    = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if (nNumProjMatches >= nProjMatches)
    {
        // Verbose::PrintMess("Sim3 reffine: There are " +
        // to_string(nNumProjMatches) + " initial matches ",
        // Verbose::VERBOSITY_DEBUG);
        Sophus::SE3d mTwm = pMatchedKF->getPoseInverse().cast<double>();
        g2o::Sim3    gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3    gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale =
            fixScale; // TODO CHECK; Solo para el monocular inertial
        if (p_tracker->sensor == System::IMU_MONOCULAR &&
            !pCurrentKF->getMap()->getInertialBA2())
            bFixedScale = false;
        int numOptMatches = Optimizer::optimizeSim3(p_currentKF,
                                                    pMatchedKF,
                                                    vpMatchedMPs,
                                                    gScm,
                                                    10,
                                                    bFixedScale,
                                                    mHessian7x7,
                                                    true);

        // Verbose::PrintMess("Sim3 reffine: There are " +
        // to_string(numOptMatches) + " matches after of the optimization ",
        // Verbose::VERBOSITY_DEBUG);

        if (numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

            vector<MapPoint *> vpMatchedMP;
            vpMatchedMP.resize(p_currentKF->getMapPointMatches().size(),
                               static_cast<MapPoint *>(nullptr));

            nNumProjMatches = findMatchesByProjection(pCurrentKF,
                                                      pMatchedKF,
                                                      gScw_estimation,
                                                      spAlreadyMatchedMPs,
                                                      vpMPs,
                                                      vpMatchedMPs);
            if (nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::detectCommonRegionsFromBoW(
    std::vector<KeyFrame *> &vpBowCand,
    KeyFrame               *&pMatchedKF2,
    KeyFrame               *&pLastCurrentKF,
    g2o::Sim3               &g2oScw,
    int                     &nNumCoincidences,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    int nBoWMatches     = 20;
    int nBoWInliers     = 15;
    int nSim3Inliers    = 20;
    int nProjMatches    = 50;
    int nProjOptMatches = 80;

    set<KeyFrame *> spConnectedKeyFrames = p_currentKF->getConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame               *pBestMatchedKF;
    int                     nBestMatchesReproj   = 0;
    int                     nBestNumCoindicendes = 0;
    g2o::Sim3               g2oBestScw;
    std::vector<MapPoint *> vpBestMapPoints;
    std::vector<MapPoint *> vpBestMatchedMapPoints;

    int         numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    // Verbose::PrintMess("BoW candidates: There are " +
    // to_string(vpBowCand.size()) + " possible candidates ",
    // Verbose::VERBOSITY_DEBUG);
    for (KeyFrame *pKFi : vpBowCand)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
        // Current KF against KF with covisibles version
        std::vector<KeyFrame *> vpCovKFi =
            pKFi->getBestCovisibilityKeyFrames(nNumCovisibles);
        if (vpCovKFi.empty())
        {
            // std::cout << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        }
        else
        {
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }

        bool bAbortByNearKF = false;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (spConnectedKeyFrames.find(vpCovKFi[j]) !=
                spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
        if (bAbortByNearKF)
        {
            // std::cout << "Check BoW aborted because is close to the matched
            // one " << std::endl;
            continue;
        }
        // std::cout << "Check BoW continue because is far to the matched one "
        // << std::endl;

        std::vector<std::vector<MapPoint *>> vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint *> spMatchedMPi;
        int                  numBoWMatches = 0;

        KeyFrame *pMostBoWMatchesKF  = pKFi;
        int       nMostBoWNumMatches = 0;

        std::vector<MapPoint *> vpMatchedPoints =
            std::vector<MapPoint *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<MapPoint *>(nullptr));
        std::vector<KeyFrame *> vpKeyFrameMatchedMP =
            std::vector<KeyFrame *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<KeyFrame *>(nullptr));

        int nIndexMostBoWMatchesKF = 0;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.searchByBoW(p_currentKF,
                                             vpCovKFi[j],
                                             vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches     = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            for (int k = 0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPoint *pMPi_j = vvpMatchedMPs[j][k];
                if (!pMPi_j || pMPi_j->isBad())
                    continue;

                if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]     = pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        // pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

        if (numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = fixScale;
            if (p_tracker->sensor == System::IMU_MONOCULAR &&
                !p_currentKF->getMap()->getInertialBA2())
                bFixedScale = false;

            Sim3Solver solver = Sim3Solver(p_currentKF,
                                           pMostBoWMatchesKF,
                                           vpMatchedPoints,
                                           bFixedScale,
                                           vpKeyFrameMatchedMP);
            solver.setRansacParameters(0.99,
                                       nBoWInliers,
                                       300); // at least 15 inliers

            bool            bNoMore = false;
            vector<bool>    vbInliers;
            int             nInliers;
            bool            bConverge = false;
            Eigen::Matrix4f mTcm;
            while (!bConverge && !bNoMore)
            {
                mTcm =
                    solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
                // Verbose::PrintMess("BoW guess: Solver achieve " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
            }

            if (bConverge)
            {
                // std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                // Verbose::PrintMess("BoW guess: Convergende with " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
                //  Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->getBestCovisibilityKeyFrames(
                    nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFrame *> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                // std::cout << "There are " << vpCovKFi.size() <<" near KFs" <<
                // std::endl;

                set<MapPoint *>    spMapPoints;
                vector<MapPoint *> vpMapPoints;
                vector<KeyFrame *> vpKeyFrames;
                for (KeyFrame *pCovKFi : vpCovKFi)
                {
                    for (MapPoint *pCovMPij : pCovKFi->getMapPointMatches())
                    {
                        if (!pCovMPij || pCovMPij->isBad())
                            continue;

                        if (spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                // std::cout << "There are " << vpKeyFrames.size() <<" KFs which
                // view all the mappoints" << std::endl;

                g2o::Sim3 gScm(solver.getEstimatedRotation().cast<double>(),
                               solver.getEstimatedTranslation().cast<double>(),
                               (double)solver.getEstimatedScale());
                g2o::Sim3 gSmw(
                    pMostBoWMatchesKF->getRotation().cast<double>(),
                    pMostBoWMatchesKF->getTranslation().cast<double>(),
                    1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current
                                              // from the world position
                Sophus::Sim3f correctedPose = utils::converter::Converter::toSophus(gScw);

                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(p_currentKF->getMapPointMatches().size(),
                                   static_cast<MapPoint *>(nullptr));
                vector<KeyFrame *> vpMatchedKF;
                vpMatchedKF.resize(p_currentKF->getMapPointMatches().size(),
                                   static_cast<KeyFrame *>(nullptr));
                int numProjMatches = matcher.searchByProjection(p_currentKF,
                                                                correctedPose,
                                                                vpMapPoints,
                                                                vpKeyFrames,
                                                                vpMatchedMP,
                                                                vpMatchedKF,
                                                                8,
                                                                1.5);
                // cout <<"BoW: " << numProjMatches << " matches between " <<
                // vpMapPoints.size() << " points with coarse Sim3" << endl;

                if (numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = fixScale;
                    if (p_tracker->sensor == System::IMU_MONOCULAR &&
                        !p_currentKF->getMap()->getInertialBA2())
                        bFixedScale = false;

                    int numOptMatches = Optimizer::optimizeSim3(p_currentKF,
                                                                pKFi,
                                                                vpMatchedMP,
                                                                gScm,
                                                                10,
                                                                fixScale,
                                                                mHessian7x7,
                                                                true);

                    if (numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(
                            pMostBoWMatchesKF->getRotation().cast<double>(),
                            pMostBoWMatchesKF->getTranslation().cast<double>(),
                            1.0);
                        g2o::Sim3 gScw =
                            gScm * gSmw; // Similarity matrix of current from
                                         // the world position
                        Sophus::Sim3f correctedPose = utils::converter::Converter::toSophus(gScw);

                        vector<MapPoint *> vpMatchedMP;
                        vpMatchedMP.resize(
                            p_currentKF->getMapPointMatches().size(),
                            static_cast<MapPoint *>(nullptr));
                        int numProjOptMatches =
                            matcher.searchByProjection(p_currentKF,
                                                       correctedPose,
                                                       vpMapPoints,
                                                       vpMatchedMP,
                                                       5,
                                                       1.0);

                        if (numProjOptMatches >= nProjOptMatches)
                        {
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for (MapPoint *pMPi : vpMatchedMP)
                            {
                                if (!pMPi || pMPi->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t, size_t> indexes =
                                    pMPi->getIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if (index >= 0)
                                {
                                    int coord_x =
                                        pKFi->keyPointsUndistorted[index].pt.x;
                                    if (coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if (coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y =
                                        pKFi->keyPointsUndistorted[index].pt.y;
                                    if (coord_y < min_y)
                                    {
                                        min_y = coord_y;
                                    }
                                    if (coord_y > max_y)
                                    {
                                        max_y = coord_y;
                                    }
                                }
                            }

                            int                nNumKFs = 0;
                            // vpMatchedMPs = vpMatchedMP;
                            // vpMPs = vpMapPoints;
                            //  Check the Sim3 transformation with the current
                            //  KeyFrame covisibles
                            vector<KeyFrame *> vpCurrentCovKFs =
                                p_currentKF->getBestCovisibilityKeyFrames(
                                    nNumCovisibles);

                            int j = 0;
                            while (nNumKFs < 3 && j < vpCurrentCovKFs.size())
                            {
                                KeyFrame    *pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc =
                                    (pKFj->getPose() *
                                     p_currentKF->getPoseInverse())
                                        .cast<double>();
                                g2o::Sim3          gSjc(mTjc.unit_quaternion(),
                                               mTjc.translation(),
                                               1.0);
                                g2o::Sim3          gSjw = gSjc * gScw;
                                int                numProjMatches_j = 0;
                                vector<MapPoint *> vpMatchedMPs_j;
                                bool bValid = detectCommonRegionsFromLastKF(
                                    pKFj,
                                    pMostBoWMatchesKF,
                                    gSjw,
                                    numProjMatches_j,
                                    vpMapPoints,
                                    vpMatchedMPs_j);

                                if (bValid)
                                {
                                    Sophus::SE3f Tc_w  = p_currentKF->getPose();
                                    Sophus::SE3f Tw_cj = pKFj->getPoseInverse();
                                    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vector_dist =
                                        Tc_cj.translation();
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if (nNumKFs < 3)
                            {
                                vnStage[index]        = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if (nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj     = numProjOptMatches;
                                nBestNumCoindicendes   = nNumKFs;
                                pBestMatchedKF         = pMostBoWMatchesKF;
                                g2oBestScw             = gScw;
                                vpBestMapPoints        = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }
                        }
                    }
                }
            }
        }
        index++;
    }

    if (nBestMatchesReproj > 0)
    {
        pLastCurrentKF   = p_currentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2      = pBestMatchedKF;
        pMatchedKF2->setNotErase();
        g2oScw       = g2oBestScw;
        vpMPs        = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for (int i = 0; i < vnStage.size(); ++i)
        {
            if (vnStage[i] > maxStage)
            {
                maxStage   = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
    }
    return false;
}

bool LoopClosing::detectCommonRegionsFromLastKF(
    KeyFrame                *pCurrentKF,
    KeyFrame                *pMatchedKF,
    g2o::Sim3               &gScw,
    int                     &nNumProjMatches,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    set<MapPoint *> spAlreadyMatchedMPs(vpMatchedMPs.begin(),
                                        vpMatchedMPs.end());
    nNumProjMatches = findMatchesByProjection(pCurrentKF,
                                              pMatchedKF,
                                              gScw,
                                              spAlreadyMatchedMPs,
                                              vpMPs,
                                              vpMatchedMPs);

    int nProjMatches = 30;
    if (nNumProjMatches >= nProjMatches)
    {
        return true;
    }

    return false;
}

int LoopClosing::findMatchesByProjection(KeyFrame        *pCurrentKF,
                                         KeyFrame        *pMatchedKFw,
                                         g2o::Sim3       &g2oScw,
                                         set<MapPoint *> &spMatchedMPinOrigin,
                                         vector<MapPoint *> &vpMapPoints,
                                         vector<MapPoint *> &vpMatchedMapPoints)
{
    int                nNumCovisibles = 10;
    vector<KeyFrame *> vpCovKFm =
        pMatchedKFw->getBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame *> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame *> spCurrentCovisbles = pCurrentKF->getConnectedKeyFrames();
    if (nInitialCov < nNumCovisibles)
    {
        for (int i = 0; i < nInitialCov; ++i)
        {
            vector<KeyFrame *> vpKFs =
                vpCovKFm[i]->getBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j         = 0;
            while (j < vpKFs.size() && nInserted < nNumCovisibles)
            {
                if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() &&
                    spCurrentCovisbles.find(vpKFs[j]) ==
                        spCurrentCovisbles.end())
                {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPoint *> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for (KeyFrame *pKFi : vpCovKFm)
    {
        for (MapPoint *pMPij : pKFi->getMapPointMatches())
        {
            if (!pMPij || pMPij->isBad())
                continue;

            if (spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    Sophus::Sim3f correctedPose = utils::converter::Converter::toSophus(g2oScw);
    ORBmatcher    matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->getMapPointMatches().size(),
                              static_cast<MapPoint *>(nullptr));
    int num_matches = matcher.searchByProjection(pCurrentKF,
                                                 correctedPose,
                                                 vpMapPoints,
                                                 vpMatchedMapPoints,
                                                 3,
                                                 1.5);

    return num_matches;
}

void LoopClosing::correctLoop()
{
    // Avoid new keyframes are inserted while correcting the loop
    p_localMapper->requestStop();
    p_localMapper->emptyQueue();

    /* Stop and reclaim any global bundle-adjustment worker before mutation. */
    stopGlobalBundleAdjustment();

    // Wait until Local Mapping has effectively stopped
    while (!p_localMapper->isStopped())
        usleep(1000);

    // Ensure current keyframe is updated
    p_currentKF->updateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected
    // Sim3 pose by propagation
    currentConnectedKFs = p_currentKF->getVectorCovisibleKeyFrames();
    currentConnectedKFs.push_back(p_currentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[p_currentKF] = mg2oLoopScw;
    Sophus::SE3f Twc           = p_currentKF->getPoseInverse();
    Sophus::SE3f Tcw           = p_currentKF->getPose();
    g2o::Sim3    g2oScw(Tcw.unit_quaternion().cast<double>(),
                     Tcw.translation().cast<double>(),
                     1.0);
    NonCorrectedSim3[p_currentKF] = g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3
    // (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),
                              mg2oLoopScw.translation() / mg2oLoopScw.scale());
    p_currentKF->setPose(correctedTcw.cast<float>());

    Map *pLoopMap = p_currentKF->getMap();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFusion =
        std::chrono::steady_clock::now();
#endif

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                          vend = currentConnectedKFs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *pKFi = *vit;

            if (pKFi != p_currentKF)
            {
                Sophus::SE3f Tiw = pKFi->getPose();
                Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                // Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi] = g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform
                // Sim3 to SE3 (scale translation)
                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                          g2oCorrectedSiw.translation() /
                                              g2oCorrectedSiw.scale());
                pKFi->setPose(correctedTiw.cast<float>());

                // Pose without correction
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),
                                 Tiw.translation().cast<double>(),
                                 1.0);
                NonCorrectedSim3[pKFi] = g2oSiw;
            }
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so
        // that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit  = CorrectedSim3.begin(),
                                       mend = CorrectedSim3.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi            = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            // Update keyframe pose with corrected Sim3. First transform Sim3 to
            // SE3 (scale translation)
            /*Sophus::SE3d
            correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation()
            / g2oCorrectedSiw.scale());
            pKFi->setPose(correctedTiw.cast<float>());*/

            vector<MapPoint *> vpMPsi = pKFi->getMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                MapPoint *pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;
                if (pMPi->isBad())
                    continue;
                if (pMPi->correctedByKeyFrameId == p_currentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with
                // corrected pose
                Eigen::Vector3d P3Dw = pMPi->getWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw =
                    g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                pMPi->setWorldPos(eigCorrectedP3Dw.cast<float>());
                pMPi->correctedByKeyFrameId        = p_currentKF->mnId;
                pMPi->correctedReferenceKeyFrameId = pKFi->mnId;
                pMPi->updateNormalAndDepth();
            }

            // Correct velocity according to orientation correction
            if (bImuInit)
            {
                Eigen::Quaternionf Rcor =
                    (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation())
                        .cast<float>();
                pKFi->setVelocity(Rcor * pKFi->getVelocity());
            }

            // Make sure connections are updated
            pKFi->updateConnections();
        }
        // TODO Check this index increasement
        p_atlas->getCurrentMap()->increaseChangeIndex();

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < loopMatchedMPs.size(); i++)
        {
            if (loopMatchedMPs[i])
            {
                MapPoint *pLoopMP = loopMatchedMPs[i];
                MapPoint *pCurMP  = p_currentKF->getMapPoint(i);
                if (pCurMP)
                    pCurMP->replace(pLoopMP);
                else
                {
                    p_currentKF->addMapPoint(pLoopMP, i);
                    pLoopMP->addObservation(p_currentKF, i);
                    pLoopMP->computeDistinctiveDescriptors();
                }
            }
        }
        // cout << "LC: end replacing duplicated" << endl;
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    searchAndFuse(CorrectedSim3, loopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will
    // appear attaching both sides of the loop
    map<KeyFrame *, set<KeyFrame *>> LoopConnections;

    for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                      vend = currentConnectedKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame          *pKFi = *vit;
        vector<KeyFrame *> vpPreviousNeighbors =
            pKFi->getVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->updateConnections();
        LoopConnections[pKFi] = pKFi->getConnectedKeyFrames();
        for (vector<KeyFrame *>::iterator
                 vit_prev  = vpPreviousNeighbors.begin(),
                 vend_prev = vpPreviousNeighbors.end();
             vit_prev != vend_prev;
             vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for (vector<KeyFrame *>::iterator vit2  = currentConnectedKFs.begin(),
                                          vend2 = currentConnectedKFs.end();
             vit2 != vend2;
             vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = fixScale;
    // TODO CHECK; Solo para el monocular inertial
    if (p_tracker->sensor == System::IMU_MONOCULAR &&
        !p_currentKF->getMap()->getInertialBA2())
        bFixedScale = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndFusion =
        std::chrono::steady_clock::now();

    double timeFusion =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndFusion - time_StartFusion)
            .count();
    vdLoopFusion_ms.push_back(timeFusion);
#endif
    // cout << "Optimize essential graph" << endl;
    if (pLoopMap->isInertial() && pLoopMap->isImuInitialized())
    {
        Optimizer::optimizeEssentialGraph4DoF(pLoopMap,
                                              p_loopMatchedKF,
                                              p_currentKF,
                                              NonCorrectedSim3,
                                              CorrectedSim3,
                                              LoopConnections);
    }
    else
    {
        // cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
        Optimizer::optimizeEssentialGraph(pLoopMap,
                                          p_loopMatchedKF,
                                          p_currentKF,
                                          NonCorrectedSim3,
                                          CorrectedSim3,
                                          LoopConnections,
                                          bFixedScale);
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndOpt =
        std::chrono::steady_clock::now();

    double timeOptEss =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndOpt - time_EndFusion)
            .count();
    vdLoopOptEss_ms.push_back(timeOptEss);
#endif

    p_atlas->informNewBigChange();

    // Add loop edge
    p_loopMatchedKF->addLoopEdge(p_currentKF);
    p_currentKF->addLoopEdge(p_loopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few
    // keyframes, if not it would take too much time)
    if (!pLoopMap->isImuInitialized() ||
        (pLoopMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1))
    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
        runningGBA    = true;
        finishedGBA   = false;
        correctionGBA = numCorrection;
        globalBundleAdjustmentStopRequested.store(false,
                                                  std::memory_order_release);

        p_threadGBA = new thread(&LoopClosing::runGlobalBundleAdjustment,
                                 this,
                                 pLoopMap,
                                 p_currentKF->mnId,
                                 fullBundleAdjustmentIndex);
    }

    // Loop closed. Release Local Mapping.
    p_localMapper->release();

    lastLoopKeyFrameId =
        p_currentKF
            ->mnId; // TODO old varible, it is not use in the new algorithm
}

semantic::SemanticMergeDecision LoopClosing::mergeLocal()
{
    /* ---------------------------------------------------------------------- *
     * SECTION 1 - INITIALISATION
     *
     * Merge Policy
     * ------------------------------------------------------------------------
     * The current map is treated as the authoritative map and therefore
     * survives the merge. The matched map is transformed into the coordinate
     * frame of the current map before all of its contents are appended into
     * the current map. Once all objects have been transferred, the merge map
     * is marked as bad and removed from the atlas.
     * ---------------------------------------------------------------------- */

    /* Constant used to determine the number of temporal keyframes */
    constexpr int kNumTemporalKFs = 25;

    /* Extract the system parameters */
    p_sysParams = types::SystemParams::getParams();

    /* Reject stale place-recognition candidates before stopping other workers.
     */
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        p_currentKF->isBad() || p_mergeMatchedKF->isBad())
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    Map *pCurrentMap = p_currentKF->getMap();
    Map *pMergeMap   = p_mergeMatchedKF->getMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->isBad() ||
        pMergeMap->isBad() || !p_atlas->isActiveMap(pCurrentMap) ||
        !p_atlas->isActiveMap(pMergeMap))
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 2 - STOP GLOBAL BUNDLE ADJUSTMENT
     *
     * The merge modifies map ownership, poses and graph connectivity.
     * Therefore no optimisation thread is allowed to access these objects
     * while the merge is taking place.
     * ---------------------------------------------------------------------- */

    /* Flag to indicate if bundle adjustment should be relaunched */
    bool bRelaunchBA = false;

    bRelaunchBA = stopGlobalBundleAdjustment();

    /* ---------------------------------------------------------------------- *
     * SECTION 3 - STOP LOCAL MAPPING
     * ---------------------------------------------------------------------- */

    /* Request stop */
    p_localMapper->requestStop();

    /* Wait until local mapper stops */
    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 4 - IDENTIFY SOURCE AND DESTINATION MAPS
     *
     * pCurrentMap
     *      Survives the merge.
     *
     * pMergeMap
     *      Is transformed into the current-map frame before being appended
     *      into pCurrentMap.
     * ---------------------------------------------------------------------- */

    /*
     * Prevent semantic worker threads from modifying either graph while map
     * frames, ownership, and cross-entity references are being changed.
     */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    /* Revalidate after quiescing workers; retained retired maps keep stale
     * raw pointers alive, so pointer non-nullness alone is insufficient. */
    if (p_currentKF->isBad() || p_mergeMatchedKF->isBad() ||
        p_currentKF->getMap() != pCurrentMap ||
        p_mergeMatchedKF->getMap() != pMergeMap ||
        !p_atlas->isActiveMap(pCurrentMap) || !p_atlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        return semantic::SemanticMergeDecision::REJECT;
    }

    const Sophus::SE3d Twc = p_currentKF->getPoseInverse().cast<double>();
    const g2o::Sim3    g2oNonCorrectedSwc(Twc.unit_quaternion(),
                                       Twc.translation(),
                                       1.0);
    const g2o::Sim3    g2oSwCurrentWMerge = g2oNonCorrectedSwc * mg2oMergeScw;
    const g2o::Sim3    g2oSwMergeWCurrent = g2oSwCurrentWMerge.inverse();

    const semantic::SemanticMergeGateResult semanticMergeGate =
        semantic::SemanticVerify::evaluateMapMergeGate(
            pCurrentMap,
            pMergeMap,
            g2oSwCurrentWMerge,
            semantic::SemanticVerify::configFromSystemParams());
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId() << " decision="
              << semantic::SemanticVerify::mergeDecisionName(
                     semanticMergeGate.decision)
              << " reason="
              << semantic::SemanticVerify::mergeReasonName(
                     semanticMergeGate.reason)
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return semanticMergeGate.decision;
    }

    /* Discard queued keyframes only after every merge rejection gate passed. */
    p_localMapper->emptyQueue();

    /* Update the connections of the current keyframe */
    p_currentKF->updateConnections();

    /* ---------------------------------------------------------------------- *
     * SECTION 5 - BUILD THE CURRENT-MAP LOCAL WINDOW
     *
     * This window forms the fixed reference side of the weld.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> spLocalWindowKFs;
    std::set<MapPoint *> spLocalWindowMPs;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (pCurrentMap->isInertial() && pMergeMap->isInertial())
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        KeyFrame *pKFi      = p_currentKF;
        int       nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->getMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->p_prevKF;
            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        pKFi      = p_currentKF->p_nextKF;
        nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->getMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->p_nextKF;
            nInserted++;
        }
    }
    else
    {
        spLocalWindowKFs.insert(p_currentKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> vpCovisibleKFs =
        p_currentKF->getBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spLocalWindowKFs.insert(p_currentKF);

    constexpr int kMaxExpansionIterations = 5;

    int nExpansion = 0;

    /*!
     * If there is not enough keyframes in the local window, then we look at the
     * keyframes currently in spLocalWindowKF and find there best covisibility
     * keyframes. We do this to gaurantee there is enough keyframes in the
     * `spLocalWindowKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spLocalWindowKFs`, if the current keyframe doesn't
     * have enough covisible keyframes attached to it.
     */
    while (spLocalWindowKFs.size() < kNumTemporalKFs &&
           nExpansion < kMaxExpansionIterations)
    {
        std::vector<KeyFrame *> vpNewCovisible;

        for (KeyFrame *pKFi : spLocalWindowKFs)
        {
            const auto vpCovisible =
                pKFi->getBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

            for (KeyFrame *pKFcov : vpCovisible)
            {
                if (!pKFcov)
                    continue;

                if (pKFcov->isBad())
                    continue;

                if (spLocalWindowKFs.count(pKFcov))
                    continue;

                vpNewCovisible.push_back(pKFcov);
            }
        }

        spLocalWindowKFs.insert(vpNewCovisible.begin(), vpNewCovisible.end());

        ++nExpansion;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the current-map welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *pKFi : spLocalWindowKFs)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const std::set<MapPoint *> spMPs = pKFi->getMapPoints();

        /* Insert all the map points into the map point local window */
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 6 - BUILD THE MERGE-MAP LOCAL WINDOW
     *
     * These keyframes will be transformed into the current-map frame before
     * being transferred into the surviving map. Essentially, repeat above
     * steps but with the merge map.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> spMergeConnectedKFs;
    std::set<MapPoint *> spMapPointMerge;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (pCurrentMap->isInertial() && pMergeMap->isInertial())
    {
        KeyFrame *pKFi      = p_mergeMatchedKF;
        int       nInserted = 0;

        /* ------------------------------------------------------------------ *
         * Walk backwards
         * ------------------------------------------------------------------ */

        while (pKFi && nInserted < (kNumTemporalKFs / 2))
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->p_prevKF;

            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards
         * ------------------------------------------------------------------ */

        pKFi = p_mergeMatchedKF->p_nextKF;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->p_nextKF;

            nInserted++;
        }
    }
    else
    {
        spMergeConnectedKFs.insert(p_mergeMatchedKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    vpCovisibleKFs =
        p_mergeMatchedKF->getBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spMergeConnectedKFs.insert(p_mergeMatchedKF);

    /* Reset counter */
    nExpansion = 0;

    /*!
     * If there is not enough keyframes in the merge connected window, then we
     * look at the keyframes currently in spMergeConnectedKFs and find there
     * best covisibility keyframes. We do this to gaurantee there is enough
     * keyframes in the `spMergeConnectedKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spMergeConnectedKFs`, if the current keyframe
     * doesn't have enough covisible keyframes attached to it.
     */
    while (spMergeConnectedKFs.size() < kNumTemporalKFs &&
           nExpansion < kMaxExpansionIterations)
    {
        std::vector<KeyFrame *> vpNewCovisible;

        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            const auto vpCovisible =
                pKFi->getBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

            for (KeyFrame *pKFcov : vpCovisible)
            {
                if (!pKFcov)
                    continue;

                if (pKFcov->isBad())
                    continue;

                if (spMergeConnectedKFs.count(pKFcov))
                    continue;

                vpNewCovisible.push_back(pKFcov);
            }
        }

        spMergeConnectedKFs.insert(vpNewCovisible.begin(),
                                   vpNewCovisible.end());

        ++nExpansion;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the imported welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const auto spMPs = pKFi->getMapPoints();

        /* Insert all the map points into the map point local window */
        spMapPointMerge.insert(spMPs.begin(), spMPs.end());
    }

    /*!
     * Search and fuse only accepts std::vector<MapPoint *>, not
     * std::set<MapPoint *>. Hence, need to move the `spLocalWindowMPs` to a
     * new object to enable search and fuse. With `spLocalWindowMPs` being a
     * set this does mean that `vpCheckFuseMapPoint` will have no duplicates.
     */

    /* Init list of fused map points */
    std::vector<MapPoint *> vpCheckFuseMapPoint;

    /* Reserve the memory for the fused map points */
    vpCheckFuseMapPoint.reserve(spLocalWindowMPs.size());

    /* Copy the map points from the local window */
    vpCheckFuseMapPoint.assign(spLocalWindowMPs.begin(),
                               spLocalWindowMPs.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 7 - COMPUTE THE MAP-TO-MAP SIMILARITY TRANSFORM
     *
     *
     * Purpose
     * ----------------------------------------------------------------------
     *
     * Compute the similarity transform required to express all geometry from
     * the merge-map world frame inside the surviving current-map world frame.
     *
     *
     * Coordinate Frames
     * ----------------------------------------------------------------------
     *
     *      Merge World -----> Current Camera -----> Current World
     *          |                 mg2oMergeScw            Twc
     *
     *
     * Result
     * ----------------------------------------------------------------------
     *
     * g2oSwCurrentWMerge :
     *      Maps merge-world coordinates into current-world coordinates.
     *
     * g2oSwMergeWCurrent :
     *      Inverse transform used when correcting imported keyframe poses.
     *
     *
     * ---------------------------------------------------------------------- */

    /* The map-to-map transforms were computed before the floor preflight so a
     * rejected merge could leave poses and ownership untouched. */

    /* ---------------------------------------------------------------------- *
     * SECTION 8 - CORRECT IMPORTED KEYFRAME POSES
     *
     * Every keyframe belonging to the merge-map welding window is transformed
     * into the current-map reference frame.
     *
     * During this stage no ownership changes occur. The corrected poses are
     * simply cached for later insertion into the surviving map. These
     * transforms are stored in: `vNonCorrectedSim3` and `vCorrectedSim3`.
     * ---------------------------------------------------------------------- */

    /* Stores merge side keyframe's original pose before applying correction */
    KeyFrameAndPose vNonCorrectedSim3;

    /* Stores the corrected merge keyframe pose */
    KeyFrameAndPose vCorrectedSim3;

    /* Iterate through every merge keyframe in the merge connected KF list */
    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip invalid keyframes */
        if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        const Sophus::SE3d TiwMerge = pKFi->getPose().cast<double>();

        /* Convert to a g2o::Sim3 object type */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Find the transform from current world to keyframe iteration (i) */
        const g2o::Sim3 g2oSiwCurrent = g2oSiwMerge * g2oSwMergeWCurrent;

        /* Store transforms */
        vNonCorrectedSim3[pKFi] = g2oSiwMerge;
        vCorrectedSim3[pKFi]    = g2oSiwCurrent;

        /* Find the scale of transform */
        const double s = g2oSiwCurrent.scale();

        /* Find the transform from merge to current map */
        pKFi->correctedScale = s;
        pKFi->tcwMerge       = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                      g2oSiwCurrent.translation() / s)
                             .cast<float>();

        /* If there is IMU, extract velocity */
        if (pCurrentMap->isImuInitialized())
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            pKFi->vwbMerge = Rcor * pKFi->getVelocity();
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 9 - CORRECT IMPORTED MAP POINTS
     *
     * Transform every imported landmark into the current-map coordinate frame.
     *
     * Position:
     *      Full Sim3 transformation.
     *
     * Normal:
     *      Rotation only.
     *
     * Invalid landmarks are removed from the temporary welding set.
     * ---------------------------------------------------------------------- */

    /* Iterate through all the mapped points in the merge map */
    for (auto itMP = spMapPointMerge.begin(); itMP != spMapPointMerge.end();)
    {
        /* Copy the map points */
        MapPoint *pMPi = *itMP;

        /* If the mapped points are invalud, erase and skip */
        if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
        {
            itMP = spMapPointMerge.erase(itMP);
            continue;
        }

        /* Extract position of point */
        const Eigen::Vector3d P3DwMerge = pMPi->getWorldPos().cast<double>();

        /* Transform the point into the current map world frame */
        pMPi->posMerge = g2oSwCurrentWMerge.map(P3DwMerge).cast<float>();

        /* Transform the points surface normal into current map world frame */
        pMPi->normalVectorMerge =
            g2oSwCurrentWMerge.rotation().cast<float>() * pMPi->getNormal();

        /* Step to next mapped point */
        itMP++;
    }

    /* Existing current-map landmarks are used as fusion candidates. */
    vpCheckFuseMapPoint.assign(spLocalWindowMPs.begin(),
                               spLocalWindowMPs.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 10 - TRANSFER THE WELDING WINDOW
     *
     * Both maps are locked simultaneously using std::scoped_lock to guarantee a
     * deadlock-free ownership transfer.
     *
     * The current map remains the authoritative map throughout this section.
     * ---------------------------------------------------------------------- */
    {
        /*!
         * Lock both maps with deadlock-safe acquisition while ownership moves
         */
        std::scoped_lock mapLocks(pCurrentMap->mMutexMapUpdate,
                                  pMergeMap->mMutexMapUpdate);

        /* ------------------------------------------------------------------ *
         * SECTION 11 - TRANSFER CORRECTED KEYFRAMES
         *
         * Every corrected merge-map keyframe is:
         *
         *      1. Pose corrected.
         *      2. Assigned to the current map.
         *      3. Added to the current-map container.
         *      4. Removed from the merge-map container.
         * ------------------------------------------------------------------ */

        /* For every keyframe in merge map, iterate through and transfer */
        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            /* Skip invalud keyframes */
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
            {
                continue;
            }

            /* Store the old pose of the keyframe */
            pKFi->tcwBefMerge = pKFi->getPose();
            pKFi->twcBefMerge = pKFi->getPoseInverse();

            /* Apply corrected world-to-camera pose in the current-map frame */
            pKFi->setPose(pKFi->tcwMerge);

            /* Change keyframe's internal owning-map pointer to current map */
            pKFi->updateMap(pCurrentMap);

            /* Record which current keyframe triggered this merge correction */
            pKFi->mergeCorrectedKeyFrameId = p_currentKF->mnId;

            /* Insert the same keyframe pointer into surviving map container */
            pCurrentMap->addKeyFrame(pKFi);

            /* Remove the keyframe pointer from the old merge-map container */
            pMergeMap->eraseKeyFrame(pKFi);

            /* If there is IMU, add velocity */
            if (pCurrentMap->isImuInitialized())
            {
                pKFi->setVelocity(pKFi->vwbMerge);
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 12 - TRANSFER CORRECTED MAP POINTS
         *
         * The imported landmarks have already been transformed into the
         * current-map frame and therefore only require ownership transfer.
         * ------------------------------------------------------------------ */

        /* Iterate over every merge-map point selected for transfer */
        for (MapPoint *pMPi : spMapPointMerge)
        {
            /* Skip null, invalid, or no-longer merge-owned map points */
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
            {
                continue;
            }

            /* Apply position expressed in the surviving current-map frame */
            pMPi->setWorldPos(pMPi->posMerge);

            /* Apply the normal rotated into the surviving current-map frame */
            pMPi->setNormalVector(pMPi->normalVectorMerge);

            /* Change the map point's internal owner to the current map */
            pMPi->updateMap(pCurrentMap);

            /* Register the same map-point pointer in the surviving map */
            pCurrentMap->addMapPoint(pMPi);

            /* Remove the map-point pointer from the obsolete merge map */
            pMergeMap->eraseMapPoint(pMPi);
        }

        /* Set the map to be the current map */
        p_atlas->changeMap(pCurrentMap);

        /* Incrase index tracking the amount of times the maps been changed */
        pCurrentMap->increaseChangeIndex();
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 13 - REBUILD THE IMPORTED SPANNING TREE
     *
     * The imported spanning tree is attached beneath the surviving current
     * keyframe before the previous parent chain is reversed.
     *
     * This preserves graph connectivity while preventing cyclic parent
     * relationships.
     *
     * The links are updated so the imported merge-map keyframes belong to one
     * connected keyframe graph rooted in the surviving current map. You first
     * transfer and reconnect the merge-map keyframes into the current map’s
     * keyframe graph, then (IN SECTION 14) recompute the covisibility graph
     * from their shared map-point observations
     * ---------------------------------------------------------------------- */

    /* If the oriign keyframe of the merp map is valid */
    if (pMergeMap->getOriginKeyFrame())
    {
        /* Allow the former merge-map root to become a normal tree child */
        pMergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }

    /* Init variables to track the new child and parent keyframes */
    KeyFrame *pNewChild  = nullptr;
    KeyFrame *pNewParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    pNewChild = p_mergeMatchedKF->getParent();

    /* The matched merge keyframe becomes the first reversed parent */
    pNewParent = p_mergeMatchedKF;

    /* Attach the matched merge keyframe beneath the current keyframe */
    p_mergeMatchedKF->changeParent(p_currentKF);

    /* Reverse each edge along the original merge-map parent chain */
    while (pNewChild)
    {
        /* Remove the old child edge before reversing its direction */
        pNewChild->eraseChild(pNewParent);

        /* Save the next original parent before changing this relation */
        KeyFrame *pOldParent = pNewChild->getParent();

        /* Make the former parent a child of the previous keyframe */
        pNewChild->changeParent(pNewParent);

        /* Advance the new-parent pointer one level up the old chain */
        pNewParent = pNewChild;

        /* Continue with the next parent from the original tree chain */
        pNewChild = pOldParent;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 14 - REFRESH COVISIBILITY GRAPH
     *
     * Refresh the imported covisibility graph before searching for duplicate
     * landmarks between the two welding windows.
     * ---------------------------------------------------------------------- */

    /* Refresh links for the matched merge-side keyframe */
    p_mergeMatchedKF->updateConnections();

    /* Init list of connected keyframes in merge map */
    std::vector<KeyFrame *> vpMergeConnectedKFs;

    /* Retrieve keyframes covisible with the matched merge keyframe */
    vpMergeConnectedKFs = p_mergeMatchedKF->getVectorCovisibleKeyFrames();

    /* Include the matched merge keyframe in the fusion set */
    vpMergeConnectedKFs.push_back(p_mergeMatchedKF);

    /* Fuse duplicate current-map points into corrected merge keyframes */
    searchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    /* Refresh covisibility links for current-map local keyframes */
    for (KeyFrame *pKFi : spLocalWindowKFs)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        pKFi->updateConnections();
    }

    /* Refresh covisibility links for imported merge-side keyframes */
    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        pKFi->updateConnections();
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 15 - LOCAL MERGE OPTIMISATION
     *
     * Perform a local optimisation immediately after the welding window has
     * been merged.
     *
     * Depending on the sensor configuration either:
     *
     *      • MergeInertialBA()
     *      • LoopClosureLocalBundleAdjustment()
     *
     * is executed.
     * ---------------------------------------------------------------------- */

    /* Shared stop flag passed to the selected optimisation routine */
    bool bStop = false;

    /* Init list of local keyframes in current window */
    std::vector<KeyFrame *> vpLocalCurrentWindowKFs;

    /* Remove keyframes stored by any previous merge operation */
    vpLocalCurrentWindowKFs.clear();

    /* Remove merge-connected keyframes stored by earlier processing */
    vpMergeConnectedKFs.clear();

    /* Copy current-side local keyframes into the optimiser vector */
    std::copy(spLocalWindowKFs.begin(),
              spLocalWindowKFs.end(),
              std::back_inserter(vpLocalCurrentWindowKFs));

    /* Copy merge-side connected keyframes into the optimiser vector */
    std::copy(spMergeConnectedKFs.begin(),
              spMergeConnectedKFs.end(),
              std::back_inserter(vpMergeConnectedKFs));

    /* Check whether the active sensor configuration includes an IMU */
    if (p_tracker->sensor == System::IMU_MONOCULAR ||
        p_tracker->sensor == System::IMU_STEREO ||
        p_tracker->sensor == System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        Optimizer::mergeInertialBA(p_currentKF,
                                   p_mergeMatchedKF,
                                   &bStop,
                                   pCurrentMap,
                                   vCorrectedSim3);
    }
    else
    {
        /* Refine the merged region using visual observations only */
        Optimizer::loopClosureLocalBundleAdjustment(p_mergeMatchedKF,
                                                    vpMergeConnectedKFs,
                                                    vpLocalCurrentWindowKFs,
                                                    &bStop);
    }

    /* Resume local mapping after merge optimisation is complete */
    p_localMapper->release();

    /* ---------------------------------------------------------------------- *
     * SECTION 16 - RETRIEVE THE REMAINING MERGE MAP
     *
     * The local welding window has already been transferred.
     *
     * This section retrieves every remaining object that still belongs to the
     * obsolete merge map.
     * ---------------------------------------------------------------------- */

    /* Copy all planes currently owned by the merge map */
    std::vector<geometric::Plane *> vpCurrentMapPlanes =
        pMergeMap->getAllPlanes();

    /* Copy all keyframes currently owned by the merge map */
    std::vector<KeyFrame *> vpCurrentMapKFs = pMergeMap->getAllKeyFrames();

    const bool hasValidRemainingMergeKeyFrame =
        std::any_of(vpCurrentMapKFs.begin(),
                    vpCurrentMapKFs.end(),
                    [pMergeMap](KeyFrame *p_keyFrame_in)
                    {
                        return p_keyFrame_in != nullptr &&
                               !p_keyFrame_in->isBad() &&
                               p_keyFrame_in->getMap() == pMergeMap;
                    });

    /* Copy all map points currently owned by the merge map */
    std::vector<MapPoint *> vpCurrentMapMPs = pMergeMap->getAllMapPoints();

    /* Copy all markers currently owned by the merge map */
    std::vector<semantic::Marker *> vpCurrentMapMarkers =
        pMergeMap->getAllMarkers();

    /* Copy all passages currently owned by the merge map */
    std::vector<vs_graphs::core::semantic::Passage *> vpCurrentMapPassages =
        pMergeMap->getAllPassages();

    /* Copy all detected rooms currently owned by the merge map */
    std::vector<semantic::Room *> vpCurrentDetectedMapRooms =
        pMergeMap->getAllDetectedMapRooms();

    /* Copy all marker-based rooms currently owned by the merge map */
    std::vector<semantic::Room *> vpCurrentMarkerBasedMapRooms =
        pMergeMap->getAllMarkerBasedMapRooms();

    /* Copy all floors currently owned by the merge map */
    std::vector<semantic::Floor *> vpCurrentMapFloors =
        pMergeMap->getAllFloors();

    /* Stop local mapping before any remaining ownership is transferred. */
    p_localMapper->requestStop();

    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 17 - CORRECT REMAINING MERGE-MAP GEOMETRY
     *
     * Every remaining keyframe and landmark that was not part of the welding
     * window is transformed into the surviving current-map frame prior to graph
     * optimisation.
     * ---------------------------------------------------------------------- */

    /* Process remaining keyframes only when the merge map is not empty */
    if (hasValidRemainingMergeKeyFrame)
    {
        /* Apply monocular scale correction to the remaining merge map */
        if (p_tracker->sensor == System::MONOCULAR)
        {
            /* Lock the merge map while updating its poses and landmarks */
            std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);

            /* Correct each remaining merge keyframe into current world */
            for (KeyFrame *pKFi : vpCurrentMapKFs)
            {
                /* Skip invalid keyframes or keyframes no longer in this map */
                if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the keyframe pose in the merge map world frame */
                const Sophus::SE3d TiwMerge = pKFi->getPose().cast<double>();

                /* Convert the rigid keyframe pose into a unit scale Sim3 */
                const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                            TiwMerge.translation(),
                                            1.0);

                /* Express the keyframe pose in the current world frame */
                const g2o::Sim3 g2oSiwCurrent =
                    g2oSiwMerge * g2oSwMergeWCurrent;

                /* Store the original keyframe pose before correction */
                vNonCorrectedSim3[pKFi] = g2oSiwMerge;

                /* Store the corrected keyframe pose for later processing */
                vCorrectedSim3[pKFi] = g2oSiwCurrent;

                /* Extract the scale introduced by the map correction */
                const double s = g2oSiwCurrent.scale();

                /* Store the applied scale in the keyframe */
                pKFi->correctedScale = s;

                /* Preserve the original world to camera pose */
                pKFi->tcwBefMerge = pKFi->getPose();

                /* Preserve the original camera to world pose */
                pKFi->twcBefMerge = pKFi->getPoseInverse();

                /* Apply the corrected rigid pose in the current world frame */
                pKFi->setPose(Sophus::SE3d(g2oSiwCurrent.rotation(),
                                           g2oSiwCurrent.translation() / s)
                                  .cast<float>());

                /* Rotate velocity when the surviving map uses inertial data */
                if (pCurrentMap->isImuInitialized())
                {
                    /* Compute the rotation from old to corrected world frame */
                    const Eigen::Quaternionf Rcor =
                        (g2oSiwCurrent.rotation().inverse() *
                         g2oSiwMerge.rotation())
                            .cast<float>();

                    /* Express the keyframe velocity in the corrected frame */
                    pKFi->setVelocity(Rcor * pKFi->getVelocity());
                }
            }

            /* Correct each remaining merge landmark into current world */
            for (MapPoint *pMPi : vpCurrentMapMPs)
            {
                /* Skip invalid points or points no longer in this map */
                if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the landmark position in the merge world frame */
                const Eigen::Vector3d P3DwMerge =
                    pMPi->getWorldPos().cast<double>();

                const Eigen::Vector3f normal_mergeWorld = pMPi->getNormal();

                /* Transform the landmark into the current world frame */
                pMPi->setWorldPos(
                    g2oSwCurrentWMerge.map(P3DwMerge).cast<float>());

                pMPi->setNormalVector(
                    g2oSwCurrentWMerge.rotation().cast<float>() *
                    normal_mergeWorld);

                /* Refresh the point normal and valid viewing depth range */
                pMPi->updateNormalAndDepth();
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 18 - OPTIMISE THE COMPLETE MERGED GRAPH
         *
         * Once every remaining object has been corrected into the current-map
         * frame, optimise the complete merged graph before changing ownership.
         * ------------------------------------------------------------------ */

        if (p_tracker->sensor != System::MONOCULAR)
        {
            Optimizer::optimizeEssentialGraph(p_mergeMatchedKF,
                                              pMergeMap,
                                              vpLocalCurrentWindowKFs,
                                              vpMergeConnectedKFs,
                                              vpCurrentMapKFs,
                                              vpCurrentMapMPs,
                                              g2oSwCurrentWMerge);

            /*!
             * Skeleton topology is derived from the live Voxblox volume. The
             * map-revision notification invalidates that volume after this
             * correction, so no stale topology snapshot is retained here.
             */
        }
    }

    const bool semanticGeometryWasOptimized =
        hasValidRemainingMergeKeyFrame &&
        p_tracker->sensor != System::MONOCULAR;

    bool semanticGeometryWasPropagated = semanticGeometryWasOptimized;

    /*!
     * Small source maps and monocular merges do not run the merge essential
     * graph. Propagate the final welding-BA pose deltas directly so semantic
     * geometry follows the corrected imported keyframes rather than receiving
     * only the coarse map-level Sim3.
     */
    if (!semanticGeometryWasOptimized)
    {
        KeyFrameAndPose finalKeyFramePoses_WorldToCamera;

        for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
             vNonCorrectedSim3)
        {
            (void)poseBefore_WorldToCamera;

            if (p_keyFrame == nullptr || p_keyFrame->isBad())
            {
                continue;
            }

            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->getPose().cast<double>();

            /*
             * A monocular map merge can change scale. ORB-SLAM stores the
             * corrected keyframe as SE3 by dividing the Sim3 translation by
             * its scale, so reconstruct the corresponding Sim3 before
             * deriving the semantic world-frame correction. Using a unit
             * scale here would leave planes, rooms and passages at their old
             * size while MapPoints receive the full map Sim3.
             */
            double poseAfterScale = 1.0;

            const auto correctedPoseIterator = vCorrectedSim3.find(p_keyFrame);

            if (correctedPoseIterator != vCorrectedSim3.end() &&
                std::isfinite(correctedPoseIterator->second.scale()) &&
                std::abs(correctedPoseIterator->second.scale()) > 1e-12)
            {
                poseAfterScale = correctedPoseIterator->second.scale();
            }

            finalKeyFramePoses_WorldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                          poseAfterScale *
                              poseAfter_WorldToCamera.translation(),
                          poseAfterScale));
        }

        utils::utils::Utils::propagateSemanticPoseCorrections(
            pMergeMap,
            vNonCorrectedSim3,
            finalKeyFramePoses_WorldToCamera,
            g2oSwCurrentWMerge);

        semanticGeometryWasPropagated = true;
    }

    /*
     * A map can retain orphan landmarks after every keyframe in its local
     * window has already moved. Those points still require the map-level Sim3
     * even though there is no essential graph left to optimize.
     */
    if (!hasValidRemainingMergeKeyFrame)
    {
        for (MapPoint *p_mapPoint : vpCurrentMapMPs)
        {
            if (p_mapPoint == nullptr || p_mapPoint->isBad() ||
                p_mapPoint->getMap() != pMergeMap)
            {
                continue;
            }

            const Eigen::Vector3f position_mergeWorld_m =
                p_mapPoint->getWorldPos();

            const Eigen::Vector3f normal_mergeWorld = p_mapPoint->getNormal();

            p_mapPoint->setWorldPos(
                g2oSwCurrentWMerge.map(position_mergeWorld_m.cast<double>())
                    .cast<float>());

            p_mapPoint->setNormalVector(
                g2oSwCurrentWMerge.rotation().cast<float>() *
                normal_mergeWorld);

            p_mapPoint->updateNormalAndDepth();
        }
    }

    /* ------------------------------------------------------------------ *
     * SECTION 19 - TRANSFER THE REMAINING MAP CONTENTS
     *
     * Every remaining object stored inside the obsolete merge map is
     * transferred into the surviving current map.
     *
     * Object Types
     * ------------------------------------------------------------------
     *  - KeyFrames
     *  - MapPoints
     *  - Planes
     *  - Passages
     *  - Detected Rooms
     *  - Marker Rooms
     *  - Markers
     * ------------------------------------------------------------------ */
    {
        const bool primarySemanticGeometryWasCorrected =
            semanticGeometryWasOptimized || semanticGeometryWasPropagated;

        int nextPlaneId = 0;
        for (geometric::Plane *p_existingPlane : pCurrentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : pCurrentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                nextMarkerId =
                    std::max(nextMarkerId, p_existingMarker->getId() + 1);
            }
        }

        // Get Merge Map Mutex
        std::scoped_lock mapLocks(pCurrentMap->mMutexMapUpdate,
                                  pMergeMap->mMutexMapUpdate);

        // Loop over the KeyFrames of the current map and move them to the
        // new map
        for (KeyFrame *pKFi : vpCurrentMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
                continue;

            pKFi->updateMap(pCurrentMap);
            pCurrentMap->addKeyFrame(pKFi);
            pMergeMap->eraseKeyFrame(pKFi);
        }

        // Loop over the MapPoints of the current map and move them to the
        // new map
        for (MapPoint *pMPi : vpCurrentMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                continue;

            pMPi->updateMap(pCurrentMap);
            pCurrentMap->addMapPoint(pMPi);
            pMergeMap->eraseMapPoint(pMPi);
        }

        /* -------------------------------------------------------------- *
         * SECTION 20 - TRANSFER SEMANTIC OBJECTS
         *
         * Semantic objects are transformed into the current-map reference
         * frame before ownership is transferred.
         *
         * Geometry is preserved while map ownership is updated.
         * -------------------------------------------------------------- */
        for (geometric::Plane *plane : vpCurrentMapPlanes)
        {
            /* Skip invalid planes */
            if (plane == nullptr || plane->isBad())
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                plane->applyTransform(g2oSwCurrentWMerge);
            }

            /* Update the map the plane belongs to */
            plane->setMap(pCurrentMap);

            /*!
             * Take index size of planes in new map to find an id to add to
             * the map which hasn't been taken.
             */
            plane->setId(nextPlaneId++);

            /* Add the plane to the map new merged plane to the new map */
            pCurrentMap->addMapPlane(plane);

            /* Remove the current plane from the old map */
            pMergeMap->eraseMapPlane(plane);
        }

        // Loop over the Markers of the current map and move them to the new
        // map
        for (semantic::Marker *pMarker : vpCurrentMapMarkers)
        {
            if (!pMarker)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pMarker->applyTransform(g2oSwCurrentWMerge);
            }

            pMarker->setMap(pCurrentMap);
            pMarker->setId(nextMarkerId++);
            pCurrentMap->addMapMarker(pMarker);
            pMergeMap->eraseMapMarker(pMarker);
        }

        /*!
         * Loop over the passages of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Passage *passage : vpCurrentMapPassages)
        {
            /* Skip invalid rooms */
            if (passage == nullptr)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                passage->applyTransform(g2oSwCurrentWMerge);
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 pCurrentMap->getAllPassages())
            {
                if (p_existingPassage != nullptr &&
                    p_existingPassage->getId() == passage->getId())
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            pMergeMap->eraseMapPassage(passage);
            if (p_retainedPassage != nullptr)
            {
                p_retainedPassage->mergeFromDuplicate(passage);
                for (semantic::Room *p_room : vpCurrentDetectedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(passage,
                                                          p_retainedPassage);
                    }
                }
                for (semantic::Room *p_room : vpCurrentMarkerBasedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(passage,
                                                          p_retainedPassage);
                    }
                }
                passage->setBad();
                continue;
            }

            passage->setMap(pCurrentMap);
            pCurrentMap->addMapPassage(passage);
        }

        /*!
         * Loop over the rooms of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Room *room : vpCurrentDetectedMapRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                room->applyTransform(g2oSwCurrentWMerge);
            }

            /* Set the map of the room in the current map */
            room->setMap(pCurrentMap);

            /* Add the room to the current map */
            pCurrentMap->addDetectedMapRoom(room);

            /* Remove the room from the merged map */
            pMergeMap->eraseDetectedMapRoom(room);
        }

        // Loop over the Marker-based Rooms of the current map and move them
        // to the new map
        for (vs_graphs::core::semantic::Room *pRoom :
             vpCurrentMarkerBasedMapRooms)
        {
            if (!pRoom)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pRoom->applyTransform(g2oSwCurrentWMerge);
            }

            pRoom->setMap(pCurrentMap);
            pCurrentMap->addCandidateMapRoom(pRoom);
            pMergeMap->eraseMarkerBasedMapRoom(pRoom);
        }

        for (semantic::Floor *p_floor : vpCurrentMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (!semanticGeometryWasPropagated)
            {
                p_floor->applyTransform(g2oSwCurrentWMerge);
            }
            pMergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor : pCurrentMap->getAllFloors())
            {
                if (p_existingFloor != nullptr &&
                    p_existingFloor->getId() == p_floor->getId())
                {
                    p_retainedFloor = p_existingFloor;
                    break;
                }
            }
            if (p_retainedFloor != nullptr)
            {
                mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor);
                continue;
            }
            p_floor->setMap(pCurrentMap);
            pCurrentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /*
         * Voxblox topology is derived from a TSDF/ESDF volume and is not an
         * independently mergeable landmark set. Appending snapshots from two
         * map frames creates disconnected duplicate edges and false wall
         * crossings. The external Voxblox node receives the map-revision event,
         * clears its volume, and supplies a fresh snapshot after reintegration.
         */
        pCurrentMap->setSkeletonClusterPoints({});
        pCurrentMap->setSkeletonEdges({});

        /* Rebuild imported room-wall index entries before fusion. */
        for (semantic::Room *p_room : pCurrentMap->getAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        /* Fuse only after every semantic relationship is visible. */
        if (p_sysParams->semSeg.reassociate.enabled)
        {
            utils::utils::Utils::reAssociateSemanticPlanes(p_atlas);
        }

        std::vector<semantic::Room *> importedRooms = vpCurrentDetectedMapRooms;
        importedRooms.insert(importedRooms.end(),
                             vpCurrentMarkerBasedMapRooms.begin(),
                             vpCurrentMarkerBasedMapRooms.end());

        /* Stable semantic identity reconciliation is a merge invariant, not
         * an optional geometry-reassociation feature. */
        utils::utils::Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap, importedRooms);

        if (p_sysParams->semSeg.reassociate.enabled)
        {
            utils::utils::Utils::reAssociateRooms(p_atlas);
            utils::utils::Utils::reAssociatePassages(p_atlas);
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 22 - FINALISE THE MERGE
     *
     * Merge Complete
     * ----------------------------------------------------------------------
     *      - Current map survives.
     *      - Merge map contains no remaining objects.
     *      - Atlas ownership updated.
     *      - Merge edge inserted.
     *      - Obsolete map removed from the atlas.
     *
     * After this point every surviving SLAM object belongs exclusively to
     * pCurrentMap.
     * ---------------------------------------------------------------------- */

    p_mergeMatchedKF->addMergeEdge(p_currentKF);
    p_currentKF->addMergeEdge(p_mergeMatchedKF);

    pCurrentMap->increaseChangeIndex();

    /*!
     * A map merge changes the world-frame poses of previously integrated
     * observations. Notify derived mapping consumers only after ownership,
     * semantic reconciliation, and graph connectivity are fully committed.
     * Voxblox uses this revision to discard TSDF/ESDF state expressed in the
     * pre-merge coordinate frame.
     */
    pCurrentMap->informNewBigChange();

    /* All surviving objects now belong to pCurrentMap. */
    p_atlas->changeMap(pCurrentMap);
    p_atlas->setMapBad(pMergeMap);
    p_atlas->removeBadMaps();

    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    semanticUpdateLock.unlock();
    p_localMapper->release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

semantic::SemanticMergeDecision LoopClosing::mergeLocalInertial()
{
    /* Reject stale place-recognition candidates before stopping workers */
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        p_currentKF->isBad() || p_mergeMatchedKF->isBad())
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    Map *pCurrentMap = p_currentKF->getMap();
    Map *pMergeMap   = p_mergeMatchedKF->getMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->isBad() ||
        pMergeMap->isBad() || !p_atlas->isActiveMap(pCurrentMap) ||
        !p_atlas->isActiveMap(pMergeMap))
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    int numTemporalKFs = 11; // [TODO] Set by parameter

    // Relationship to rebuild the essential graph, it is used two times, first
    // in the local window and later in the rest of the map
    KeyFrame *pNewChild;
    KeyFrame *pNewParent;

    vector<KeyFrame *> vpLocalCurrentWindowKFs;
    vector<KeyFrame *> vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // Flag that is true only when we stopped a running BA, in this case we need
    // relaunch at the end of the merge
    bool bRelaunchBA = false;

    /* Stop and reclaim GBA before either map changes frame or ownership. */
    bRelaunchBA = stopGlobalBundleAdjustment();

    p_localMapper->requestStop();

    // Wait until Local Mapping has effectively stopped
    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* Keep the inertial semantic transfer atomic for the complete merge. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    if (p_currentKF->isBad() || p_mergeMatchedKF->isBad() ||
        p_currentKF->getMap() != pCurrentMap ||
        p_mergeMatchedKF->getMap() != pMergeMap ||
        !p_atlas->isActiveMap(pCurrentMap) || !p_atlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        return semantic::SemanticMergeDecision::REJECT;
    }

    const semantic::SemanticMergeGateResult semanticMergeGate =
        semantic::SemanticVerify::evaluateMapMergeGate(
            pCurrentMap,
            pMergeMap,
            oldCorrectedPose.inverse(),
            semantic::SemanticVerify::configFromSystemParams());
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId() << " decision="
              << semantic::SemanticVerify::mergeDecisionName(
                     semanticMergeGate.decision)
              << " reason="
              << semantic::SemanticVerify::mergeReasonName(
                     semanticMergeGate.reason)
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return semanticMergeGate.decision;
    }

    {
        float        s_on = oldCorrectedPose.scale();
        Sophus::SE3f T_on(oldCorrectedPose.rotation().cast<float>(),
                          oldCorrectedPose.translation().cast<float>());

        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);

        p_localMapper->emptyQueue();

        std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
        bool bScaleVel = false;
        if (s_on != 1)
            bScaleVel = true;
        pCurrentMap->applyScaledRotation(T_on, s_on, bScaleVel);
        p_tracker->updateFrameIMU(s_on,
                                  p_currentKF->getImuBias(),
                                  p_tracker->getLastKeyFrame());

        std::chrono::steady_clock::time_point t3 =
            std::chrono::steady_clock::now();
    }

    const int numKFnew = pCurrentMap->getKeyFrameCount();

    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
         p_tracker->sensor == System::IMU_STEREO ||
         p_tracker->sensor == System::IMU_RGBD) &&
        !pCurrentMap->getInertialBA2())
    {
        /* Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::inertialOptimization(pCurrentMap, bg, ba);
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);
        p_tracker->updateFrameIMU(1.0f, b, p_tracker->getLastKeyFrame());

        /* Set map initialized */
        pCurrentMap->setInertialBA2();
        pCurrentMap->setInertialBA1();
        pCurrentMap->setImuInitialized();
    }

    /* Retain imported room identities for post-optimization reconciliation. */
    std::vector<semantic::Room *> importedRooms;

    /* Load KFs and MPs from merge map */
    {
        /*!
         * Acquire both map-update mutexes without imposing an unsafe order.
         *
         * @note        Get Merge Map Mutex and stop tracking.
         */
        std::scoped_lock mapUpdateLocks(pCurrentMap->mMutexMapUpdate,
                                        pMergeMap->mMutexMapUpdate);

        vector<KeyFrame *>         vpMergeMapKFs = pMergeMap->getAllKeyFrames();
        vector<MapPoint *>         vpMergeMapMPs = pMergeMap->getAllMapPoints();
        vector<geometric::Plane *> vpMergeMapPlanes = pMergeMap->getAllPlanes();
        vector<semantic::Marker *> vpMergeMapMarkers =
            pMergeMap->getAllMarkers();
        vector<vs_graphs::core::semantic::Passage *> vpMergeMapPassages =
            pMergeMap->getAllPassages();
        vector<semantic::Room *> vpMergeMapDetectedRooms =
            pMergeMap->getAllDetectedMapRooms();
        vector<semantic::Room *> vpMergeMapMarkerRooms =
            pMergeMap->getAllMarkerBasedMapRooms();
        vector<semantic::Floor *> vpMergeMapFloors = pMergeMap->getAllFloors();

        importedRooms = vpMergeMapDetectedRooms;
        importedRooms.insert(importedRooms.end(),
                             vpMergeMapMarkerRooms.begin(),
                             vpMergeMapMarkerRooms.end());

        for (KeyFrame *pKFi : vpMergeMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->updateMap(pCurrentMap);
            pCurrentMap->addKeyFrame(pKFi);
            pMergeMap->eraseKeyFrame(pKFi);
        }

        for (MapPoint *pMPi : vpMergeMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                continue;

            pMPi->updateMap(pCurrentMap);
            pCurrentMap->addMapPoint(pMPi);
            pMergeMap->eraseMapPoint(pMPi);
        }

        int nextPlaneId = 0;
        for (geometric::Plane *p_existingPlane : pCurrentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        for (geometric::Plane *p_plane : vpMergeMapPlanes)
        {
            if (p_plane == nullptr || p_plane->isBad())
            {
                continue;
            }

            p_plane->setMap(pCurrentMap);
            p_plane->setId(nextPlaneId++);
            pCurrentMap->addMapPlane(p_plane);
            pMergeMap->eraseMapPlane(p_plane);
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : pCurrentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                nextMarkerId =
                    std::max(nextMarkerId, p_existingMarker->getId() + 1);
            }
        }

        for (semantic::Marker *p_marker : vpMergeMapMarkers)
        {
            if (p_marker == nullptr)
            {
                continue;
            }

            p_marker->setMap(pCurrentMap);
            p_marker->setId(nextMarkerId++);
            pCurrentMap->addMapMarker(p_marker);
            pMergeMap->eraseMapMarker(p_marker);
        }

        for (vs_graphs::core::semantic::Passage *p_passage : vpMergeMapPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 pCurrentMap->getAllPassages())
            {
                if (p_existingPassage != nullptr &&
                    p_existingPassage->getId() == p_passage->getId())
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            pMergeMap->eraseMapPassage(p_passage);
            if (p_retainedPassage != nullptr)
            {
                p_retainedPassage->mergeFromDuplicate(p_passage);
                for (semantic::Room *p_room : vpMergeMapDetectedRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(p_passage,
                                                          p_retainedPassage);
                    }
                }
                for (semantic::Room *p_room : vpMergeMapMarkerRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(p_passage,
                                                          p_retainedPassage);
                    }
                }
                p_passage->setBad();
                continue;
            }

            p_passage->setMap(pCurrentMap);
            pCurrentMap->addMapPassage(p_passage);
        }

        for (semantic::Room *p_room : vpMergeMapDetectedRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            pCurrentMap->addDetectedMapRoom(p_room);
            pMergeMap->eraseDetectedMapRoom(p_room);
        }

        for (semantic::Room *p_room : vpMergeMapMarkerRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            pCurrentMap->addCandidateMapRoom(p_room);
            pMergeMap->eraseMarkerBasedMapRoom(p_room);
        }

        for (semantic::Floor *p_floor : vpMergeMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            pMergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor : pCurrentMap->getAllFloors())
            {
                if (p_existingFloor != nullptr &&
                    p_existingFloor->getId() == p_floor->getId())
                {
                    p_retainedFloor = p_existingFloor;
                    break;
                }
            }
            if (p_retainedFloor != nullptr)
            {
                mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor);
                continue;
            }
            p_floor->setMap(pCurrentMap);
            pCurrentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /* Rebuild derived free-space topology in the corrected map frame. */
        pCurrentMap->setSkeletonClusterPoints({});
        pCurrentMap->setSkeletonEdges({});

        for (semantic::Room *p_room : pCurrentMap->getAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame *> vpKFs = pCurrentMap->getAllKeyFrames();
        for (KeyFrame *pKFi : vpKFs)
        {
            Sophus::SE3d Tiw = (pKFi->getPose()).cast<double>();
            g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
    }

    if (pMergeMap->getOriginKeyFrame() != nullptr)
    {
        pMergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }
    pNewChild =
        p_mergeMatchedKF
            ->getParent(); // Old parent, it will be the new child of this KF
    pNewParent = p_mergeMatchedKF; // Old child, now it will be the parent of
                                   // its own parent(we need eliminate this KF
                                   // from children list in its old parent)
    p_mergeMatchedKF->changeParent(p_currentKF);
    while (pNewChild)
    {
        pNewChild->eraseChild(
            pNewParent); // We remove the relation between the old parent and
                         // the new for avoid loop
        KeyFrame *pOldParent = pNewChild->getParent();
        pNewChild->changeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild  = pOldParent;
    }

    vector<MapPoint *>
        vpCheckFuseMapPoint; // MapPoint vector from current map to allow to
                             // fuse duplicated points with the old map (merge)
    vector<KeyFrame *> vpCurrentConnectedKFs;

    mergeConnectedKFs.clear();
    mergeConnectedKFs.push_back(p_mergeMatchedKF);
    vector<KeyFrame *> aux = p_mergeMatchedKF->getVectorCovisibleKeyFrames();
    mergeConnectedKFs.insert(mergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mergeConnectedKFs.size() > 6)
        mergeConnectedKFs.erase(mergeConnectedKFs.begin() + 6,
                                mergeConnectedKFs.end());

    p_currentKF->updateConnections();
    vpCurrentConnectedKFs.push_back(p_currentKF);
    aux = p_currentKF->getVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(),
                                 aux.begin(),
                                 aux.end());
    if (vpCurrentConnectedKFs.size() > 6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6,
                                    vpCurrentConnectedKFs.end());

    set<MapPoint *> spMapPointMerge;
    for (KeyFrame *pKFi : mergeConnectedKFs)
    {
        set<MapPoint *> vpMPs = pKFi->getMapPoints();
        spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        if (spMapPointMerge.size() > 1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(),
              spMapPointMerge.end(),
              std::back_inserter(vpCheckFuseMapPoint));
    searchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for (KeyFrame *pKFi : vpCurrentConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->updateConnections();
    }

    const auto finalizeInertialMerge = [this, pCurrentMap, pMergeMap]()
    {
        p_mergeMatchedKF->addMergeEdge(p_currentKF);
        p_currentKF->addMergeEdge(p_mergeMatchedKF);
        pCurrentMap->increaseChangeIndex();

        /*!
         * Inertial welding changes the same derived-map coordinate contract
         * as a visual merge. Increment the externally observed revision after
         * all corrected poses and semantic entities have become authoritative.
         */
        pCurrentMap->informNewBigChange();

        p_atlas->changeMap(pCurrentMap);
        p_atlas->setMapBad(pMergeMap);
        p_atlas->removeBadMaps();
    };
    for (KeyFrame *pKFi : mergeConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->updateConnections();
    }

    /* A sufficiently established current map can support inertial welding BA.
     */
    bool inertialBundleAdjustmentRan = false;

    if (numKFnew >= 10)
    {
        bool      bStopFlag         = false;
        KeyFrame *p_currentKeyFrame = p_tracker->getLastKeyFrame();

        if (p_currentKeyFrame != nullptr)
        {
            Optimizer::mergeInertialBA(p_currentKeyFrame,
                                       p_mergeMatchedKF,
                                       &bStopFlag,
                                       pCurrentMap,
                                       CorrectedSim3);
            inertialBundleAdjustmentRan = true;
        }
    }

    if (inertialBundleAdjustmentRan)
    {
        /* Complete the post-BA pose map, including fixed deformation nodes. */
        for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
             NonCorrectedSim3)
        {
            (void)poseBefore_WorldToCamera;

            if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
                p_keyFrame->getMap() != pCurrentMap)
            {
                continue;
            }

            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->getPose().cast<double>();

            CorrectedSim3.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                          poseAfter_WorldToCamera.translation(),
                          1.0));
        }

        const g2o::Sim3 identityTransform_WorldToWorld(
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            1.0);

        utils::utils::Utils::propagateSemanticPoseCorrections(pCurrentMap,
                                                NonCorrectedSim3,
                                                CorrectedSim3,
                                                identityTransform_WorldToWorld);
    }

    /* Fuse semantic hypotheses only after the final inertial pose correction.
     */
    if (types::SystemParams::getParams()->semSeg.reassociate.enabled)
    {
        utils::utils::Utils::reAssociateSemanticPlanes(p_atlas);
    }

    /* Matching stable room identities must collapse even when optional
     * geometry reassociation is disabled. */
    utils::utils::Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap, importedRooms);

    if (types::SystemParams::getParams()->semSeg.reassociate.enabled)
    {
        utils::utils::Utils::reAssociateRooms(p_atlas);
        utils::utils::Utils::reAssociatePassages(p_atlas);
    }

    finalizeInertialMerge();

    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    /* The semantic graph and Atlas ownership are now stable for other workers.
     */
    semanticUpdateLock.unlock();

    p_localMapper->release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

void LoopClosing::checkObservations(set<KeyFrame *> &spKFsMap1,
                                    set<KeyFrame *> &spKFsMap2)
{
    cout << "----------------------" << endl;
    for (KeyFrame *pKFi1 : spKFsMap1)
    {
        map<KeyFrame *, int> matchedMapPointCounts;
        set<MapPoint *>      spMPs = pKFi1->getMapPoints();

        for (MapPoint *pMPij : spMPs)
        {
            if (!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFrame *, tuple<int, int>> mapPointObservations =
                pMPij->getObservations();
            for (KeyFrame *pKFi2 : spKFsMap2)
            {
                if (mapPointObservations.find(pKFi2) !=
                    mapPointObservations.end())
                {
                    if (matchedMapPointCounts.find(pKFi2) !=
                        matchedMapPointCounts.end())
                    {
                        matchedMapPointCounts[pKFi2] =
                            matchedMapPointCounts[pKFi2] + 1;
                    }
                    else
                    {
                        matchedMapPointCounts[pKFi2] = 1;
                    }
                }
            }
        }

        if (matchedMapPointCounts.size() == 0)
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId
                 << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with "
                 << matchedMapPointCounts.size() << " KF from the other map"
                 << endl;
            for (pair<KeyFrame *, int> matchedKF : matchedMapPointCounts)
            {
                cout << "   -KF: " << matchedKF.first->mnId
                     << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    cout << "----------------------" << endl;
}

bool LoopClosing::isMergeInProgress(void)
{
    return mergeInProgress.load();
}

void LoopClosing::searchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                                vector<MapPoint *>    &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    // cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" <<
    // endl; cout << "FUSE: Intially there are " << CorrectedPosesMap.size() <<
    // " KFs" << endl;
    for (KeyFrameAndPose::const_iterator mit  = CorrectedPosesMap.begin(),
                                         mend = CorrectedPosesMap.end();
         mit != mend;
         mit++)
    {
        int       num_replaces = 0;
        KeyFrame *pKFi         = mit->first;
        Map      *pMap         = pKFi->getMap();

        g2o::Sim3     g2oScw = mit->second;
        Sophus::Sim3f Scw    = utils::converter::Converter::toSophus(g2oScw);

        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(nullptr));
        int numFused = matcher.fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int          nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep)
            {

                num_replaces += 1;
                pRep->replace(vpMapPoints[i]);
            }
        }

        total_replaces += num_replaces;
    }
    // cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
}

void LoopClosing::searchAndFuse(const vector<KeyFrame *> &vConectedKFs,
                                vector<MapPoint *>       &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    // cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs"
    // << endl; cout << "FUSE-POSE: Intially there are " << vConectedKFs.size()
    // << " KFs" << endl;
    for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end();
         mit != mend;
         mit++)
    {
        int           num_replaces = 0;
        KeyFrame     *pKF          = (*mit);
        Map          *pMap         = pKF->getMap();
        Sophus::SE3f  Tcw          = pKF->getPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(nullptr));
        matcher.fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int          nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep)
            {
                num_replaces += 1;
                pRep->replace(vpMapPoints[i]);
            }
        }
        /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << "
        MPs fused" << endl; total_replaces += num_replaces;*/
    }
    // cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
}

void LoopClosing::requestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        resetRequested = true;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!resetRequested)
                break;
        }
        usleep(5000);
    }
}

void LoopClosing::requestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        resetActiveMapRequested = true;
        p_mapToReset            = pMap;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!resetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::resetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if (resetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        lastLoopKeyFrameId =
            0; // TODO old variable, it is not use in the new algorithm
        resetRequested          = false;
        resetActiveMapRequested = false;
    }
    else if (resetActiveMapRequested)
    {

        for (list<KeyFrame *>::const_iterator it = mlpLoopKeyFrameQueue.begin();
             it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFrame *pKFi = *it;
            if (pKFi->getMap() == p_mapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        lastLoopKeyFrameId =
            p_atlas->getLastInitKeyFrameId(); // TODO old variable, it is not
                                              // use in the new algorithm
        resetActiveMapRequested = false;
    }
}

void LoopClosing::runGlobalBundleAdjustment(Map          *pActiveMap,
                                            unsigned long nLoopKF,
                                            unsigned int  generation_in)
{
    Verbose::printMess("Starting Global Bundle Adjustment",
                       Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA =
        std::chrono::steady_clock::now();

    nFGBA_exec += 1;

    vnGBAKFs.push_back(pActiveMap->getAllKeyFrames().size());
    vnGBAMPs.push_back(pActiveMap->getAllMapPoints().size());
#endif

    /*
     * g2o accepts a plain bool force-stop token. A first-party iteration action
     * copies the atomic cross-thread request into this worker-local flag, so
     * cancellation remains race-free and takes effect between iterations.
     */
    bool optimizerStopRequested = false;

    const bool bImuInit = pActiveMap->isImuInitialized();

    if (!bImuInit)
        Optimizer::globalBundleAdjustment(pActiveMap,
                                          10,
                                          &optimizerStopRequested,
                                          nLoopKF,
                                          false,
                                          p_tracker->getMarkerImpact(),
                                          &globalBundleAdjustmentStopRequested);
    else
        Optimizer::fullInertialBA(pActiveMap,
                                  7,
                                  false,
                                  nLoopKF,
                                  &optimizerStopRequested,
                                  false,
                                  1e2F,
                                  1e6F,
                                  nullptr,
                                  nullptr,
                                  &globalBundleAdjustmentStopRequested);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndGBA =
        std::chrono::steady_clock::now();

    double timeGBA =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndGBA - time_StartFGBA)
            .count();
    vdGBA_ms.push_back(timeGBA);

    if (optimizerStopRequested)
    {
        nFGBA_abort += 1;
    }
#endif

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new
    // keyframes not included in the Global BA and they are not consistent with
    // the updated map. We need to propagate the correction through the spanning
    // tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if (generation_in != fullBundleAdjustmentIndex)
        {
            finishedGBA = true;
            runningGBA  = false;
            return;
        }

        if (!bImuInit && pActiveMap->isImuInitialized())
        {
            finishedGBA = true;
            runningGBA  = false;
            return;
        }

        if (!optimizerStopRequested)
        {
            Verbose::printMess("Global Bundle Adjustment finished",
                               Verbose::VERBOSITY_NORMAL);
            Verbose::printMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            p_localMapper->requestStop();
            // Wait until Local Mapping has effectively stopped

            while (!p_localMapper->isStopped() && !p_localMapper->isFinished())
            {
                usleep(1000);
            }

            std::unique_lock<std::mutex> semanticUpdateLock =
                p_atlas->acquireSemanticUpdateLock();

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

            KeyFrameAndPose keyFramePosesBefore_WorldToCamera;
            KeyFrameAndPose keyFramePosesAfter_WorldToCamera;

            //  Correct keyframes starting at map first keyframe
            list<KeyFrame *> lpKFtoCheck(pActiveMap->keyFrameOrigins.begin(),
                                         pActiveMap->keyFrameOrigins.end());

            while (!lpKFtoCheck.empty())
            {
                KeyFrame             *pKF     = lpKFtoCheck.front();
                const set<KeyFrame *> sChilds = pKF->getChilds();
                Sophus::SE3f          Twc     = pKF->getPoseInverse();
                for (set<KeyFrame *>::const_iterator sit = sChilds.begin();
                     sit != sChilds.end();
                     sit++)
                {
                    KeyFrame *pChild = *sit;
                    if (!pChild || pChild->isBad())
                        continue;

                    if (pChild->baGlobalKeyFrameId != nLoopKF)
                    {
                        Sophus::SE3f Tchildc = pChild->getPose() * Twc;
                        pChild->tcwGBA =
                            Tchildc * pKF->tcwGBA; //*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = pChild->tcwGBA.so3().inverse() *
                                            pChild->getPose().so3();
                        if (pChild->isVelocitySet())
                        {
                            pChild->vwbGBA = Rcor * pChild->getVelocity();
                        }
                        else
                            Verbose::printMess("Child velocity empty!! ",
                                               Verbose::VERBOSITY_NORMAL);

                        pChild->biasGBA = pChild->getImuBias();

                        pChild->baGlobalKeyFrameId = nLoopKF;
                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->tcwBefGBA = pKF->getPose();

                const Sophus::SE3d poseBefore_WorldToCamera =
                    pKF->tcwBefGBA.cast<double>();

                keyFramePosesBefore_WorldToCamera.insert_or_assign(
                    pKF,
                    g2o::Sim3(poseBefore_WorldToCamera.unit_quaternion(),
                              poseBefore_WorldToCamera.translation(),
                              1.0));

                pKF->setPose(pKF->tcwGBA);

                const Sophus::SE3d poseAfter_WorldToCamera =
                    pKF->getPose().cast<double>();

                keyFramePosesAfter_WorldToCamera.insert_or_assign(
                    pKF,
                    g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                              poseAfter_WorldToCamera.translation(),
                              1.0));

                if (pKF->isImu)
                {
                    pKF->vwbBefGBA = pKF->getVelocity();

                    pKF->setVelocity(pKF->vwbGBA);
                    pKF->setNewBias(pKF->biasGBA);
                }

                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint *> vpMPs = pActiveMap->getAllMapPoints();

            for (size_t i = 0; i < vpMPs.size(); i++)
            {
                MapPoint *pMP = vpMPs[i];

                if (pMP == nullptr || pMP->isBad())
                    continue;

                bool mapPointWasCorrected = false;

                if (pMP->baGlobalKeyFrameId == nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->setWorldPos(pMP->posGBA);
                    mapPointWasCorrected = true;
                }
                else
                {
                    // Update according to the correction of its reference
                    // keyframe
                    KeyFrame *pRefKF = pMP->getReferenceKeyFrame();

                    if (pRefKF == nullptr || pRefKF->isBad() ||
                        pRefKF->getMap() != pActiveMap ||
                        pRefKF->baGlobalKeyFrameId != nLoopKF)
                    {
                        pRefKF = nullptr;

                        const auto observations = pMP->getObservations();

                        for (const auto &[p_observingKeyFrame, featureIndexes] :
                             observations)
                        {
                            (void)featureIndexes;

                            if (p_observingKeyFrame == nullptr ||
                                p_observingKeyFrame->isBad() ||
                                p_observingKeyFrame->getMap() != pActiveMap ||
                                p_observingKeyFrame->baGlobalKeyFrameId !=
                                    nLoopKF)
                            {
                                continue;
                            }

                            if (pRefKF == nullptr ||
                                p_observingKeyFrame->mnId < pRefKF->mnId)
                            {
                                pRefKF = p_observingKeyFrame;
                            }
                        }
                    }

                    if (pRefKF == nullptr)
                    {
                        continue;
                    }

                    /*if(pRefKF->mTcwBefGBA.empty())
                        continue;*/

                    // Map to non-corrected camera
                    // cv::Mat Rcw =
                    // pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3); cv::Mat
                    // tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    Eigen::Vector3f Xc = pRefKF->tcwBefGBA * pMP->getWorldPos();

                    // Backproject using corrected camera
                    pMP->setWorldPos(pRefKF->getPoseInverse() * Xc);
                    mapPointWasCorrected = true;
                }

                if (mapPointWasCorrected)
                {
                    pMP->updateNormalAndDepth();
                }
            }

            const g2o::Sim3 identityTransform_WorldToWorld(
                Eigen::Quaterniond::Identity(),
                Eigen::Vector3d::Zero(),
                1.0);

            /* Keep every semantic entity aligned with the corrected cameras. */
            utils::utils::Utils::propagateSemanticPoseCorrections(
                pActiveMap,
                keyFramePosesBefore_WorldToCamera,
                keyFramePosesAfter_WorldToCamera,
                identityTransform_WorldToWorld);

            /* Preserve plane variables which were optimized directly by GBA. */
            for (geometric::Plane *p_plane : pActiveMap->getAllPlanes())
            {
                if (p_plane == nullptr || p_plane->isBad() ||
                    p_plane->baGlobalKeyFrameId != nLoopKF)
                {
                    continue;
                }

                p_plane->alignGeometryToEquation(p_plane->planeGBA);
            }

            pActiveMap->informNewBigChange();
            pActiveMap->increaseChangeIndex();

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f,
            // mpTracker->GetLastKeyFrame()->getImuBias(),
            // mpTracker->GetLastKeyFrame());

            p_localMapper->release();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndUpdateMap =
                std::chrono::steady_clock::now();

            double timeUpdateMap =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndUpdateMap - time_EndGBA)
                    .count();
            vdUpdateMap_ms.push_back(timeUpdateMap);

            double timeFGBA = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndUpdateMap - time_StartFGBA)
                                  .count();
            vdFGBATotal_ms.push_back(timeFGBA);
#endif
            Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        finishedGBA = true;
        runningGBA  = false;
    }
}

void LoopClosing::requestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    // cout << "LC: Finish requested" << endl;
    finishRequested = true;
}

bool LoopClosing::checkFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finishRequested;
}

void LoopClosing::setFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    finished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finished;
}

} // namespace core
} // namespace vs_graphs
