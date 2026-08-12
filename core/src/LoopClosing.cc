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

#include "LoopClosing.h"

#include "Converter.h"
#include "G2oTypes.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Sim3Solver.h"

#include <chrono>
#include <limits>
#include <mutex>
#include <thread>

namespace ORB_SLAM3
{
namespace
{

bool verifyLoopMergeFloors(
    Map             *p_survivingMap_in,
    Map             *p_absorbedMap_in,
    const g2o::Sim3 &transform_absorbedWorldToSurvivingWorld_in,
    std::string     &result_out)
{
    Floor *p_survivingFloor =
        Floor::selectBestObservedFloor(p_survivingMap_in->GetAllFloors());
    Floor *p_absorbedFloor =
        Floor::selectBestObservedFloor(p_absorbedMap_in->GetAllFloors());

    const std::optional<Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? p_survivingFloor->getPlaneIdentity()
                                    : std::nullopt;
    const std::optional<Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? p_absorbedFloor->getPlaneIdentity()
                                   : std::nullopt;

    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        result_out = "DEFERRED";
        std::cout << "[FloorVerify] Map#" << p_survivingMap_in->GetId()
                  << " and Map#" << p_absorbedMap_in->GetId()
                  << " floor verification deferred (current="
                  << (survivingIdentity.has_value() ? "valid" : "missing")
                  << ", merge="
                  << (absorbedIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0" << std::endl;
        return false;
    }

    const std::optional<Floor::PlaneIdentity> transformedAbsorbedIdentity =
        Floor::transformPlaneIdentity(
            *absorbedIdentity,
            transform_absorbedWorldToSurvivingWorld_in);
    double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
    double floorOffset_m        = std::numeric_limits<double>::infinity();

    const bool floorsMatch =
        transformedAbsorbedIdentity.has_value() &&
        Floor::planeIdentitiesMatch(*survivingIdentity,
                                    *transformedAbsorbedIdentity,
                                    Floor::kMergeMaxPlaneNormalAngle_deg,
                                    Floor::kMergeMaxPlaneOffset_m,
                                    floorNormalAngle_deg,
                                    floorOffset_m);

    if (!floorsMatch)
    {
        result_out = "REJECTED";
        std::cerr << "[FloorVerify] Rejecting loop merge: Map#"
                  << p_survivingMap_in->GetId() << " and Map#"
                  << p_absorbedMap_in->GetId()
                  << " floor planes mismatch (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m
                  << " m; limits=" << Floor::kMergeMaxPlaneNormalAngle_deg
                  << " deg/" << Floor::kMergeMaxPlaneOffset_m
                  << " m). result=REJECTED committed=0" << std::endl;
        return false;
    }

    std::cout << "[FloorVerify] Map#" << p_survivingMap_in->GetId()
              << " and Map#" << p_absorbedMap_in->GetId()
              << " floor planes match (angle=" << floorNormalAngle_deg
              << " deg, offset=" << floorOffset_m
              << " m). result=ACCEPTED committed=0" << std::endl;
    result_out = "ACCEPTED";
    return true;
}

void collapseMergedFloors(Map *p_survivingMap_in)
{
    if (p_survivingMap_in == nullptr)
    {
        return;
    }

    const std::vector<Floor *> allFloors = p_survivingMap_in->GetAllFloors();
    if (allFloors.size() <= 1U)
    {
        return;
    }

    Floor *p_keeperFloor = Floor::selectBestObservedFloor(allFloors);
    if (p_keeperFloor == nullptr)
    {
        return;
    }

    for (Floor *p_duplicateFloor : allFloors)
    {
        if (p_duplicateFloor == nullptr || p_duplicateFloor == p_keeperFloor)
        {
            continue;
        }

        for (Room *p_room : p_duplicateFloor->getRooms())
        {
            if (p_room != nullptr && !p_room->isBad())
            {
                p_keeperFloor->addRoom(p_room);
            }
        }

        p_survivingMap_in->EraseMapFloor(p_duplicateFloor);
        std::cout << "[LoopClosing] Fused duplicate Floor#"
                  << p_duplicateFloor->getId() << " into Floor#"
                  << p_keeperFloor->getId()
                  << " and retained the better-observed plane identity."
                  << std::endl;
    }

    for (Room *p_room : p_survivingMap_in->GetAllDetectedMapRooms())
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
    mbResetRequested(false),
    mbResetActiveMapRequested(false),
    mbFinishRequested(false),
    mbFinished(true),
    mpAtlas(pAtlas),
    mpKeyFrameDB(pDB),
    mpORBVocabulary(pVoc),
    mpMatchedKF(NULL),
    mbLoopDetected(false),
    mnLoopNumCoincidences(0),
    mnLoopNumNotFound(0),
    mbMergeDetected(false),
    mbMergeInProgress(false),
    mnMergeNumCoincidences(0),
    mnMergeNumNotFound(0),
    mLastLoopKFid(0),
    mbRunningGBA(false),
    mbFinishedGBA(true),
    mpThreadGBA(NULL),
    mbFixScale(bFixScale),
    mnFullBAIdx(0),
    mbActiveLC(bActiveLC)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF             = static_cast<KeyFrame *>(NULL);

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
    mnNumCorrection   = 0;
    mnCorrectionGBA   = 0;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void LoopClosing::SetMergeStatus(bool mergeStatus_in)
{
    mbMergeInProgress.store(mergeStatus_in);
}

LoopClosing::LoopCorrectionStatus LoopClosing::GetLoopCorrectionStatus() const
{
    std::lock_guard<std::mutex> lock(mMutexLoopCorrectionStatus);
    return mLoopCorrectionStatus;
}

void LoopClosing::recordLoopCorrectionEvent(bool               accepted_in,
                                            const std::string &reason_in)
{
    {
        std::lock_guard<std::mutex> lock(mMutexLoopCorrectionStatus);
        ++mLoopCorrectionStatus.sequence;
        mLoopCorrectionStatus.hasEvent     = true;
        mLoopCorrectionStatus.lastAccepted = accepted_in;
        mLoopCorrectionStatus.lastReason   = reason_in;

        if (accepted_in)
        {
            ++mLoopCorrectionStatus.acceptedCount;
        }
        else
        {
            ++mLoopCorrectionStatus.rejectedCount;
        }

        if (mpCurrentKF != nullptr)
        {
            mLoopCorrectionStatus.lastCurrentKeyFrameId = mpCurrentKF->mnId;
            mLoopCorrectionStatus.lastCurrentTimestamp =
                mpCurrentKF->mTimeStamp;
            if (mpCurrentKF->GetMap() != nullptr)
            {
                mLoopCorrectionStatus.lastMapId =
                    mpCurrentKF->GetMap()->GetId();
            }
        }
        if (mpLoopMatchedKF != nullptr)
        {
            mLoopCorrectionStatus.lastMatchedKeyFrameId = mpLoopMatchedKF->mnId;
            mLoopCorrectionStatus.lastMatchedTimestamp =
                mpLoopMatchedKF->mTimeStamp;
        }
    }
}

bool LoopClosing::stopGlobalBundleAdjustment()
{
    std::thread *p_globalBundleAdjustmentThread = nullptr;
    bool         optimizationWasRunning         = false;

    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);

        optimizationWasRunning = mbRunningGBA;

        if (optimizationWasRunning)
        {
            /* Invalidate the result before waiting for the worker to finish. */
            ++mnFullBAIdx;
            globalBundleAdjustmentStopRequested.store(
                true,
                std::memory_order_release);
        }

        /*
         * Move ownership out while holding the state mutex. The mutex must be
         * released before join() because the worker takes it while finishing.
         */
        p_globalBundleAdjustmentThread = mpThreadGBA;
        mpThreadGBA                    = nullptr;
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
        mbRunningGBA  = false;
        mbFinishedGBA = true;
    }

    return optimizationWasRunning;
}

void LoopClosing::relaunchGlobalBundleAdjustment(Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr || mpCurrentKF == nullptr)
    {
        return;
    }

    std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
    mbRunningGBA  = true;
    mbFinishedGBA = false;
    globalBundleAdjustmentStopRequested.store(false, std::memory_order_release);
    mpThreadGBA = new std::thread(&LoopClosing::RunGlobalBundleAdjustment,
                                  this,
                                  p_activeMap_in,
                                  mpCurrentKF->mnId,
                                  mnFullBAIdx);
}

void LoopClosing::Run(void)
{
    /*!
     * Mark the LoopClosing worker as active. SetFinish() changes this back to
     * true when Run() exits.
     *
     * This flag is observed by other threads through isFinished().
     */
    mbFinished = false;

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
        if (CheckNewKeyFrames())
        {
            /*!
             * Check that the last keyframe to be added is valid.
             *
             * If so, then clear the buffer of candidate keyframes and buffer
             * of merged keyframes. he previous processed KF may still hold
             * debug/candidate lists from its last place-recognition query.
             */
            if (mpLastCurrentKF)
            {
                /*!
                 * Remove stale same-map loop candidates associated with the
                 * previous current KF.
                 */
                mpLastCurrentKF->mvpLoopCandKFs.clear();

                /*!
                 * Remove stale cross-map merge candidates associated with the
                 * previous current KF.
                 */
                mpLastCurrentKF->mvpMergeCandKFs.clear();
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
            bool bFindedRegion = NewDetectCommonRegions();

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
                if (mbMergeDetected)
                {
                    /* If required, confirm IMU is working */
                    if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
                         mpTracker->mSensor == System::IMU_STEREO ||
                         mpTracker->mSensor == System::IMU_RGBD) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
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
                            mpMergeMatchedKF->GetPose().cast<double>();

                        /* Convert above keyframe pose into Sim3 datatype */
                        g2o::Sim3 gSmw2(mTmw.unit_quaternion(),
                                        mTmw.translation(),
                                        1.0);

                        /*!
                         * Get pose of the current keyframe in the current
                         * keyframes world frame.
                         */
                        Sophus::SE3d mTcw =
                            mpCurrentKF->GetPose().cast<double>();

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
                        mSold_new = (gSw2c * gScw1);

                        /*!
                         * If in both frames an IMU is used, use IMU readings
                         * for inertial odometry map constraints.
                         */
                        if (mpCurrentKF->GetMap()->IsInertial() &&
                            mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            cout << "Merge check transformation with IMU"
                                 << endl;

                            /* Reject maps with bad scale */
                            if (mSold_new.scale() < 0.90 ||
                                mSold_new.scale() > 1.1)
                            {
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected    = false;
                                Verbose::PrintMess(
                                    "scale bad estimated. Abort merging",
                                    Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
                                 mpTracker->mSensor == System::IMU_STEREO ||
                                 mpTracker->mSensor == System::IMU_RGBD) &&
                                mpCurrentKF->GetMap()->GetIniertialBA1())
                            {
                                Eigen::Vector3d phi = LogSO3(
                                    mSold_new.rotation().toRotationMatrix());
                                phi(0)    = 0;
                                phi(1)    = 0;
                                mSold_new = g2o::Sim3(ExpSO3(phi),
                                                      mSold_new.translation(),
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
                        nMerges += 1;
#endif

                        /* Set flag to indicate that mergins is happening */
                        SetMergeStatus(true);

                        /* Choose merging method based on if IMU is used */
                        if (mpTracker->mSensor == System::IMU_MONOCULAR ||
                            mpTracker->mSensor == System::IMU_STEREO ||
                            mpTracker->mSensor == System::IMU_RGBD)
                        {
                            /* Merge maps using IMU */
                            MergeLocalInertial();
                        }
                        else
                        {
                            /* Merge maps */
                            MergeLocal();
                        }

                        /* Set flag to indicate that mergins has finished */
                        SetMergeStatus(false);

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMerge =
                            std::chrono::steady_clock::now();

                        double timeMergeTotal =
                            std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndMerge - time_StartMerge)
                                .count();
                        vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

                        std::cout
                            << "[LoopClosing] Map merge has been finished."
                            << std::endl;
                    }

                    /*!
                     * Log the recognition event. This way we can correspond the
                     * times that both keyframes where matched. This effectively
                     * acts as 2 independent lists where there index indicates
                     * the matching KF pair.
                     */
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    /* Reset all variables */
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected    = false;

                    /*!
                     * Reset any pending loop closure candidate after a
                     * successful map merge.
                     *
                     * A place recognition query can simultaneously produce:
                     *
                     *      - a merge candidate (`mbMergeDetected`) when the
                     *        matched keyframe belongs to another map.
                     *
                     *      - a loop closure candidate (`mbLoopDetected`) when
                     *        the matched keyframe belongs to the current map.
                     *
                     * After merging maps, any loop closure candidate becomes
                     * invalid because the map topology, keyframe ownership, and
                     * map point associations may have changed. Therefore the
                     * loop closure state is discarded.
                     */
                    if (mbLoopDetected)
                    {
                        recordLoopCorrectionEvent(false,
                                                  "superseded_by_map_merge");
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected    = false;
                    }
                }

                /*!
                 * A loop closure candidate has been successfully detected and
                 * geometrically validated. The matched keyframe belongs to the
                 * same map as the current keyframe, meaning the estimated Sim3
                 * transformation can be used to correct accumulated drift in
                 * the map.
                 */
                if (mbLoopDetected)
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
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
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
                    if (mpCurrentKF->GetMap()->IsInertial())
                    {
                        Sophus::SE3d Twc =
                            mpCurrentKF->GetPoseInverse().cast<double>();

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
                            if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
                                 mpTracker->mSensor == System::IMU_STEREO ||
                                 mpTracker->mSensor == System::IMU_RGBD) &&
                                mpCurrentKF->GetMap()->GetIniertialBA2())
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
                        mvpLoopMapPoints = mvpLoopMPs;

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
                        CorrectLoop();
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
                        mnNumCorrection += 1;
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
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected    = false;
                }
            }
            mpLastCurrentKF = mpCurrentKF;
        }

        ResetIfRequested();

        if (CheckFinish())
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
    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if (pKF->mnId != 0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be
    // performed
    if (!mbActiveLC)
    {
        return false;
    }

    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this
        // thread
        mpCurrentKF->SetNotErase();
        mpCurrentKF->mbCurrentPlaceRecognition = true;

        mpLastMap = mpCurrentKF->GetMap();
    }

    if (mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if (mpTracker->mSensor == System::STEREO &&
        mpLastMap->GetAllKeyFrames().size() < 5) // 12
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if (mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
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
    if (mnLoopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse())
                .cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw           = gScl * mg2oLoopSlw;
        int       numProjMatches = 0;
        vector<MapPoint *> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF,
                                                            mpLoopMatchedKF,
                                                            gScw,
                                                            numProjMatches,
                                                            mvpLoopMPs,
                                                            vpMatchedMPs);
        if (bCommonRegion)
        {

            bLoopDetectedInKF = true;

            mnLoopNumCoincidences++;
            mpLoopLastCurrentKF->SetErase();
            mpLoopLastCurrentKF = mpCurrentKF;
            mg2oLoopSlw         = gScw;
            mvpLoopMatchedMPs   = vpMatchedMPs;

            mbLoopDetected    = mnLoopNumCoincidences >= 3;
            mnLoopNumNotFound = 0;
        }
        else
        {
            bLoopDetectedInKF = false;

            mnLoopNumNotFound++;
            if (mnLoopNumNotFound >= 2)
            {
                recordLoopCorrectionEvent(false, "geometric_validation");
                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMPs.clear();
                mnLoopNumNotFound = 0;
            }
        }
    }

    // Merge candidates
    bool bMergeDetectedInKF = false;
    if (mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse())
                .cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw           = gScl * mg2oMergeSlw;
        int       numProjMatches = 0;
        vector<MapPoint *> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF,
                                                            mpMergeMatchedKF,
                                                            gScw,
                                                            numProjMatches,
                                                            mvpMergeMPs,
                                                            vpMatchedMPs);
        if (bCommonRegion)
        {
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++;
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw         = gScw;
            mvpMergeMatchedMPs   = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3;
        }
        else
        {
            mbMergeDetected    = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if (mnMergeNumNotFound >= 2)
            {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
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

    if (mbMergeDetected || mbLoopDetected)
    {
#ifdef REGISTER_TIMES
        vdEstSim3_ms.push_back(timeEstSim3);
#endif
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    // TODO: This is only necessary if we use a minimun score for pick the best
    // candidates
    const vector<KeyFrame *> vpConnectedKeyFrames =
        mpCurrentKF->GetVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
    if (!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartQuery =
            std::chrono::steady_clock::now();
#endif
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF,
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
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand,
                                                    mpLoopMatchedKF,
                                                    mpLoopLastCurrentKF,
                                                    mg2oLoopSlw,
                                                    mnLoopNumCoincidences,
                                                    mvpLoopMPs,
                                                    mvpLoopMatchedMPs);
    }
    // Merge candidates
    if (!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand,
                                                     mpMergeMatchedKF,
                                                     mpMergeLastCurrentKF,
                                                     mg2oMergeSlw,
                                                     mnMergeNumCoincidences,
                                                     mvpMergeMPs,
                                                     mvpMergeMatchedMPs);
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

    mpKeyFrameDB->add(mpCurrentKF);

    if (mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(
    KeyFrame                *pCurrentKF,
    KeyFrame                *pMatchedKF,
    g2o::Sim3               &gScw,
    int                     &nNumProjMatches,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    set<MapPoint *> spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF,
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
        Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
        g2o::Sim3    gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3    gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale =
            mbFixScale; // TODO CHECK; Solo para el monocular inertial
        if (mpTracker->mSensor == System::IMU_MONOCULAR &&
            !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale = false;
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF,
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
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(),
                               static_cast<MapPoint *>(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF,
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

bool LoopClosing::DetectCommonRegionsFromBoW(
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

    set<KeyFrame *> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

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
            pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
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
            std::vector<MapPoint *>(mpCurrentKF->GetMapPointMatches().size(),
                                    static_cast<MapPoint *>(NULL));
        std::vector<KeyFrame *> vpKeyFrameMatchedMP =
            std::vector<KeyFrame *>(mpCurrentKF->GetMapPointMatches().size(),
                                    static_cast<KeyFrame *>(NULL));

        int nIndexMostBoWMatchesKF = 0;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.SearchByBoW(mpCurrentKF,
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
            bool bFixedScale = mbFixScale;
            if (mpTracker->mSensor == System::IMU_MONOCULAR &&
                !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale = false;

            Sim3Solver solver = Sim3Solver(mpCurrentKF,
                                           pMostBoWMatchesKF,
                                           vpMatchedPoints,
                                           bFixedScale,
                                           vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99,
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
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(
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
                    for (MapPoint *pCovMPij : pCovKFi->GetMapPointMatches())
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

                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),
                               solver.GetEstimatedTranslation().cast<double>(),
                               (double)solver.GetEstimatedScale());
                g2o::Sim3 gSmw(
                    pMostBoWMatchesKF->GetRotation().cast<double>(),
                    pMostBoWMatchesKF->GetTranslation().cast<double>(),
                    1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current
                                              // from the world position
                Sophus::Sim3f mScw = Converter::toSophus(gScw);

                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(),
                                   static_cast<MapPoint *>(NULL));
                vector<KeyFrame *> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(),
                                   static_cast<KeyFrame *>(NULL));
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF,
                                                                mScw,
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

                    bool bFixedScale = mbFixScale;
                    if (mpTracker->mSensor == System::IMU_MONOCULAR &&
                        !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale = false;

                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF,
                                                                pKFi,
                                                                vpMatchedMP,
                                                                gScm,
                                                                10,
                                                                mbFixScale,
                                                                mHessian7x7,
                                                                true);

                    if (numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(
                            pMostBoWMatchesKF->GetRotation().cast<double>(),
                            pMostBoWMatchesKF->GetTranslation().cast<double>(),
                            1.0);
                        g2o::Sim3 gScw =
                            gScm * gSmw; // Similarity matrix of current from
                                         // the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPoint *> vpMatchedMP;
                        vpMatchedMP.resize(
                            mpCurrentKF->GetMapPointMatches().size(),
                            static_cast<MapPoint *>(NULL));
                        int numProjOptMatches =
                            matcher.SearchByProjection(mpCurrentKF,
                                                       mScw,
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
                                    pMPi->GetIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if (index >= 0)
                                {
                                    int coord_x = pKFi->mvKeysUn[index].pt.x;
                                    if (coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if (coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y = pKFi->mvKeysUn[index].pt.y;
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
                                mpCurrentKF->GetBestCovisibilityKeyFrames(
                                    nNumCovisibles);

                            int j = 0;
                            while (nNumKFs < 3 && j < vpCurrentCovKFs.size())
                            {
                                KeyFrame    *pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc =
                                    (pKFj->GetPose() *
                                     mpCurrentKF->GetPoseInverse())
                                        .cast<double>();
                                g2o::Sim3          gSjc(mTjc.unit_quaternion(),
                                               mTjc.translation(),
                                               1.0);
                                g2o::Sim3          gSjw = gSjc * gScw;
                                int                numProjMatches_j = 0;
                                vector<MapPoint *> vpMatchedMPs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(
                                    pKFj,
                                    pMostBoWMatchesKF,
                                    gSjw,
                                    numProjMatches_j,
                                    vpMapPoints,
                                    vpMatchedMPs_j);

                                if (bValid)
                                {
                                    Sophus::SE3f Tc_w  = mpCurrentKF->GetPose();
                                    Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
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
        pLastCurrentKF   = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2      = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
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

bool LoopClosing::DetectCommonRegionsFromLastKF(
    KeyFrame                *pCurrentKF,
    KeyFrame                *pMatchedKF,
    g2o::Sim3               &gScw,
    int                     &nNumProjMatches,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    set<MapPoint *> spAlreadyMatchedMPs(vpMatchedMPs.begin(),
                                        vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF,
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

int LoopClosing::FindMatchesByProjection(KeyFrame        *pCurrentKF,
                                         KeyFrame        *pMatchedKFw,
                                         g2o::Sim3       &g2oScw,
                                         set<MapPoint *> &spMatchedMPinOrigin,
                                         vector<MapPoint *> &vpMapPoints,
                                         vector<MapPoint *> &vpMatchedMapPoints)
{
    int                nNumCovisibles = 10;
    vector<KeyFrame *> vpCovKFm =
        pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame *> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame *> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if (nInitialCov < nNumCovisibles)
    {
        for (int i = 0; i < nInitialCov; ++i)
        {
            vector<KeyFrame *> vpKFs =
                vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
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
        for (MapPoint *pMPij : pKFi->GetMapPointMatches())
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

    Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
    ORBmatcher    matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(),
                              static_cast<MapPoint *>(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF,
                                                 mScw,
                                                 vpMapPoints,
                                                 vpMatchedMapPoints,
                                                 3,
                                                 1.5);

    return num_matches;
}

void LoopClosing::CorrectLoop()
{
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue();

    /* Stop and reclaim any global bundle-adjustment worker before mutation. */
    stopGlobalBundleAdjustment();

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped())
        usleep(1000);

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected
    // Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
    Sophus::SE3f Twc           = mpCurrentKF->GetPoseInverse();
    Sophus::SE3f Tcw           = mpCurrentKF->GetPose();
    g2o::Sim3    g2oScw(Tcw.unit_quaternion().cast<double>(),
                     Tcw.translation().cast<double>(),
                     1.0);
    NonCorrectedSim3[mpCurrentKF] = g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3
    // (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),
                              mg2oLoopScw.translation() / mg2oLoopScw.scale());
    mpCurrentKF->SetPose(correctedTcw.cast<float>());

    Map *pLoopMap = mpCurrentKF->GetMap();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFusion =
        std::chrono::steady_clock::now();
#endif

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for (vector<KeyFrame *>::iterator vit  = mvpCurrentConnectedKFs.begin(),
                                          vend = mvpCurrentConnectedKFs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *pKFi = *vit;

            if (pKFi != mpCurrentKF)
            {
                Sophus::SE3f Tiw = pKFi->GetPose();
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
                pKFi->SetPose(correctedTiw.cast<float>());

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
            pKFi->SetPose(correctedTiw.cast<float>());*/

            vector<MapPoint *> vpMPsi = pKFi->GetMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                MapPoint *pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;
                if (pMPi->isBad())
                    continue;
                if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with
                // corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw =
                    g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                pMPi->mnCorrectedByKF      = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Correct velocity according to orientation correction
            if (bImuInit)
            {
                Eigen::Quaternionf Rcor =
                    (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation())
                        .cast<float>();
                pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        // TODO Check this index increasement
        mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++)
        {
            if (mvpLoopMatchedMPs[i])
            {
                MapPoint *pLoopMP = mvpLoopMatchedMPs[i];
                MapPoint *pCurMP  = mpCurrentKF->GetMapPoint(i);
                if (pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP, i);
                    pLoopMP->AddObservation(mpCurrentKF, i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
        // cout << "LC: end replacing duplicated" << endl;
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will
    // appear attaching both sides of the loop
    map<KeyFrame *, set<KeyFrame *>> LoopConnections;

    for (vector<KeyFrame *>::iterator vit  = mvpCurrentConnectedKFs.begin(),
                                      vend = mvpCurrentConnectedKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame          *pKFi = *vit;
        vector<KeyFrame *> vpPreviousNeighbors =
            pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
        for (vector<KeyFrame *>::iterator
                 vit_prev  = vpPreviousNeighbors.begin(),
                 vend_prev = vpPreviousNeighbors.end();
             vit_prev != vend_prev;
             vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for (vector<KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(),
                                          vend2 = mvpCurrentConnectedKFs.end();
             vit2 != vend2;
             vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = mbFixScale;
    // TODO CHECK; Solo para el monocular inertial
    if (mpTracker->mSensor == System::IMU_MONOCULAR &&
        !mpCurrentKF->GetMap()->GetIniertialBA2())
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
    if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap,
                                              mpLoopMatchedKF,
                                              mpCurrentKF,
                                              NonCorrectedSim3,
                                              CorrectedSim3,
                                              LoopConnections);
    }
    else
    {
        // cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
        Optimizer::OptimizeEssentialGraph(pLoopMap,
                                          mpLoopMatchedKF,
                                          mpCurrentKF,
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

    mpAtlas->InformNewBigChange();

    // Add loop edge
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few
    // keyframes, if not it would take too much time)
    if (!pLoopMap->isImuInitialized() ||
        (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))
    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
        mbRunningGBA    = true;
        mbFinishedGBA   = false;
        mnCorrectionGBA = mnNumCorrection;
        globalBundleAdjustmentStopRequested.store(false,
                                                  std::memory_order_release);

        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,
                                 this,
                                 pLoopMap,
                                 mpCurrentKF->mnId,
                                 mnFullBAIdx);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid =
        mpCurrentKF
            ->mnId; // TODO old varible, it is not use in the new algorithm
}

void LoopClosing::MergeLocal()
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
    sysParams = SystemParams::GetParams();

    /* Reject stale place-recognition candidates before stopping other workers.
     */
    if (mpCurrentKF == nullptr || mpMergeMatchedKF == nullptr ||
        mpCurrentKF->isBad() || mpMergeMatchedKF->isBad())
    {
        return;
    }

    Map *pCurrentMap = mpCurrentKF->GetMap();
    Map *pMergeMap   = mpMergeMatchedKF->GetMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->IsBad() ||
        pMergeMap->IsBad() || !mpAtlas->isActiveMap(pCurrentMap) ||
        !mpAtlas->isActiveMap(pMergeMap))
    {
        return;
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
    mpLocalMapper->RequestStop();

    /* Wait until local mapper stops */
    while (!mpLocalMapper->isStopped())
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
        mpAtlas->acquireSemanticUpdateLock();

    /* Revalidate after quiescing workers; retained retired maps keep stale
     * raw pointers alive, so pointer non-nullness alone is insufficient. */
    if (mpCurrentKF->isBad() || mpMergeMatchedKF->isBad() ||
        mpCurrentKF->GetMap() != pCurrentMap ||
        mpMergeMatchedKF->GetMap() != pMergeMap ||
        !mpAtlas->isActiveMap(pCurrentMap) || !mpAtlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        mpLocalMapper->Release();
        return;
    }

    const Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
    const g2o::Sim3    g2oNonCorrectedSwc(Twc.unit_quaternion(),
                                       Twc.translation(),
                                       1.0);
    const g2o::Sim3    g2oSwCurrentWMerge = g2oNonCorrectedSwc * mg2oMergeScw;
    const g2o::Sim3    g2oSwMergeWCurrent = g2oSwCurrentWMerge.inverse();

    std::string floorVerificationResult;
    if (!verifyLoopMergeFloors(pCurrentMap,
                               pMergeMap,
                               g2oSwCurrentWMerge,
                               floorVerificationResult))
    {
        semanticUpdateLock.unlock();
        mpLocalMapper->Release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return;
    }

    /* Discard queued keyframes only after every merge rejection gate passed. */
    mpLocalMapper->EmptyQueue();

    /* Update the connections of the current keyframe */
    mpCurrentKF->UpdateConnections();

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
    if (pCurrentMap->IsInertial() && pMergeMap->IsInertial())
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        KeyFrame *pKFi      = mpCurrentKF;
        int       nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->mPrevKF;
            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        pKFi      = mpCurrentKF->mNextKF;
        nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->mNextKF;
            nInserted++;
        }
    }
    else
    {
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> vpCovisibleKFs =
        mpCurrentKF->GetBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spLocalWindowKFs.insert(mpCurrentKF);

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
                pKFi->GetBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

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
        const std::set<MapPoint *> spMPs = pKFi->GetMapPoints();

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
    if (pCurrentMap->IsInertial() && pMergeMap->IsInertial())
    {
        KeyFrame *pKFi      = mpMergeMatchedKF;
        int       nInserted = 0;

        /* ------------------------------------------------------------------ *
         * Walk backwards
         * ------------------------------------------------------------------ */

        while (pKFi && nInserted < (kNumTemporalKFs / 2))
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->mPrevKF;

            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards
         * ------------------------------------------------------------------ */

        pKFi = mpMergeMatchedKF->mNextKF;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->mNextKF;

            nInserted++;
        }
    }
    else
    {
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    vpCovisibleKFs =
        mpMergeMatchedKF->GetBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spMergeConnectedKFs.insert(mpMergeMatchedKF);

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
                pKFi->GetBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

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
        const auto spMPs = pKFi->GetMapPoints();

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
        if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        const Sophus::SE3d TiwMerge = pKFi->GetPose().cast<double>();

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
        pKFi->mfScale   = s;
        pKFi->mTcwMerge = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                       g2oSiwCurrent.translation() / s)
                              .cast<float>();

        /* If there is IMU, extract velocity */
        if (pCurrentMap->isImuInitialized())
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
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
        if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
        {
            itMP = spMapPointMerge.erase(itMP);
            continue;
        }

        /* Extract position of point */
        const Eigen::Vector3d P3DwMerge = pMPi->GetWorldPos().cast<double>();

        /* Transform the point into the current map world frame */
        pMPi->mPosMerge = g2oSwCurrentWMerge.map(P3DwMerge).cast<float>();

        /* Transform the points surface normal into current map world frame */
        pMPi->mNormalVectorMerge =
            g2oSwCurrentWMerge.rotation().cast<float>() * pMPi->GetNormal();

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
            if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            /* Store the old pose of the keyframe */
            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

            /* Apply corrected world-to-camera pose in the current-map frame */
            pKFi->SetPose(pKFi->mTcwMerge);

            /* Change keyframe's internal owning-map pointer to current map */
            pKFi->UpdateMap(pCurrentMap);

            /* Record which current keyframe triggered this merge correction */
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;

            /* Insert the same keyframe pointer into surviving map container */
            pCurrentMap->AddKeyFrame(pKFi);

            /* Remove the keyframe pointer from the old merge-map container */
            pMergeMap->EraseKeyFrame(pKFi);

            /* If there is IMU, add velocity */
            if (pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
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
            if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
            {
                continue;
            }

            /* Apply position expressed in the surviving current-map frame */
            pMPi->SetWorldPos(pMPi->mPosMerge);

            /* Apply the normal rotated into the surviving current-map frame */
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);

            /* Change the map point's internal owner to the current map */
            pMPi->UpdateMap(pCurrentMap);

            /* Register the same map-point pointer in the surviving map */
            pCurrentMap->AddMapPoint(pMPi);

            /* Remove the map-point pointer from the obsolete merge map */
            pMergeMap->EraseMapPoint(pMPi);
        }

        /* Set the map to be the current map */
        mpAtlas->ChangeMap(pCurrentMap);

        /* Incrase index tracking the amount of times the maps been changed */
        pCurrentMap->IncreaseChangeIndex();
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
    if (pMergeMap->GetOriginKF())
    {
        /* Allow the former merge-map root to become a normal tree child */
        pMergeMap->GetOriginKF()->SetFirstConnection(false);
    }

    /* Init variables to track the new child and parent keyframes */
    KeyFrame *pNewChild  = nullptr;
    KeyFrame *pNewParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    pNewChild = mpMergeMatchedKF->GetParent();

    /* The matched merge keyframe becomes the first reversed parent */
    pNewParent = mpMergeMatchedKF;

    /* Attach the matched merge keyframe beneath the current keyframe */
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);

    /* Reverse each edge along the original merge-map parent chain */
    while (pNewChild)
    {
        /* Remove the old child edge before reversing its direction */
        pNewChild->EraseChild(pNewParent);

        /* Save the next original parent before changing this relation */
        KeyFrame *pOldParent = pNewChild->GetParent();

        /* Make the former parent a child of the previous keyframe */
        pNewChild->ChangeParent(pNewParent);

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
    mpMergeMatchedKF->UpdateConnections();

    /* Init list of connected keyframes in merge map */
    std::vector<KeyFrame *> vpMergeConnectedKFs;

    /* Retrieve keyframes covisible with the matched merge keyframe */
    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();

    /* Include the matched merge keyframe in the fusion set */
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);

    /* Fuse duplicate current-map points into corrected merge keyframes */
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    /* Refresh covisibility links for current-map local keyframes */
    for (KeyFrame *pKFi : spLocalWindowKFs)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        pKFi->UpdateConnections();
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
        pKFi->UpdateConnections();
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
    if (mpTracker->mSensor == System::IMU_MONOCULAR ||
        mpTracker->mSensor == System::IMU_STEREO ||
        mpTracker->mSensor == System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        Optimizer::MergeInertialBA(mpCurrentKF,
                                   mpMergeMatchedKF,
                                   &bStop,
                                   pCurrentMap,
                                   vCorrectedSim3);
    }
    else
    {
        /* Refine the merged region using visual observations only */
        Optimizer::LoopClosureLocalBundleAdjustment(mpMergeMatchedKF,
                                                    vpMergeConnectedKFs,
                                                    vpLocalCurrentWindowKFs,
                                                    &bStop);
    }

    /* Resume local mapping after merge optimisation is complete */
    mpLocalMapper->Release();

    /* ---------------------------------------------------------------------- *
     * SECTION 16 - RETRIEVE THE REMAINING MERGE MAP
     *
     * The local welding window has already been transferred.
     *
     * This section retrieves every remaining object that still belongs to the
     * obsolete merge map.
     * ---------------------------------------------------------------------- */

    /* Copy all planes currently owned by the merge map */
    std::vector<Plane *> vpCurrentMapPlanes = pMergeMap->GetAllPlanes();

    /* Copy all keyframes currently owned by the merge map */
    std::vector<KeyFrame *> vpCurrentMapKFs = pMergeMap->GetAllKeyFrames();

    const bool hasValidRemainingMergeKeyFrame =
        std::any_of(vpCurrentMapKFs.begin(),
                    vpCurrentMapKFs.end(),
                    [pMergeMap](KeyFrame *p_keyFrame_in)
                    {
                        return p_keyFrame_in != nullptr &&
                               !p_keyFrame_in->isBad() &&
                               p_keyFrame_in->GetMap() == pMergeMap;
                    });

    /* Copy all map points currently owned by the merge map */
    std::vector<MapPoint *> vpCurrentMapMPs = pMergeMap->GetAllMapPoints();

    /* Copy all markers currently owned by the merge map */
    std::vector<Marker *> vpCurrentMapMarkers = pMergeMap->GetAllMarkers();

    /* Copy all passages currently owned by the merge map */
    std::vector<ORB_SLAM3::Passage *> vpCurrentMapPassages =
        pMergeMap->GetAllPassages();

    /* Copy all detected rooms currently owned by the merge map */
    std::vector<Room *> vpCurrentDetectedMapRooms =
        pMergeMap->GetAllDetectedMapRooms();

    /* Copy all marker-based rooms currently owned by the merge map */
    std::vector<Room *> vpCurrentMarkerBasedMapRooms =
        pMergeMap->GetAllMarkerBasedMapRooms();

    /* Copy all floors currently owned by the merge map */
    std::vector<Floor *> vpCurrentMapFloors = pMergeMap->GetAllFloors();

    /* Stop local mapping before any remaining ownership is transferred. */
    mpLocalMapper->RequestStop();

    while (!mpLocalMapper->isStopped())
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
        if (mpTracker->mSensor == System::MONOCULAR)
        {
            /* Lock the merge map while updating its poses and landmarks */
            std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);

            /* Correct each remaining merge keyframe into current world */
            for (KeyFrame *pKFi : vpCurrentMapKFs)
            {
                /* Skip invalid keyframes or keyframes no longer in this map */
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the keyframe pose in the merge map world frame */
                const Sophus::SE3d TiwMerge = pKFi->GetPose().cast<double>();

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
                pKFi->mfScale = s;

                /* Preserve the original world to camera pose */
                pKFi->mTcwBefMerge = pKFi->GetPose();

                /* Preserve the original camera to world pose */
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                /* Apply the corrected rigid pose in the current world frame */
                pKFi->SetPose(Sophus::SE3d(g2oSiwCurrent.rotation(),
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
                    pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
                }
            }

            /* Correct each remaining merge landmark into current world */
            for (MapPoint *pMPi : vpCurrentMapMPs)
            {
                /* Skip invalid points or points no longer in this map */
                if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the landmark position in the merge world frame */
                const Eigen::Vector3d P3DwMerge =
                    pMPi->GetWorldPos().cast<double>();

                const Eigen::Vector3f normal_mergeWorld = pMPi->GetNormal();

                /* Transform the landmark into the current world frame */
                pMPi->SetWorldPos(
                    g2oSwCurrentWMerge.map(P3DwMerge).cast<float>());

                pMPi->SetNormalVector(
                    g2oSwCurrentWMerge.rotation().cast<float>() *
                    normal_mergeWorld);

                /* Refresh the point normal and valid viewing depth range */
                pMPi->UpdateNormalAndDepth();
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 18 - OPTIMISE THE COMPLETE MERGED GRAPH
         *
         * Once every remaining object has been corrected into the current-map
         * frame, optimise the complete merged graph before changing ownership.
         * ------------------------------------------------------------------ */

        if (mpTracker->mSensor != System::MONOCULAR)
        {
            Optimizer::OptimizeEssentialGraph(mpMergeMatchedKF,
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
        mpTracker->mSensor != System::MONOCULAR;

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
                p_keyFrame->GetPose().cast<double>();

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

        Utils::propagateSemanticPoseCorrections(
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
                p_mapPoint->GetMap() != pMergeMap)
            {
                continue;
            }

            const Eigen::Vector3f position_mergeWorld_m =
                p_mapPoint->GetWorldPos();

            const Eigen::Vector3f normal_mergeWorld = p_mapPoint->GetNormal();

            p_mapPoint->SetWorldPos(
                g2oSwCurrentWMerge.map(position_mergeWorld_m.cast<double>())
                    .cast<float>());

            p_mapPoint->SetNormalVector(
                g2oSwCurrentWMerge.rotation().cast<float>() *
                normal_mergeWorld);

            p_mapPoint->UpdateNormalAndDepth();
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
        for (Plane *p_existingPlane : pCurrentMap->GetAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        int nextPassageId = 0;
        for (ORB_SLAM3::Passage *p_existingPassage :
             pCurrentMap->GetAllPassages())
        {
            if (p_existingPassage != nullptr)
            {
                nextPassageId =
                    std::max(nextPassageId, p_existingPassage->getId() + 1);
            }
        }

        int nextRoomId = 0;
        for (Room *p_existingRoom : pCurrentMap->GetAllRooms())
        {
            if (p_existingRoom != nullptr)
            {
                nextRoomId = std::max(nextRoomId, p_existingRoom->getId() + 1);
            }
        }

        int nextFloorId = 0;
        for (Floor *p_existingFloor : pCurrentMap->GetAllFloors())
        {
            if (p_existingFloor != nullptr)
            {
                nextFloorId =
                    std::max(nextFloorId, p_existingFloor->getId() + 1);
            }
        }

        int nextMarkerId = 0;
        for (Marker *p_existingMarker : pCurrentMap->GetAllMarkers())
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
            if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
                continue;

            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        // Loop over the MapPoints of the current map and move them to the
        // new map
        for (MapPoint *pMPi : vpCurrentMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        /* -------------------------------------------------------------- *
         * SECTION 20 - TRANSFER SEMANTIC OBJECTS
         *
         * Semantic objects are transformed into the current-map reference
         * frame before ownership is transferred.
         *
         * Geometry is preserved while map ownership is updated.
         * -------------------------------------------------------------- */
        for (Plane *plane : vpCurrentMapPlanes)
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
            plane->SetMap(pCurrentMap);

            /*!
             * Take index size of planes in new map to find an id to add to
             * the map which hasn't been taken.
             */
            plane->setId(nextPlaneId++);

            /* Add the plane to the map new merged plane to the new map */
            pCurrentMap->AddMapPlane(plane);

            /* Remove the current plane from the old map */
            pMergeMap->EraseMapPlane(plane);
        }

        // Loop over the Markers of the current map and move them to the new
        // map
        for (Marker *pMarker : vpCurrentMapMarkers)
        {
            if (!pMarker)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pMarker->applyTransform(g2oSwCurrentWMerge);
            }

            pMarker->setMap(pCurrentMap);
            pMarker->setId(nextMarkerId++);
            pCurrentMap->AddMapMarker(pMarker);
            pMergeMap->EraseMapMarker(pMarker);
        }

        /*!
         * Loop over the passages of the primary map and move them to the
         * secondary map.
         */
        for (ORB_SLAM3::Passage *passage : vpCurrentMapPassages)
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

            /* Remove from the source index before changing its published ID. */
            pMergeMap->EraseMapPassage(passage);
            passage->setMap(pCurrentMap);

            /*!
             * Take index size of rooms in new map to find an id to add to
             * the map which hasn't been taken.
             */
            passage->setId(nextPassageId++);

            /* Add the room to the current map */
            pCurrentMap->AddMapPassage(passage);
        }

        /*!
         * Loop over the rooms of the primary map and move them to the
         * secondary map.
         */
        for (ORB_SLAM3::Room *room : vpCurrentDetectedMapRooms)
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

            /*!
             * Take index size of rooms in new map to find an id to add to
             * the map which hasn't been taken.
             */
            room->setId(nextRoomId++);

            /* Add the room to the current map */
            pCurrentMap->AddDetectedMapRoom(room);

            /* Remove the room from the merged map */
            pMergeMap->EraseDetectedMapRoom(room);
        }

        // Loop over the Marker-based Rooms of the current map and move them
        // to the new map
        for (ORB_SLAM3::Room *pRoom : vpCurrentMarkerBasedMapRooms)
        {
            if (!pRoom)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pRoom->applyTransform(g2oSwCurrentWMerge);
            }

            pRoom->setMap(pCurrentMap);
            pRoom->setId(nextRoomId++);
            pCurrentMap->AddCandidateMapRoom(pRoom);
            pMergeMap->EraseMarkerBasedMapRoom(pRoom);
        }

        for (Floor *p_floor : vpCurrentMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (!semanticGeometryWasPropagated)
            {
                p_floor->applyTransform(g2oSwCurrentWMerge);
            }
            p_floor->setMap(pCurrentMap);
            p_floor->setId(nextFloorId++);
            pCurrentMap->AddMapFloor(p_floor);
            pMergeMap->EraseMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /*
         * Voxblox topology is derived from a TSDF/ESDF volume and is not an
         * independently mergeable landmark set. Appending snapshots from two
         * map frames creates disconnected duplicate edges and false wall
         * crossings. The external Voxblox node receives the map-revision event,
         * clears its volume, and supplies a fresh snapshot after reintegration.
         */
        pCurrentMap->SetSkeletonClusterPoints({});
        pCurrentMap->SetSkeletonEdges({});

        /* Rebuild imported room-wall index entries before fusion. */
        for (Room *p_room : pCurrentMap->GetAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->AddRoomWallPlane(p_wall);
                }
            }
        }

        /* Fuse only after every semantic relationship is visible. */
        if (sysParams->sem_seg.reassociate.enabled)
        {
            Utils::reAssociateSemanticPlanes(mpAtlas);

            std::vector<Room *> importedRooms = vpCurrentDetectedMapRooms;
            importedRooms.insert(importedRooms.end(),
                                 vpCurrentMarkerBasedMapRooms.begin(),
                                 vpCurrentMarkerBasedMapRooms.end());

            Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap, importedRooms);
            Utils::reAssociateRooms(mpAtlas);
            Utils::reAssociatePassages(mpAtlas);
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

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();

    /*!
     * A map merge changes the world-frame poses of previously integrated
     * observations. Notify derived mapping consumers only after ownership,
     * semantic reconciliation, and graph connectivity are fully committed.
     * Voxblox uses this revision to discard TSDF/ESDF state expressed in the
     * pre-merge coordinate frame.
     */
    pCurrentMap->InformNewBigChange();

    /* All surviving objects now belong to pCurrentMap. */
    mpAtlas->ChangeMap(pCurrentMap);
    mpAtlas->SetMapBad(pMergeMap);
    mpAtlas->RemoveBadMaps();

    std::cout << "[FloorVerify] Map#" << pCurrentMap->GetId() << " and Map#"
              << pMergeMap->GetId() << " result=" << floorVerificationResult
              << " committed=1" << std::endl;

    semanticUpdateLock.unlock();
    mpLocalMapper->Release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }
}

void LoopClosing::MergeLocalInertial()
{
    /* Reject stale place-recognition candidates before stopping workers */
    if (mpCurrentKF == nullptr || mpMergeMatchedKF == nullptr ||
        mpCurrentKF->isBad() || mpMergeMatchedKF->isBad())
    {
        return;
    }

    Map *pCurrentMap = mpCurrentKF->GetMap();
    Map *pMergeMap   = mpMergeMatchedKF->GetMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->IsBad() ||
        pMergeMap->IsBad() || !mpAtlas->isActiveMap(pCurrentMap) ||
        !mpAtlas->isActiveMap(pMergeMap))
    {
        return;
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

    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    /* Keep the inertial semantic transfer atomic for the complete merge. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        mpAtlas->acquireSemanticUpdateLock();

    if (mpCurrentKF->isBad() || mpMergeMatchedKF->isBad() ||
        mpCurrentKF->GetMap() != pCurrentMap ||
        mpMergeMatchedKF->GetMap() != pMergeMap ||
        !mpAtlas->isActiveMap(pCurrentMap) || !mpAtlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        mpLocalMapper->Release();
        return;
    }

    std::string floorVerificationResult;
    if (!verifyLoopMergeFloors(pCurrentMap,
                               pMergeMap,
                               mSold_new.inverse(),
                               floorVerificationResult))
    {
        semanticUpdateLock.unlock();
        mpLocalMapper->Release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return;
    }

    {
        float        s_on = mSold_new.scale();
        Sophus::SE3f T_on(mSold_new.rotation().cast<float>(),
                          mSold_new.translation().cast<float>());

        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);

        mpLocalMapper->EmptyQueue();

        std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
        bool bScaleVel = false;
        if (s_on != 1)
            bScaleVel = true;
        pCurrentMap->ApplyScaledRotation(T_on, s_on, bScaleVel);
        mpTracker->UpdateFrameIMU(s_on,
                                  mpCurrentKF->GetImuBias(),
                                  mpTracker->GetLastKeyFrame());

        std::chrono::steady_clock::time_point t3 =
            std::chrono::steady_clock::now();
    }

    const int numKFnew = pCurrentMap->KeyFramesInMap();

    if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
         mpTracker->mSensor == System::IMU_STEREO ||
         mpTracker->mSensor == System::IMU_RGBD) &&
        !pCurrentMap->GetIniertialBA2())
    {
        /* Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap, bg, ba);
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);
        mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

        /* Set map initialized */
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();
    }

    /* Retain imported room identities for post-optimization reconciliation. */
    std::vector<Room *> importedRooms;

    /* Load KFs and MPs from merge map */
    {
        /*!
         * Acquire both map-update mutexes without imposing an unsafe order.
         *
         * @note        Get Merge Map Mutex and stop tracking.
         */
        std::scoped_lock mapUpdateLocks(pCurrentMap->mMutexMapUpdate,
                                        pMergeMap->mMutexMapUpdate);

        vector<KeyFrame *> vpMergeMapKFs     = pMergeMap->GetAllKeyFrames();
        vector<MapPoint *> vpMergeMapMPs     = pMergeMap->GetAllMapPoints();
        vector<Plane *>    vpMergeMapPlanes  = pMergeMap->GetAllPlanes();
        vector<Marker *>   vpMergeMapMarkers = pMergeMap->GetAllMarkers();
        vector<ORB_SLAM3::Passage *> vpMergeMapPassages =
            pMergeMap->GetAllPassages();
        vector<Room *> vpMergeMapDetectedRooms =
            pMergeMap->GetAllDetectedMapRooms();
        vector<Room *> vpMergeMapMarkerRooms =
            pMergeMap->GetAllMarkerBasedMapRooms();
        vector<Floor *> vpMergeMapFloors = pMergeMap->GetAllFloors();

        importedRooms = vpMergeMapDetectedRooms;
        importedRooms.insert(importedRooms.end(),
                             vpMergeMapMarkerRooms.begin(),
                             vpMergeMapMarkerRooms.end());

        for (KeyFrame *pKFi : vpMergeMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        for (MapPoint *pMPi : vpMergeMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        int nextPlaneId = 0;
        for (Plane *p_existingPlane : pCurrentMap->GetAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        for (Plane *p_plane : vpMergeMapPlanes)
        {
            if (p_plane == nullptr || p_plane->isBad())
            {
                continue;
            }

            p_plane->SetMap(pCurrentMap);
            p_plane->setId(nextPlaneId++);
            pCurrentMap->AddMapPlane(p_plane);
            pMergeMap->EraseMapPlane(p_plane);
        }

        int nextMarkerId = 0;
        for (Marker *p_existingMarker : pCurrentMap->GetAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                nextMarkerId =
                    std::max(nextMarkerId, p_existingMarker->getId() + 1);
            }
        }

        for (Marker *p_marker : vpMergeMapMarkers)
        {
            if (p_marker == nullptr)
            {
                continue;
            }

            p_marker->setMap(pCurrentMap);
            p_marker->setId(nextMarkerId++);
            pCurrentMap->AddMapMarker(p_marker);
            pMergeMap->EraseMapMarker(p_marker);
        }

        int nextPassageId = 0;
        for (ORB_SLAM3::Passage *p_existingPassage :
             pCurrentMap->GetAllPassages())
        {
            if (p_existingPassage != nullptr)
            {
                nextPassageId =
                    std::max(nextPassageId, p_existingPassage->getId() + 1);
            }
        }

        for (ORB_SLAM3::Passage *p_passage : vpMergeMapPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            pMergeMap->EraseMapPassage(p_passage);
            p_passage->setMap(pCurrentMap);
            p_passage->setId(nextPassageId++);
            pCurrentMap->AddMapPassage(p_passage);
        }

        int nextRoomId = 0;
        for (Room *p_existingRoom : pCurrentMap->GetAllRooms())
        {
            if (p_existingRoom != nullptr)
            {
                nextRoomId = std::max(nextRoomId, p_existingRoom->getId() + 1);
            }
        }

        for (Room *p_room : vpMergeMapDetectedRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            p_room->setId(nextRoomId++);
            pCurrentMap->AddDetectedMapRoom(p_room);
            pMergeMap->EraseDetectedMapRoom(p_room);
        }

        for (Room *p_room : vpMergeMapMarkerRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            p_room->setId(nextRoomId++);
            pCurrentMap->AddCandidateMapRoom(p_room);
            pMergeMap->EraseMarkerBasedMapRoom(p_room);
        }

        int nextFloorId = 0;
        for (Floor *p_existingFloor : pCurrentMap->GetAllFloors())
        {
            if (p_existingFloor != nullptr)
            {
                nextFloorId =
                    std::max(nextFloorId, p_existingFloor->getId() + 1);
            }
        }

        for (Floor *p_floor : vpMergeMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            p_floor->setMap(pCurrentMap);
            p_floor->setId(nextFloorId++);
            pCurrentMap->AddMapFloor(p_floor);
            pMergeMap->EraseMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /* Rebuild derived free-space topology in the corrected map frame. */
        pCurrentMap->SetSkeletonClusterPoints({});
        pCurrentMap->SetSkeletonEdges({});

        for (Room *p_room : pCurrentMap->GetAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->AddRoomWallPlane(p_wall);
                }
            }
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame *> vpKFs = pCurrentMap->GetAllKeyFrames();
        for (KeyFrame *pKFi : vpKFs)
        {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
    }

    if (pMergeMap->GetOriginKF() != nullptr)
    {
        pMergeMap->GetOriginKF()->SetFirstConnection(false);
    }
    pNewChild =
        mpMergeMatchedKF
            ->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of
                                   // its own parent(we need eliminate this KF
                                   // from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);
    while (pNewChild)
    {
        pNewChild->EraseChild(
            pNewParent); // We remove the relation between the old parent and
                         // the new for avoid loop
        KeyFrame *pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild  = pOldParent;
    }

    vector<MapPoint *>
        vpCheckFuseMapPoint; // MapPoint vector from current map to allow to
                             // fuse duplicated points with the old map (merge)
    vector<KeyFrame *> vpCurrentConnectedKFs;

    mvpMergeConnectedKFs.clear();
    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<KeyFrame *> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(),
                                aux.begin(),
                                aux.end());
    if (mvpMergeConnectedKFs.size() > 6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6,
                                   mvpMergeConnectedKFs.end());

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(),
                                 aux.begin(),
                                 aux.end());
    if (vpCurrentConnectedKFs.size() > 6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6,
                                    vpCurrentConnectedKFs.end());

    set<MapPoint *> spMapPointMerge;
    for (KeyFrame *pKFi : mvpMergeConnectedKFs)
    {
        set<MapPoint *> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        if (spMapPointMerge.size() > 1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(),
              spMapPointMerge.end(),
              std::back_inserter(vpCheckFuseMapPoint));
    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for (KeyFrame *pKFi : vpCurrentConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    const auto finalizeInertialMerge = [this, pCurrentMap, pMergeMap]()
    {
        mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
        mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);
        pCurrentMap->IncreaseChangeIndex();

        /*!
         * Inertial welding changes the same derived-map coordinate contract
         * as a visual merge. Increment the externally observed revision after
         * all corrected poses and semantic entities have become authoritative.
         */
        pCurrentMap->InformNewBigChange();

        mpAtlas->ChangeMap(pCurrentMap);
        mpAtlas->SetMapBad(pMergeMap);
        mpAtlas->RemoveBadMaps();
    };
    for (KeyFrame *pKFi : mvpMergeConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    /* A sufficiently established current map can support inertial welding BA.
     */
    bool inertialBundleAdjustmentRan = false;

    if (numKFnew >= 10)
    {
        bool      bStopFlag         = false;
        KeyFrame *p_currentKeyFrame = mpTracker->GetLastKeyFrame();

        if (p_currentKeyFrame != nullptr)
        {
            Optimizer::MergeInertialBA(p_currentKeyFrame,
                                       mpMergeMatchedKF,
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
                p_keyFrame->GetMap() != pCurrentMap)
            {
                continue;
            }

            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->GetPose().cast<double>();

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

        Utils::propagateSemanticPoseCorrections(pCurrentMap,
                                                NonCorrectedSim3,
                                                CorrectedSim3,
                                                identityTransform_WorldToWorld);
    }

    /* Fuse semantic hypotheses only after the final inertial pose correction.
     */
    if (SystemParams::GetParams()->sem_seg.reassociate.enabled)
    {
        Utils::reAssociateSemanticPlanes(mpAtlas);

        Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap, importedRooms);
        Utils::reAssociateRooms(mpAtlas);
        Utils::reAssociatePassages(mpAtlas);
    }

    finalizeInertialMerge();

    std::cout << "[FloorVerify] Map#" << pCurrentMap->GetId() << " and Map#"
              << pMergeMap->GetId() << " result=" << floorVerificationResult
              << " committed=1" << std::endl;

    /* The semantic graph and Atlas ownership are now stable for other workers.
     */
    semanticUpdateLock.unlock();

    mpLocalMapper->Release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }

    return;
}

void LoopClosing::CheckObservations(set<KeyFrame *> &spKFsMap1,
                                    set<KeyFrame *> &spKFsMap2)
{
    cout << "----------------------" << endl;
    for (KeyFrame *pKFi1 : spKFsMap1)
    {
        map<KeyFrame *, int> mMatchedMP;
        set<MapPoint *>      spMPs = pKFi1->GetMapPoints();

        for (MapPoint *pMPij : spMPs)
        {
            if (!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFrame *, tuple<int, int>> mMPijObs =
                pMPij->GetObservations();
            for (KeyFrame *pKFi2 : spKFsMap2)
            {
                if (mMPijObs.find(pKFi2) != mMPijObs.end())
                {
                    if (mMatchedMP.find(pKFi2) != mMatchedMP.end())
                    {
                        mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                    }
                    else
                    {
                        mMatchedMP[pKFi2] = 1;
                    }
                }
            }
        }

        if (mMatchedMP.size() == 0)
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId
                 << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with "
                 << mMatchedMP.size() << " KF from the other map" << endl;
            for (pair<KeyFrame *, int> matchedKF : mMatchedMP)
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
    return mbMergeInProgress.load();
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
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
        Map      *pMap         = pKFi->GetMap();

        g2o::Sim3     g2oScw = mit->second;
        Sophus::Sim3f Scw    = Converter::toSophus(g2oScw);

        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(NULL));
        int numFused = matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int          nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep)
            {

                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }

        total_replaces += num_replaces;
    }
    // cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
}

void LoopClosing::SearchAndFuse(const vector<KeyFrame *> &vConectedKFs,
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
        Map          *pMap         = pKF->GetMap();
        Sophus::SE3f  Tcw          = pKF->GetPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(NULL));
        matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int          nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep)
            {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << "
        MPs fused" << endl; total_replaces += num_replaces;*/
    }
    // cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
}

void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset              = pMap;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid =
            0; // TODO old variable, it is not use in the new algorithm
        mbResetRequested          = false;
        mbResetActiveMapRequested = false;
    }
    else if (mbResetActiveMapRequested)
    {

        for (list<KeyFrame *>::const_iterator it = mlpLoopKeyFrameQueue.begin();
             it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFrame *pKFi = *it;
            if (pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid =
            mpAtlas->GetLastInitKFid(); // TODO old variable, it is not use in
                                        // the new algorithm
        mbResetActiveMapRequested = false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(Map          *pActiveMap,
                                            unsigned long nLoopKF,
                                            unsigned int  generation_in)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment",
                       Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA =
        std::chrono::steady_clock::now();

    nFGBA_exec += 1;

    vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
    vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

    /*
     * g2o accepts a plain bool force-stop token. A first-party iteration action
     * copies the atomic cross-thread request into this worker-local flag, so
     * cancellation remains race-free and takes effect between iterations.
     */
    bool optimizerStopRequested = false;

    const bool bImuInit = pActiveMap->isImuInitialized();

    if (!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap,
                                          10,
                                          &optimizerStopRequested,
                                          nLoopKF,
                                          false,
                                          mpTracker->GetMarkerImpact(),
                                          &globalBundleAdjustmentStopRequested);
    else
        Optimizer::FullInertialBA(pActiveMap,
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
        if (generation_in != mnFullBAIdx)
        {
            mbFinishedGBA = true;
            mbRunningGBA  = false;
            return;
        }

        if (!bImuInit && pActiveMap->isImuInitialized())
        {
            mbFinishedGBA = true;
            mbRunningGBA  = false;
            return;
        }

        if (!optimizerStopRequested)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished",
                               Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            std::unique_lock<std::mutex> semanticUpdateLock =
                mpAtlas->acquireSemanticUpdateLock();

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

            KeyFrameAndPose keyFramePosesBefore_WorldToCamera;
            KeyFrameAndPose keyFramePosesAfter_WorldToCamera;

            //  Correct keyframes starting at map first keyframe
            list<KeyFrame *> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),
                                         pActiveMap->mvpKeyFrameOrigins.end());

            while (!lpKFtoCheck.empty())
            {
                KeyFrame             *pKF     = lpKFtoCheck.front();
                const set<KeyFrame *> sChilds = pKF->GetChilds();
                Sophus::SE3f          Twc     = pKF->GetPoseInverse();
                for (set<KeyFrame *>::const_iterator sit = sChilds.begin();
                     sit != sChilds.end();
                     sit++)
                {
                    KeyFrame *pChild = *sit;
                    if (!pChild || pChild->isBad())
                        continue;

                    if (pChild->mnBAGlobalForKF != nLoopKF)
                    {
                        Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                        pChild->mTcwGBA =
                            Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() *
                                            pChild->GetPose().so3();
                        if (pChild->isVelocitySet())
                        {
                            pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ",
                                               Verbose::VERBOSITY_NORMAL);

                        pChild->mBiasGBA = pChild->GetImuBias();

                        pChild->mnBAGlobalForKF = nLoopKF;
                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();

                const Sophus::SE3d poseBefore_WorldToCamera =
                    pKF->mTcwBefGBA.cast<double>();

                keyFramePosesBefore_WorldToCamera.insert_or_assign(
                    pKF,
                    g2o::Sim3(poseBefore_WorldToCamera.unit_quaternion(),
                              poseBefore_WorldToCamera.translation(),
                              1.0));

                pKF->SetPose(pKF->mTcwGBA);

                const Sophus::SE3d poseAfter_WorldToCamera =
                    pKF->GetPose().cast<double>();

                keyFramePosesAfter_WorldToCamera.insert_or_assign(
                    pKF,
                    g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                              poseAfter_WorldToCamera.translation(),
                              1.0));

                if (pKF->bImu)
                {
                    pKF->mVwbBefGBA = pKF->GetVelocity();

                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);
                }

                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint *> vpMPs = pActiveMap->GetAllMapPoints();

            for (size_t i = 0; i < vpMPs.size(); i++)
            {
                MapPoint *pMP = vpMPs[i];

                if (pMP == nullptr || pMP->isBad())
                    continue;

                bool mapPointWasCorrected = false;

                if (pMP->mnBAGlobalForKF == nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                    mapPointWasCorrected = true;
                }
                else
                {
                    // Update according to the correction of its reference
                    // keyframe
                    KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                    if (pRefKF == nullptr || pRefKF->isBad() ||
                        pRefKF->GetMap() != pActiveMap ||
                        pRefKF->mnBAGlobalForKF != nLoopKF)
                    {
                        pRefKF = nullptr;

                        const auto observations = pMP->GetObservations();

                        for (const auto &[p_observingKeyFrame, featureIndexes] :
                             observations)
                        {
                            (void)featureIndexes;

                            if (p_observingKeyFrame == nullptr ||
                                p_observingKeyFrame->isBad() ||
                                p_observingKeyFrame->GetMap() != pActiveMap ||
                                p_observingKeyFrame->mnBAGlobalForKF != nLoopKF)
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
                    Eigen::Vector3f Xc =
                        pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                    // Backproject using corrected camera
                    pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                    mapPointWasCorrected = true;
                }

                if (mapPointWasCorrected)
                {
                    pMP->UpdateNormalAndDepth();
                }
            }

            const g2o::Sim3 identityTransform_WorldToWorld(
                Eigen::Quaterniond::Identity(),
                Eigen::Vector3d::Zero(),
                1.0);

            /* Keep every semantic entity aligned with the corrected cameras. */
            Utils::propagateSemanticPoseCorrections(
                pActiveMap,
                keyFramePosesBefore_WorldToCamera,
                keyFramePosesAfter_WorldToCamera,
                identityTransform_WorldToWorld);

            /* Preserve plane variables which were optimized directly by GBA. */
            for (Plane *p_plane : pActiveMap->GetAllPlanes())
            {
                if (p_plane == nullptr || p_plane->isBad() ||
                    p_plane->mnBAGlobalForKF != nLoopKF)
                {
                    continue;
                }

                p_plane->alignGeometryToEquation(p_plane->mPlaneGBA);
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f,
            // mpTracker->GetLastKeyFrame()->GetImuBias(),
            // mpTracker->GetLastKeyFrame());

            mpLocalMapper->Release();

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
            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA  = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    // cout << "LC: Finish requested" << endl;
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} // namespace ORB_SLAM3
