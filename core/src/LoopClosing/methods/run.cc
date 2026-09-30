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

#include "G2oTypes.h"
#include "Semantic/SemanticVerify.h"
#include "System.h"
#include "Tracking.h"

#include <chrono>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void LoopClosing::run(void)
{
    /*!
     * Mark the LoopClosing worker as active. SetFinish() changes this back to
     * true when Run() exits.
     *
     * This flag is observed by other threads through isFinished().
     */
    hasFinished = false;

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
        bool hasNewKeyFrames{};
        if (checkNewKeyFrames(hasNewKeyFrames) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkNewKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (hasNewKeyFrames)
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
            std::chrono::steady_clock::time_point timeStartPr =
                std::chrono::steady_clock::now();
#endif

            /*!
             * Pop/process the next queued keyframe and look for a same-map loop
             * or cross-map merge candidate. This function consumes the next KF,
             * queries the database, validates Sim3 geometry, and updates
             * mbLoopDetected / mbMergeDetected plus their matched-KF state.
             */
            bool isFindedRegion{};
            if (newDetectCommonRegions(isFindedRegion) !=
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: newDetectCommonRegions returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point timeEndPr =
                std::chrono::steady_clock::now();

            double timePrTotal = std::chrono::duration_cast<
                                     std::chrono::duration<double, std::milli>>(
                                     timeEndPr - timeStartPr)
                                     .count();
            placeRecognitionTotalTimes_ms.push_back(timePrTotal);
#endif

            /* If a detected region is found, perform loop closure */
            if (isFindedRegion)
            {
                /* Merge if NewDetectCommonRegions() indicates so */
                if (isMergeDetected)
                {
                    semantic::SemanticMergeDecision mergeDecision =
                        semantic::SemanticMergeDecision::REJECT;

                    /* If required, confirm IMU is working */
                    Map *p_currentKFMap = nullptr;
                    if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                          p_tracker->sensor == System::IMU_STEREO ||
                          p_tracker->sensor == System::IMU_RGBD)) &&
                        p_currentKF->getMap(p_currentKFMap) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isImuInitialized2{};
                    if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                          p_tracker->sensor == System::IMU_STEREO ||
                          p_tracker->sensor == System::IMU_RGBD)) &&
                        p_currentKFMap->isImuInitialized(isImuInitialized2) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isImuInitialized returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                         p_tracker->sensor == System::IMU_STEREO ||
                         p_tracker->sensor == System::IMU_RGBD) &&
                        (!isImuInitialized2))
                    {
                        std::cout << "IMU is not initilized, merge is aborted"
                                  << std::endl;
                    }
                    else
                    {
                        /*!
                         * Get pose of the matched keyframe in the matched
                         * keyframes world frame.
                         */
                        Sophus::SE3f mergeMatchedKFPose{};
                        if (p_mergeMatchedKF->getPose(mergeMatchedKFPose) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3d mTmw = mergeMatchedKFPose.cast<double>();

                        /* Convert above keyframe pose into Sim3 datatype */
                        g2o::Sim3 gSmw2(mTmw.unit_quaternion(),
                                        mTmw.translation(),
                                        1.0);

                        /*!
                         * Get pose of the current keyframe in the current
                         * keyframes world frame.
                         */
                        Sophus::SE3f currentKFPose{};
                        if (p_currentKF->getPose(currentKFPose) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3d mTcw = currentKFPose.cast<double>();

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
                        Map *p_currentKFMap2 = nullptr;
                        if (p_currentKF->getMap(p_currentKFMap2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        bool isInertial2{};
                        if (p_currentKFMap2->isInertial(isInertial2) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isInertial returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Map *p_mergeMatchedKFMap = nullptr;
                        if ((isInertial2) &&
                            p_mergeMatchedKF->getMap(p_mergeMatchedKFMap) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        bool isInertial3{};
                        if ((isInertial2) &&
                            p_mergeMatchedKFMap->isInertial(isInertial3) !=
                                MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isInertial returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        if (isInertial2 && isInertial3)
                        {
                            std::cout << "Merge check transformation with IMU"
                                      << std::endl;

                            /* Reject maps with bad scale */
                            if (oldCorrectedPose.scale() < 0.90 ||
                                oldCorrectedPose.scale() > 1.1)
                            {
                                if (p_mergeLastCurrentKF->setErase() !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: setErase returned a failure "
                                        "status although it cannot fail; "
                                        "continuing as before.",
                                        __func__);
                                }
                                if (p_mergeMatchedKF->setErase() !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: setErase returned a failure "
                                        "status although it cannot fail; "
                                        "continuing as before.",
                                        __func__);
                                }
                                mergeNumCoincidences = 0;
                                mergeMatchedMPs.clear();
                                mergeMPs.clear();
                                mergeNumNotFound = 0;
                                isMergeDetected  = false;
                                if (Verbose::printMess(
                                        "scale bad estimated. Abort merging",
                                        Verbose::VERBOSITY_NORMAL) !=
                                    VerboseStatus::VERBOSE_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: printMess returned a failure "
                                        "status although it cannot fail; "
                                        "continuing as before.",
                                        __func__);
                                }
                                continue;
                            }
                            // If inertial, force only yaw
                            Map *p_currentKFMap3 = nullptr;
                            if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                                  p_tracker->sensor == System::IMU_STEREO ||
                                  p_tracker->sensor == System::IMU_RGBD)) &&
                                p_currentKF->getMap(p_currentKFMap3) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getMap returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            bool inertialBA1{};
                            if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                                  p_tracker->sensor == System::IMU_STEREO ||
                                  p_tracker->sensor == System::IMU_RGBD)) &&
                                p_currentKFMap3->getInertialBA1(inertialBA1) !=
                                    MapStatus::MAP_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getInertialBA1 returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                                 p_tracker->sensor == System::IMU_STEREO ||
                                 p_tracker->sensor == System::IMU_RGBD) &&
                                inertialBA1)
                            {
                                Eigen::Vector3d phi{};
                                if (logSO3(oldCorrectedPose.rotation()
                                               .toRotationMatrix(),
                                           phi) !=
                                    G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: logSO3 returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                phi(0) = 0;
                                phi(1) = 0;
                                Eigen::Matrix3d rotation2{};
                                if (expSO3(phi, rotation2) !=
                                    G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: expSO3 returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                oldCorrectedPose =
                                    g2o::Sim3(rotation2,
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
                        std::chrono::steady_clock::time_point timeStartMerge =
                            std::chrono::steady_clock::now();
#endif

                        /* Set flag to indicate that mergins is happening */
                        if (setMergeStatus(true) !=
                            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setMergeStatus returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }

                        /* Choose merging method based on if IMU is used */
                        if (p_tracker->sensor == System::IMU_MONOCULAR ||
                            p_tracker->sensor == System::IMU_STEREO ||
                            p_tracker->sensor == System::IMU_RGBD)
                        {
                            /* Merge maps using IMU */
                            semantic::SemanticMergeDecision localInertial{};
                            if (mergeLocalInertial(localInertial) !=
                                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: mergeLocalInertial returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            mergeDecision = localInertial;
                        }
                        else
                        {
                            /* Merge maps */
                            semantic::SemanticMergeDecision local{};
                            if (mergeLocal(local) !=
                                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: mergeLocal returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            mergeDecision = local;
                        }

                        /* Set flag to indicate that mergins has finished */
                        if (setMergeStatus(false) !=
                            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setMergeStatus returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }

#ifdef REGISTER_TIMES
                        if (mergeDecision ==
                            semantic::SemanticMergeDecision::ACCEPT)
                        {
                            std::chrono::steady_clock::time_point timeEndMerge =
                                std::chrono::steady_clock::now();

                            double timeMergeTotal =
                                std::chrono::duration_cast<
                                    std::chrono::duration<double, std::milli>>(
                                    timeEndMerge - timeStartMerge)
                                    .count();
                            mergeTotalTimes_ms.push_back(timeMergeTotal);
                            mergeCount += 1;
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
                        placeRecognitionCurrentTimes.push_back(
                            p_currentKF->timeStamp);
                        placeRecognitionMatchedTimes.push_back(
                            p_mergeMatchedKF->timeStamp);
                        placeRecognitionTypes.push_back(1);

                        if (p_mergeLastCurrentKF->setErase() !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setErase returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        if (p_mergeMatchedKF->setErase() !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setErase returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        mergeNumCoincidences = 0;
                        mergeMatchedMPs.clear();
                        mergeMPs.clear();
                        mergeNumNotFound = 0;
                        isMergeDetected  = false;

                        /* A committed merge invalidates any same-map loop
                         * candidate collected against the old topology. */
                        if (isLoopDetected)
                        {
                            if (recordLoopCorrectionEvent(
                                    false,
                                    "superseded_by_map_merge") !=
                                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: recordLoopCorrectionEvent returned a "
                                    "failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if (p_loopLastCurrentKF->setErase() !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: setErase returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            if (p_loopMatchedKF->setErase() !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: setErase returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            loopNumCoincidences = 0;
                            loopMatchedMPs.clear();
                            loopMPs.clear();
                            loopNumNotFound = 0;
                            isLoopDetected  = false;
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
                        if (p_mergeLastCurrentKF->setErase() !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setErase returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        if (p_mergeMatchedKF->setErase() !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setErase returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        mergeNumCoincidences = 0;
                        mergeMatchedMPs.clear();
                        mergeMPs.clear();
                        mergeNumNotFound = 0;
                        isMergeDetected  = false;
                    }
                }

                /*!
                 * A loop closure candidate has been successfully detected and
                 * geometrically validated. The matched keyframe belongs to the
                 * same map as the current keyframe, meaning the estimated Sim3
                 * transformation can be used to correct accumulated drift in
                 * the map.
                 */
                if (isLoopDetected)
                {
                    std::cout
                        << "[LoopClosing] Loop detected! Correcting the map ..."
                        << std::endl;

                    /* Init a variable to track of a good loop closure occurs */
                    bool isGoodLoop = true;

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
                     * placeRecognitionTypes identifies the recognition type:
                     *   0 -> loop closure
                     *   1 -> map merge
                     */
                    placeRecognitionCurrentTimes.push_back(
                        p_currentKF->timeStamp);
                    placeRecognitionMatchedTimes.push_back(
                        p_loopMatchedKF->timeStamp);
                    placeRecognitionTypes.push_back(0);

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
                    Map *p_currentKFMap4 = nullptr;
                    if (p_currentKF->getMap(p_currentKFMap4) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isInertial4{};
                    if (p_currentKFMap4->isInertial(isInertial4) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isInertial returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (isInertial4)
                    {
                        Sophus::SE3f currentKFPoseInverse{};
                        if (p_currentKF->getPoseInverse(currentKFPoseInverse) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPoseInverse returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3d poseCameraToWorld =
                            currentKFPoseInverse.cast<double>();

                        /*!
                         * Convert the current camera pose from SE3 into a Sim3
                         * representation so that it can be combined with the
                         * loop correction estimate.
                         */
                        g2o::Sim3 g2oTwc(poseCameraToWorld.unit_quaternion(),
                                         poseCameraToWorld.translation(),
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
                        Eigen::Vector3d phi{};
                        if (logSO3(g2oSww_new.rotation().toRotationMatrix(),
                                   phi) !=
                            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: logSO3 returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }

                        if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f &&
                            fabs(phi(2)) < 0.349f)
                        {
                            /*!
                             * After inertial initialization, roll and pitch are
                             * constrained by gravity. Therefore only the yaw
                             * component of the loop correction is allowed to
                             * modify the map orientation.
                             */
                            Map *p_currentKFMap5 = nullptr;
                            if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                                  p_tracker->sensor == System::IMU_STEREO ||
                                  p_tracker->sensor == System::IMU_RGBD)) &&
                                p_currentKF->getMap(p_currentKFMap5) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getMap returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            bool inertialBA2{};
                            if (((p_tracker->sensor == System::IMU_MONOCULAR ||
                                  p_tracker->sensor == System::IMU_STEREO ||
                                  p_tracker->sensor == System::IMU_RGBD)) &&
                                p_currentKFMap5->getInertialBA2(inertialBA2) !=
                                    MapStatus::MAP_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getInertialBA2 returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if ((p_tracker->sensor == System::IMU_MONOCULAR ||
                                 p_tracker->sensor == System::IMU_STEREO ||
                                 p_tracker->sensor == System::IMU_RGBD) &&
                                inertialBA2)
                            {
                                phi(0) = 0;
                                phi(1) = 0;
                                Eigen::Matrix3d rotation3{};
                                if (expSO3(phi, rotation3) !=
                                    G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: expSO3 returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                g2oSww_new = g2o::Sim3(rotation3,
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
                            isGoodLoop = false;
                            if (recordLoopCorrectionEvent(false,
                                                          "inertial_overlap") !=
                                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: recordLoopCorrectionEvent returned a "
                                    "failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                        }
                    }

                    /*!
                     * Apply the loop correction only after all validation
                     * checks have passed. CorrectLoop() performs the map
                     * optimisation and updates keyframe/map point poses to
                     * remove accumulated drift.
                     */
                    if (isGoodLoop)
                    {
                        loopMapPoints = loopMPs;

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point timeStartLoop =
                            std::chrono::steady_clock::now();

                        loopCount += 1;

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
                        if (correctLoop() !=
                            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: correctLoop returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point timeEndLoop =
                            std::chrono::steady_clock::now();

                        double timeLoopTotal =
                            std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                timeEndLoop - timeStartLoop)
                                .count();
                        loopTotalTimes_ms.push_back(timeLoopTotal);
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
                        if (recordLoopCorrectionEvent(true, "corrected") !=
                            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: recordLoopCorrectionEvent returned a "
                                "failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }
                    }

                    /*!
                     * Release all temporary loop closure state.
                     *
                     * The candidate keyframes are no longer required because
                     * the correction has either been applied or rejected.
                     * Resetting these variables allows future loop closure
                     * attempts to start from a clean state.
                     */
                    if (p_loopLastCurrentKF->setErase() !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setErase returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_loopMatchedKF->setErase() !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setErase returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    loopNumCoincidences = 0;
                    loopMatchedMPs.clear();
                    loopMPs.clear();
                    loopNumNotFound = 0;
                    isLoopDetected  = false;
                }
            }
            p_lastCurrentKF = p_currentKF;
        }

        if (resetIfRequested() !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resetIfRequested returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool shouldFinish{};
        if (checkFinish(shouldFinish) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (shouldFinish)
        {
            break;
        }

        /* Find the time after it took to run the loop */
        const std::chrono::system_clock::time_point end =
            std::chrono::high_resolution_clock::now();

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
    bool wasRunning{};
    if (stopGlobalBundleAdjustment(wasRunning) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: stopGlobalBundleAdjustment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (setFinish() != LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
