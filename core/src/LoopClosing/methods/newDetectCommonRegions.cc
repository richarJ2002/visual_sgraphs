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

#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "System.h"
#include "Tracking.h"

#include <chrono>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus LoopClosing::newDetectCommonRegions(bool &isDetected_out)
{
    // To deactivate placerecognition. No loopclosing nor merging will be
    // performed
    if (!isLoopClosingActive)
    {
        isDetected_out = false;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    {
        std::unique_lock<std::mutex> lock(loopQueueMutex);
        p_currentKF = loopKeyFrameQueue.front();
        loopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this
        // thread
        if (p_currentKF->setNotErase() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setNotErase returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_currentKF->isInCurrentPlaceRecognition = true;

        Map *p_currentKFMap = nullptr;
        if (p_currentKF->getMap(p_currentKFMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_lastMap = p_currentKFMap;
    }

    bool lastMapIsInertial{};
    if (p_lastMap->isInertial(lastMapIsInertial) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool lastMapInertialBA2{};
    if ((lastMapIsInertial) && p_lastMap->getInertialBA2(lastMapInertialBA2) !=
                                   MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInertialBA2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (lastMapIsInertial && !lastMapInertialBA2)
    {
        if (p_keyFrameDatabase->add(p_currentKF) !=
            KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: add returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        if (p_currentKF->setErase() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setErase returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isDetected_out = false;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    std::vector<KeyFrame *> lastMapAllKeyFrames{};
    if ((p_tracker->sensor == System::STEREO) &&
        p_lastMap->getAllKeyFrames(lastMapAllKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_tracker->sensor == System::STEREO &&
        lastMapAllKeyFrames.size() < 5) // 12
    {
        if (p_keyFrameDatabase->add(p_currentKF) !=
            KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: add returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        if (p_currentKF->setErase() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setErase returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isDetected_out = false;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    std::vector<KeyFrame *> lastMapAllKeyFrames2{};
    if (p_lastMap->getAllKeyFrames(lastMapAllKeyFrames2) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (lastMapAllKeyFrames2.size() < 12)
    {
        if (p_keyFrameDatabase->add(p_currentKF) !=
            KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: add returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        if (p_currentKF->setErase() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setErase returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isDetected_out = false;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    // Check the last candidates with geometric validation
    //  Loop candidates
    bool isLoopDetectedInKeyFrame = false;
    bool shouldCheckSpatial       = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_1 =
        std::chrono::steady_clock::now();
#endif
    if (loopNumCoincidences > 0)
    {
        shouldCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3f currentKFPose{};
        if (p_currentKF->getPose(currentKFPose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f loopLastCurrentKFPoseInverse{};
        if (p_loopLastCurrentKF->getPoseInverse(loopLastCurrentKFPoseInverse) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d mTcl =
            (currentKFPose * loopLastCurrentKFPoseInverse).cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw                 = gScl * mg2oLoopSlw;
        int       projectionMatchCount = 0;
        std::vector<MapPoint *> matchedMapPoints;
        bool                    isCommonRegionFound{};
        if (detectAndReffineSim3FromLastKF(p_currentKF,
                                           p_loopMatchedKF,
                                           gScw,
                                           projectionMatchCount,
                                           loopMPs,
                                           matchedMapPoints,
                                           isCommonRegionFound) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detectAndReffineSim3FromLastKF returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (isCommonRegionFound)
        {

            isLoopDetectedInKeyFrame = true;

            loopNumCoincidences++;
            if (p_loopLastCurrentKF->setErase() !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setErase returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_loopLastCurrentKF = p_currentKF;
            mg2oLoopSlw         = gScw;
            loopMatchedMPs      = matchedMapPoints;

            isLoopDetected  = loopNumCoincidences >= 3;
            loopNumNotFound = 0;
        }
        else
        {
            isLoopDetectedInKeyFrame = false;

            loopNumNotFound++;
            if (loopNumNotFound >= 2)
            {
                if (recordLoopCorrectionEvent(false, "geometric_validation") !=
                    LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: recordLoopCorrectionEvent returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_loopLastCurrentKF->setErase() !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setErase returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_loopMatchedKF->setErase() !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setErase returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                loopNumCoincidences = 0;
                loopMatchedMPs.clear();
                loopMPs.clear();
                loopNumNotFound = 0;
            }
        }
    }

    // Merge candidates
    bool isMergeDetectedInKeyFrame = false;
    if (mergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3f currentKFPose2{};
        if (p_currentKF->getPose(currentKFPose2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f mergeLastCurrentKFPoseInverse{};
        if (p_mergeLastCurrentKF->getPoseInverse(
                mergeLastCurrentKFPoseInverse) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d mTcl =
            (currentKFPose2 * mergeLastCurrentKFPoseInverse).cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw                 = gScl * mg2oMergeSlw;
        int       projectionMatchCount = 0;
        std::vector<MapPoint *> matchedMapPoints;
        bool                    isCommonRegionFound{};
        if (detectAndReffineSim3FromLastKF(p_currentKF,
                                           p_mergeMatchedKF,
                                           gScw,
                                           projectionMatchCount,
                                           mergeMPs,
                                           matchedMapPoints,
                                           isCommonRegionFound) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detectAndReffineSim3FromLastKF returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (isCommonRegionFound)
        {
            isMergeDetectedInKeyFrame = true;

            mergeNumCoincidences++;
            if (p_mergeLastCurrentKF->setErase() !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setErase returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_mergeLastCurrentKF = p_currentKF;
            mg2oMergeSlw         = gScw;
            mergeMatchedMPs      = matchedMapPoints;

            isMergeDetected = mergeNumCoincidences >= 3;
        }
        else
        {
            isMergeDetected           = false;
            isMergeDetectedInKeyFrame = false;

            mergeNumNotFound++;
            if (mergeNumNotFound >= 2)
            {
                if (p_mergeLastCurrentKF->setErase() !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setErase returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_mergeMatchedKF->setErase() !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setErase returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
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

    if (isMergeDetected || isLoopDetected)
    {
#ifdef REGISTER_TIMES
        sim3EstimationTimes_ms.push_back(timeEstSim3);
#endif
        if (p_keyFrameDatabase->add(p_currentKF) !=
            KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: add returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        isDetected_out = true;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    // TODO: This is only necessary if we use a minimun score for pick the best
    // candidates
    std::vector<KeyFrame *> connectedKeyFrames{};
    if (p_currentKF->getVectorCovisibleKeyFrames(connectedKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    // Extract candidates from the bag of words
    std::vector<KeyFrame *> mergeBowCandidates, loopBowCandidates;
    if (!isMergeDetectedInKeyFrame || !isLoopDetectedInKeyFrame)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeStartQuery =
            std::chrono::steady_clock::now();
#endif
        if (p_keyFrameDatabase->detectNBestCandidates(p_currentKF,
                                                      loopBowCandidates,
                                                      mergeBowCandidates,
                                                      3) !=
            KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: detectNBestCandidates returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeEndQuery =
            std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   timeEndQuery - timeStartQuery)
                                   .count();
        dataQueryTimes_ms.push_back(timeDataQuery);
#endif
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_2 =
        std::chrono::steady_clock::now();
#endif
    // Check the BoW candidates if the geometric candidate list is empty
    // Loop candidates
    if (!isLoopDetectedInKeyFrame && !loopBowCandidates.empty())
    {
        bool isDetected{};
        if (detectCommonRegionsFromBoW(loopBowCandidates,
                                       p_loopMatchedKF,
                                       p_loopLastCurrentKF,
                                       mg2oLoopSlw,
                                       loopNumCoincidences,
                                       loopMPs,
                                       loopMatchedMPs,
                                       isDetected) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detectCommonRegionsFromBoW returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        isLoopDetected = isDetected;
    }
    // Merge candidates
    if (!isMergeDetectedInKeyFrame && !mergeBowCandidates.empty())
    {
        bool isDetected2{};
        if (detectCommonRegionsFromBoW(mergeBowCandidates,
                                       p_mergeMatchedKF,
                                       p_mergeLastCurrentKF,
                                       mg2oMergeSlw,
                                       mergeNumCoincidences,
                                       mergeMPs,
                                       mergeMatchedMPs,
                                       isDetected2) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detectCommonRegionsFromBoW returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        isMergeDetected = isDetected2;
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndEstSim3_2 =
        std::chrono::steady_clock::now();

    timeEstSim3 +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndEstSim3_2 - time_StartEstSim3_2)
            .count();
    sim3EstimationTimes_ms.push_back(timeEstSim3);
#endif

    if (p_keyFrameDatabase->add(p_currentKF) !=
        KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: add returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (isMergeDetected || isLoopDetected)
    {
        isDetected_out = true;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    if (p_currentKF->setErase() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setErase returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_currentKF->isInCurrentPlaceRecognition = false;

    isDetected_out = false;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
