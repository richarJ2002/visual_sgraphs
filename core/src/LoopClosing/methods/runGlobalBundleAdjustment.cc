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

#include "LocalMapping.h"
#include "Optimizer.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <chrono>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus
    LoopClosing::runGlobalBundleAdjustment(Map          *p_activeMap_inout,
                                           unsigned long loopKeyFrameCount_in,
                                           unsigned int  generation_in)
{
    if (Verbose::printMess("Starting Global Bundle Adjustment",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeStartFGba =
        std::chrono::steady_clock::now();

    fullGbaExecutionCount += 1;

    std::vector<KeyFrame *> activeMapKeyFrames;
    if (p_activeMap_inout->getAllKeyFrames(activeMapKeyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getAllKeyFrames returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    gbaKeyFrameCounts.push_back(activeMapKeyFrames.size());
    std::vector<MapPoint *> activeMapMapPoints;
    if (p_activeMap_inout->getAllMapPoints(activeMapMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getAllMapPoints returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    gbaMapPointCounts.push_back(activeMapMapPoints.size());
#endif

    /*
     * g2o accepts a plain bool force-stop token. A first-party iteration action
     * copies the atomic cross-thread request into this worker-local flag, so
     * cancellation remains race-free and takes effect between iterations.
     */
    bool optimizerStopRequested = false;

    bool isImuInitialized{};
    if (p_activeMap_inout->isImuInitialized(isImuInitialized) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (!isImuInitialized)
    {
        double trackerGetMarkerImpact{};
        if (p_tracker->getMarkerImpact(trackerGetMarkerImpact) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMarkerImpact returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (Optimizer::globalBundleAdjustment(
                p_activeMap_inout,
                10,
                &optimizerStopRequested,
                loopKeyFrameCount_in,
                false,
                trackerGetMarkerImpact,
                &isGlobalBundleAdjustmentStopRequested) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: globalBundleAdjustment returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        if (Optimizer::fullInertialBA(p_activeMap_inout,
                                      7,
                                      false,
                                      loopKeyFrameCount_in,
                                      &optimizerStopRequested,
                                      false,
                                      1e2F,
                                      1e6F,
                                      nullptr,
                                      nullptr,
                                      &isGlobalBundleAdjustmentStopRequested) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fullInertialBA returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndGba =
        std::chrono::steady_clock::now();

    double timeGba =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndGba - timeStartFGba)
            .count();
    gbaTimes_ms.push_back(timeGba);

    if (optimizerStopRequested)
    {
        fullGbaAbortCount += 1;
    }
#endif

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new
    // keyframes not included in the Global BA and they are not consistent with
    // the updated map. We need to propagate the correction through the spanning
    // tree
    {
        unique_lock<mutex> lock(gbaMutex);
        if (generation_in != fullBundleAdjustmentIndex)
        {
            hasGbaFinished = true;
            isGbaRunning   = false;
            return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
        }

        bool activeMapIsImuInitialized{};
        if ((!isImuInitialized) &&
            p_activeMap_inout->isImuInitialized(activeMapIsImuInitialized) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!isImuInitialized && activeMapIsImuInitialized)
        {
            hasGbaFinished = true;
            isGbaRunning   = false;
            return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
        }

        if (!optimizerStopRequested)
        {
            if (Verbose::printMess("Global Bundle Adjustment finished",
                                   Verbose::VERBOSITY_NORMAL) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (Verbose::printMess("Updating map ...",
                                   Verbose::VERBOSITY_NORMAL) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_localMapper->requestStop() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: requestStop returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            // Wait until Local Mapping has effectively stopped

            for (;;)
            {
                bool localMapperIsStopped{};
                if (p_localMapper->isStopped(localMapperIsStopped) !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isStopped returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool localMapperIsFinished{};
                if ((!localMapperIsStopped) &&
                    p_localMapper->isFinished(localMapperIsFinished) !=
                        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isFinished returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!(!localMapperIsStopped && !localMapperIsFinished))
                {
                    break;
                }
                usleep(1000);
            }

            std::unique_lock<std::mutex> semanticUpdateLock{};
            if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: acquireSemanticUpdateLock returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(p_activeMap_inout->mapUpdateMutex);

            KeyFrameAndPose keyFramePosesBefore_WorldToCamera;
            KeyFrameAndPose keyFramePosesAfter_WorldToCamera;

            //  Correct keyframes starting at map first keyframe
            list<KeyFrame *> keyFramesToCheck(
                p_activeMap_inout->keyFrameOrigins.begin(),
                p_activeMap_inout->keyFrameOrigins.end());

            while (!keyFramesToCheck.empty())
            {
                KeyFrame            *p_keyFrame = keyFramesToCheck.front();
                std::set<KeyFrame *> childs{};
                if (p_keyFrame->getChilds(childs) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getChilds returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SE3f Twc{};
                if (p_keyFrame->getPoseInverse(Twc) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPoseInverse returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                for (set<KeyFrame *>::const_iterator sit = childs.begin();
                     sit != childs.end();
                     sit++)
                {
                    KeyFrame *p_child = *sit;
                    bool      childIsBad{};
                    if (!(!p_child) &&
                        p_child->isBad(childIsBad) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!p_child || childIsBad)
                        continue;

                    if (p_child->baGlobalKeyFrameId != loopKeyFrameCount_in)
                    {
                        Sophus::SE3f childPose{};
                        if (p_child->getPose(childPose) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3f tchildc = childPose * Twc;
                        p_child->tcwGBA =
                            tchildc * p_keyFrame->tcwGBA; //*Tcorc*pKF->mTcwGBA;

                        Sophus::SE3f childPose2{};
                        if (p_child->getPose(childPose2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Sophus::SO3f Rcor =
                            p_child->tcwGBA.so3().inverse() * childPose2.so3();
                        bool childIsVelocitySet{};
                        if (p_child->isVelocitySet(childIsVelocitySet) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isVelocitySet returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        if (childIsVelocitySet)
                        {
                            Eigen::Vector3f childVelocity{};
                            if (p_child->getVelocity(childVelocity) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getVelocity returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            p_child->vwbGBA = Rcor * childVelocity;
                        }
                        else
                        {
                            if (Verbose::printMess("Child velocity empty!! ",
                                                   Verbose::VERBOSITY_NORMAL) !=
                                VerboseStatus::VERBOSE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: printMess returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                        }

                        IMU::Bias childImuBias{};
                        if (p_child->getImuBias(childImuBias) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getImuBias returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        p_child->biasGBA = childImuBias;

                        p_child->baGlobalKeyFrameId = loopKeyFrameCount_in;
                    }
                    keyFramesToCheck.push_back(p_child);
                }

                Sophus::SE3f keyFramePose{};
                if (p_keyFrame->getPose(keyFramePose) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_keyFrame->tcwBefGBA = keyFramePose;

                const Sophus::SE3d poseBefore_WorldToCamera =
                    p_keyFrame->tcwBefGBA.cast<double>();

                keyFramePosesBefore_WorldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseBefore_WorldToCamera.unit_quaternion(),
                              poseBefore_WorldToCamera.translation(),
                              1.0));

                if (p_keyFrame->setPose(p_keyFrame->tcwGBA) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                Sophus::SE3f keyFramePose2{};
                if (p_keyFrame->getPose(keyFramePose2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const Sophus::SE3d poseAfter_WorldToCamera =
                    keyFramePose2.cast<double>();

                keyFramePosesAfter_WorldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                              poseAfter_WorldToCamera.translation(),
                              1.0));

                if (p_keyFrame->isImu)
                {
                    Eigen::Vector3f keyFrameVelocity{};
                    if (p_keyFrame->getVelocity(keyFrameVelocity) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    p_keyFrame->vwbBefGBA = keyFrameVelocity;

                    if (p_keyFrame->setVelocity(p_keyFrame->vwbGBA) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame->setNewBias(p_keyFrame->biasGBA) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setNewBias returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                }

                keyFramesToCheck.pop_front();
            }

            // Correct MapPoints
            std::vector<MapPoint *> mapPoints{};
            if (p_activeMap_inout->getAllMapPoints(mapPoints) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
                 mapPointIndex++)
            {
                MapPoint *p_mapPoint = mapPoints[mapPointIndex];

                bool mapPointIsBad{};
                if (!(p_mapPoint == nullptr) &&
                    p_mapPoint->isBad(mapPointIsBad) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_mapPoint == nullptr || mapPointIsBad)
                    continue;

                bool mapPointWasCorrected = false;

                if (p_mapPoint->baGlobalKeyFrameId == loopKeyFrameCount_in)
                {
                    // If optimized by Global BA, just update
                    if (p_mapPoint->setWorldPos(p_mapPoint->posGBA) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    mapPointWasCorrected = true;
                }
                else
                {
                    // Update according to the correction of its reference
                    // keyframe
                    KeyFrame *p_referenceKeyFrame = nullptr;
                    if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getReferenceKeyFrame returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    bool referenceKeyFrameIsBad{};
                    if (!(p_referenceKeyFrame == nullptr) &&
                        p_referenceKeyFrame->isBad(referenceKeyFrameIsBad) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    Map *p_referenceKeyFrameMap = nullptr;
                    if (!(p_referenceKeyFrame == nullptr ||
                          referenceKeyFrameIsBad) &&
                        p_referenceKeyFrame->getMap(p_referenceKeyFrameMap) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_referenceKeyFrame == nullptr ||
                        referenceKeyFrameIsBad ||
                        p_referenceKeyFrameMap != p_activeMap_inout ||
                        p_referenceKeyFrame->baGlobalKeyFrameId !=
                            loopKeyFrameCount_in)
                    {
                        p_referenceKeyFrame = nullptr;

                        std::map<KeyFrame *, std::tuple<int, int>>
                            observations{};
                        if (p_mapPoint->getObservations(observations) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getObservations returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }

                        for (const auto &[p_observingKeyFrame, featureIndexes] :
                             observations)
                        {
                            (void)featureIndexes;

                            bool observingKeyFrameIsBad{};
                            if (!(p_observingKeyFrame == nullptr) &&
                                p_observingKeyFrame->isBad(
                                    observingKeyFrameIsBad) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: isBad returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            Map *p_observingKeyFrameMap = nullptr;
                            if (!(p_observingKeyFrame == nullptr ||
                                  observingKeyFrameIsBad) &&
                                p_observingKeyFrame->getMap(
                                    p_observingKeyFrameMap) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getMap returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            if (p_observingKeyFrame == nullptr ||
                                observingKeyFrameIsBad ||
                                p_observingKeyFrameMap != p_activeMap_inout ||
                                p_observingKeyFrame->baGlobalKeyFrameId !=
                                    loopKeyFrameCount_in)
                            {
                                continue;
                            }

                            if (p_referenceKeyFrame == nullptr ||
                                p_observingKeyFrame->id <
                                    p_referenceKeyFrame->id)
                            {
                                p_referenceKeyFrame = p_observingKeyFrame;
                            }
                        }
                    }

                    if (p_referenceKeyFrame == nullptr)
                    {
                        continue;
                    }

                    /*if(pRefKF->mTcwBefGBA.empty())
                        continue;*/

                    // Map to non-corrected camera
                    // cv::Mat Rcw =
                    // pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3); cv::Mat
                    // tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    Eigen::Vector3f mapPointWorldPos{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector3f Xc =
                        p_referenceKeyFrame->tcwBefGBA * mapPointWorldPos;

                    // Backproject using corrected camera
                    Sophus::SE3f referenceKeyFramePoseInverse{};
                    if (p_referenceKeyFrame->getPoseInverse(
                            referenceKeyFramePoseInverse) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPoseInverse returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_mapPoint->setWorldPos(referenceKeyFramePoseInverse *
                                                Xc) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    mapPointWasCorrected = true;
                }

                if (mapPointWasCorrected)
                {
                    if (p_mapPoint->updateNormalAndDepth() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                }
            }

            const g2o::Sim3 identityTransform_WorldToWorld(
                Eigen::Quaterniond::Identity(),
                Eigen::Vector3d::Zero(),
                1.0);

            /* Keep every semantic entity aligned with the corrected cameras. */
            if (utils::utils::Utils::propagateSemanticPoseCorrections(
                    p_activeMap_inout,
                    keyFramePosesBefore_WorldToCamera,
                    keyFramePosesAfter_WorldToCamera,
                    identityTransform_WorldToWorld) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // propagateSemanticPoseCorrections cannot fail; continue as
                // before.
            }

            /* Preserve plane variables which were optimized directly by GBA. */
            std::vector<geometric::Plane *> activeMapAllPlanes{};
            if (p_activeMap_inout->getAllPlanes(activeMapAllPlanes) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPlanes returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_plane : activeMapAllPlanes)
            {
                bool planeIsBad{};
                if (!(p_plane == nullptr) &&
                    p_plane->isBad(planeIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_plane == nullptr || planeIsBad ||
                    p_plane->baGlobalKeyFrameId != loopKeyFrameCount_in)
                {
                    continue;
                }

                if (p_plane->alignGeometryToEquation(p_plane->planeGBA) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: alignGeometryToEquation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            if (p_activeMap_inout->informNewBigChange() !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: informNewBigChange returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_activeMap_inout->increaseChangeIndex() !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: increaseChangeIndex returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f,
            // mpTracker->GetLastKeyFrame()->getImuBias(),
            // mpTracker->GetLastKeyFrame());

            if (p_localMapper->release() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: release returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndUpdateMap =
                std::chrono::steady_clock::now();

            double timeUpdateMap =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndUpdateMap - timeEndGba)
                    .count();
            updateMapTimes_ms.push_back(timeUpdateMap);

            double timeFGba = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndUpdateMap - timeStartFGba)
                                  .count();
            fullGbaTotalTimes_ms.push_back(timeFGba);
#endif
            if (Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }

        hasGbaFinished = true;
        isGbaRunning   = false;
    }

    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
