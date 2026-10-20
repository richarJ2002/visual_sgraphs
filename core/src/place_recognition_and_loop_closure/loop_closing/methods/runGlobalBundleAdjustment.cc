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
 * @file            runGlobalBundleAdjustment.cc
 *
 * @brief           Implements LoopClosing::runGlobalBundleAdjustment(),
 *                  declared in loop_closing/objects/LoopClosingClass.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"

#include "execution/system/system.h"
#include "mapping/geometry_helpers/utils.h"
#include "mapping/geometry_helpers/utils.h"
#include "mapping/local_mapping/local_mapping.h"
#include "optimization/optimizer/optimizer.h"

#include <chrono>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{
namespace loop_closing
{

LoopClosingStatus
    LoopClosing::runGlobalBundleAdjustment(map::Map     *p_activeMap_inout,
                                           unsigned long loopKeyFrameCount_in,
                                           unsigned int  generation_in)
{
    if (system::Verbose::printMess("Starting Global Bundle Adjustment",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
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

    std::vector<keyframe::KeyFrame *> activeMapKeyFrames;
    if (p_activeMap_inout->getAllKeyFrames(activeMapKeyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getAllKeyFrames returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    gbaKeyFrameCounts.push_back(activeMapKeyFrames.size());
    std::vector<map_point::MapPoint *> activeMapMapPoints;
    if (p_activeMap_inout->getAllMapPoints(activeMapMapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
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
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (!isImuInitialized)
    {
        if (optimizer::Optimizer::globalBundleAdjustment(
                p_activeMap_inout,
                10,
                &optimizerStopRequested,
                loopKeyFrameCount_in,
                false,
                &isGlobalBundleAdjustmentStopRequested) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: globalBundleAdjustment returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        if (optimizer::Optimizer::fullInertialBA(p_activeMap_inout,
                                      7,
                                      false,
                                      loopKeyFrameCount_in,
                                      &optimizerStopRequested,
                                      false,
                                      1e2F,
                                      1e6F,
                                      &isGlobalBundleAdjustmentStopRequested) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
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

    /*!
     * Update all MapPoints and KeyFrames
     * Local Mapping was active during BA, that means that there might be new
     * keyframes not included in the Global BA and they are not consistent with
     * the updated map. We need to propagate the correction through the spanning
     * tree
     */
    {
        std::unique_lock<std::mutex> lock(gbaMutex);
        if (generation_in != fullBundleAdjustmentIndex)
        {
            hasGbaFinished = true;
            isGbaRunning   = false;
            return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
        }

        bool activeMapIsImuInitialized{};
        if ((!isImuInitialized) &&
            p_activeMap_inout->isImuInitialized(activeMapIsImuInitialized) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
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
            if (system::Verbose::printMess("Global Bundle Adjustment finished",
                                   system::Verbose::VERBOSITY_NORMAL) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (system::Verbose::printMess("Updating map ...",
                                   system::Verbose::VERBOSITY_NORMAL) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_localMapper->requestStop() !=
                local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: requestStop returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            /* Wait until Local Mapping has effectively stopped */

            for (;;)
            {
                bool localMapperIsStopped{};
                if (p_localMapper->isStopped(localMapperIsStopped) !=
                    local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
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
                        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
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
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: acquireSemanticUpdateLock returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /* Get map::Map Mutex */
            std::unique_lock<std::mutex> mapUpdateLock(
                p_activeMap_inout->mapUpdateMutex);

            KeyFrameAndPose keyFramePosesBefore_worldToCamera;
            KeyFrameAndPose keyFramePosesAfter_worldToCamera;

            /*  Correct keyframes starting at map first keyframe */
            std::list<keyframe::KeyFrame *> keyFramesToCheck(
                p_activeMap_inout->keyFrameOrigins.begin(),
                p_activeMap_inout->keyFrameOrigins.end());

            while (!keyFramesToCheck.empty())
            {
                keyframe::KeyFrame *p_keyFrame = keyFramesToCheck.front();
                std::set<keyframe::KeyFrame *> childs{};
                if (p_keyFrame->getChilds(childs) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getChilds returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SE3f cameraPose_cameraToWorld{};
                if (p_keyFrame->getPoseInverse(cameraPose_cameraToWorld) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPoseInverse returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                for (std::set<keyframe::KeyFrame *>::const_iterator sit =
                         childs.begin();
                     sit != childs.end();
                     sit++)
                {
                    keyframe::KeyFrame *p_child = *sit;
                    bool                childIsBad{};
                    if (!(!p_child) &&
                        p_child->isBad(childIsBad) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3f tchildc =
                            childPose * cameraPose_cameraToWorld;
                        p_child->tcwGBA = tchildc * p_keyFrame->tcwGBA;

                        Sophus::SE3f childPose2{};
                        if (p_child->getPose(childPose2) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                            if (system::Verbose::printMess("Child velocity empty!! ",
                                                   system::Verbose::VERBOSITY_NORMAL) !=
                                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: printMess returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                        }

                        imu_types::bias::Bias childImuBias{};
                        if (p_child->getImuBias(childImuBias) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_keyFrame->tcwBefGBA = keyFramePose;

                const Sophus::SE3d poseBefore_worldToCamera =
                    p_keyFrame->tcwBefGBA.cast<double>();

                keyFramePosesBefore_worldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseBefore_worldToCamera.unit_quaternion(),
                              poseBefore_worldToCamera.translation(),
                              1.0));

                if (p_keyFrame->setPose(p_keyFrame->tcwGBA) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                Sophus::SE3f keyFramePose2{};
                if (p_keyFrame->getPose(keyFramePose2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const Sophus::SE3d poseAfter_worldToCamera =
                    keyFramePose2.cast<double>();

                keyFramePosesAfter_worldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                              poseAfter_worldToCamera.translation(),
                              1.0));

                if (p_keyFrame->isImu)
                {
                    Eigen::Vector3f keyFrameVelocity{};
                    if (p_keyFrame->getVelocity(keyFrameVelocity) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    p_keyFrame->vwbBefGBA = keyFrameVelocity;

                    if (p_keyFrame->setVelocity(p_keyFrame->vwbGBA) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame->setNewBias(p_keyFrame->biasGBA) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

            /* Correct MapPoints */
            std::vector<map_point::MapPoint *> mapPoints{};
            if (p_activeMap_inout->getAllMapPoints(mapPoints) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
                 mapPointIndex++)
            {
                map_point::MapPoint *p_mapPoint = mapPoints[mapPointIndex];

                bool mapPointIsBad{};
                if (!(p_mapPoint == nullptr) &&
                    p_mapPoint->isBad(mapPointIsBad) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                    /* If optimized by Global BA, just update */
                    if (p_mapPoint->setWorldPos(p_mapPoint->posGBA) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                    /*!
                     * Update according to the correction of its reference
                     * keyframe
                     */
                    keyframe::KeyFrame *p_referenceKeyFrame = nullptr;
                    if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    map::Map *p_referenceKeyFrameMap = nullptr;
                    if (!(p_referenceKeyFrame == nullptr ||
                          referenceKeyFrameIsBad) &&
                        p_referenceKeyFrame->getMap(p_referenceKeyFrameMap) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

                        std::map<keyframe::KeyFrame *, std::tuple<int, int>>
                            observations{};
                        if (p_mapPoint->getObservations(observations) !=
                            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: isBad returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            map::Map *p_observingKeyFrameMap = nullptr;
                            if (!(p_observingKeyFrame == nullptr ||
                                  observingKeyFrameIsBad) &&
                                p_observingKeyFrame->getMap(
                                    p_observingKeyFrameMap) !=
                                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

                    /* map::Map to non-corrected camera */
                    Eigen::Vector3f mapPointWorldPos{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector3f Xc =
                        p_referenceKeyFrame->tcwBefGBA * mapPointWorldPos;

                    /* Backproject using corrected camera */
                    Sophus::SE3f referenceKeyFramePoseInverse{};
                    if (p_referenceKeyFrame->getPoseInverse(
                            referenceKeyFramePoseInverse) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPoseInverse returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_mapPoint->setWorldPos(referenceKeyFramePoseInverse *
                                                Xc) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                }
            }

            const g2o::Sim3 identityTransform_worldToWorld(
                Eigen::Quaterniond::Identity(),
                Eigen::Vector3d::Zero(),
                1.0);

            /* Keep every semantic entity aligned with the corrected cameras. */
            if (utils::utils::Utils::propagateSemanticPoseCorrections(
                    p_activeMap_inout,
                    keyFramePosesBefore_worldToCamera,
                    keyFramePosesAfter_worldToCamera,
                    identityTransform_worldToWorld) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                /*!
                 * propagateSemanticPoseCorrections cannot fail; continue as
                 * before.
                 */
            }

            /* Preserve plane variables which were optimized directly by GBA. */
            std::vector<geometric::Plane *> activeMapAllPlanes{};
            if (p_activeMap_inout->getAllPlanes(activeMapAllPlanes) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
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
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: informNewBigChange returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_activeMap_inout->increaseChangeIndex() !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: increaseChangeIndex returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /* TODO Check whether tracking::Tracking::updateFrameIMU belongs
             * here */
            if (p_localMapper->release() !=
                local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
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
            if (system::Verbose::printMess("map::Map updated!",
                                   system::Verbose::VERBOSITY_NORMAL) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
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

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */
