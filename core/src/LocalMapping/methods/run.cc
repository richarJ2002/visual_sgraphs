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
 * @file            run.cc
 *
 * @brief           Implements LocalMapping::run(), declared in LocalMapping.h.
 */

#include "LocalMapping.h"

#include "Optimizer.h"
#include "Tracking.h"

#include <chrono>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void LocalMapping::run()
{
    hasFinished = false;

    while (1)
    {
        // Tracking will see that Local Mapping is busy
        if (setAcceptKeyFrames(false) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setAcceptKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        // Check if there are keyframes in the queue
        bool hasNewKeyFrames{};
        if (checkNewKeyFrames(hasNewKeyFrames) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkNewKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (hasNewKeyFrames && !isImuBad)
        {
#ifdef REGISTER_TIMES
            double timeLocalBa_ms         = 0;
            double timeKeyFrameCulling_ms = 0;

            std::chrono::steady_clock::time_point processKeyFrameStartTime =
                std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            if (processNewKeyFrame() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: processNewKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point processKeyFrameEndTime =
                std::chrono::steady_clock::now();

            double timeProcessKeyFrame =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    processKeyFrameEndTime - processKeyFrameStartTime)
                    .count();
            keyFrameInsertTimes_ms.push_back(timeProcessKeyFrame);
#endif

            // Check recent MapPoints
            if (mapPointCulling() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: mapPointCulling returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point mapPointCullingEndTime =
                std::chrono::steady_clock::now();

            double timeMapPointCulling =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    mapPointCullingEndTime - processKeyFrameEndTime)
                    .count();
            mapPointCullingTimes_ms.push_back(timeMapPointCulling);
#endif

            // Triangulate new MapPoints
            if (createNewMapPoints() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createNewMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            shouldAbortBa = false;

            bool hasNewKeyFrames2{};
            if (checkNewKeyFrames(hasNewKeyFrames2) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: checkNewKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!hasNewKeyFrames2)
            {
                // Find more matches in neighbor keyframes and fuse point
                // duplications
                if (searchInNeighbors() !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: searchInNeighbors returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point mapPointCreationEndTime =
                std::chrono::steady_clock::now();

            double timeMapPointCreation =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    mapPointCreationEndTime - mapPointCullingEndTime)
                    .count();
            mapPointCreationTimes_ms.push_back(timeMapPointCreation);
#endif

            // Only consumed by the REGISTER_TIMES statistics block below.
            [[maybe_unused]] bool wasLocalBaExecuted = false;

            int baFixedKeyFrameCount     = 0;
            int baOptimizedKeyFrameCount = 0;
            int baMapPointCount          = 0;
            int baEdgeCount              = 0;

            bool hasNewKeyFrames3{};
            if (checkNewKeyFrames(hasNewKeyFrames3) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: checkNewKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool shouldStop{};
            if ((!hasNewKeyFrames3) &&
                stopRequested(shouldStop) !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: stopRequested returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!hasNewKeyFrames3 && !shouldStop)
            {
                unsigned long atlasKeyFrameCount{};
                if (p_atlas->getKeyFrameCount(atlasKeyFrameCount) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKeyFrameCount returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (atlasKeyFrameCount > 2)
                {
                    Map *p_currentKeyFrameMap = nullptr;
                    if ((isInertial) &&
                        p_currentKeyFrame->getMap(p_currentKeyFrameMap) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isImuInitialized2{};
                    if ((isInertial) &&
                        p_currentKeyFrameMap->isImuInitialized(
                            isImuInitialized2) != MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isImuInitialized returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (isInertial && isImuInitialized2)
                    {
                        Eigen::Vector3f cameraCenter{};
                        if (p_currentKeyFrame->p_prevKF->getCameraCenter(
                                cameraCenter) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getCameraCenter returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f currentKeyFrameCameraCenter{};
                        if (p_currentKeyFrame->getCameraCenter(
                                currentKeyFrameCameraCenter) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getCameraCenter returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f cameraCenter2{};
                        if (p_currentKeyFrame->p_prevKF->p_prevKF
                                ->getCameraCenter(cameraCenter2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getCameraCenter returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f cameraCenter3{};
                        if (p_currentKeyFrame->p_prevKF->getCameraCenter(
                                cameraCenter3) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getCameraCenter returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        float cameraCenterDistance =
                            (cameraCenter - currentKeyFrameCameraCenter)
                                .norm() +
                            (cameraCenter2 - cameraCenter3).norm();

                        if (cameraCenterDistance > 0.05)
                        {
                            initializationStartTime +=
                                p_currentKeyFrame->timeStamp -
                                p_currentKeyFrame->p_prevKF->timeStamp;
                        }
                        /* A stationary interval is valid after initialization
                         * and must not invalidate the active map.
                         * Initialization itself is gated by cumulative
                         * translation below. */

                        int trackerMatchesInliers{};
                        if (p_tracker->getMatchesInliers(
                                trackerMatchesInliers) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getMatchesInliers returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        int trackerMatchesInliers2{};
                        if (!(((trackerMatchesInliers > 75) && isMonocular)) &&
                            p_tracker->getMatchesInliers(
                                trackerMatchesInliers2) !=
                                TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getMatchesInliers returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        bool isLargeBundleAdjustment =
                            ((trackerMatchesInliers > 75) && isMonocular) ||
                            ((trackerMatchesInliers2 > 100) && !isMonocular);
                        Map *p_currentKeyFrameMap2 = nullptr;
                        if (p_currentKeyFrame->getMap(p_currentKeyFrameMap2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        Map *p_currentKeyFrameMap3 = nullptr;
                        if (p_currentKeyFrame->getMap(p_currentKeyFrameMap3) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        bool inertialBA2{};
                        if (p_currentKeyFrameMap3->getInertialBA2(
                                inertialBA2) != MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getInertialBA2 returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (Optimizer::localInertialBA(p_currentKeyFrame,
                                                       &shouldAbortBa,
                                                       p_currentKeyFrameMap2,
                                                       baFixedKeyFrameCount,
                                                       baOptimizedKeyFrameCount,
                                                       baMapPointCount,
                                                       baEdgeCount,
                                                       isLargeBundleAdjustment,
                                                       !inertialBA2) !=
                            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: localInertialBA returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        wasLocalBaExecuted = true;
                    }
                    else
                    {
                        Map *p_currentKeyFrameMap4 = nullptr;
                        if (p_currentKeyFrame->getMap(p_currentKeyFrameMap4) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (Optimizer::localBundleAdjustment(
                                p_currentKeyFrame,
                                &shouldAbortBa,
                                p_currentKeyFrameMap4,
                                baFixedKeyFrameCount,
                                baOptimizedKeyFrameCount,
                                baMapPointCount,
                                baEdgeCount) !=
                            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: localBundleAdjustment returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        wasLocalBaExecuted = true;
                    }
                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point localBaEndTime =
                    std::chrono::steady_clock::now();

                if (wasLocalBaExecuted)
                {
                    timeLocalBa_ms =
                        std::chrono::duration_cast<
                            std::chrono::duration<double, std::milli>>(
                            localBaEndTime - mapPointCreationEndTime)
                            .count();
                    localBaTimes_ms.push_back(timeLocalBa_ms);

                    localBaExecutionCount += 1;
                    if (shouldAbortBa)
                    {
                        localBaAbortCount += 1;
                    }
                    localBaEdgeCounts.push_back(baEdgeCount);
                    localBaOptimizedKeyFrameCounts.push_back(
                        baOptimizedKeyFrameCount);
                    localBaFixedKeyFrameCounts.push_back(baFixedKeyFrameCount);
                    localBaMapPointCounts.push_back(baMapPointCount);
                }

#endif

                // IMU initialization
                Map *p_currentKeyFrameMap5 = nullptr;
                if (p_currentKeyFrame->getMap(p_currentKeyFrameMap5) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool isImuInitialized3{};
                if (p_currentKeyFrameMap5->isImuInitialized(
                        isImuInitialized3) != MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isImuInitialized returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (!isImuInitialized3 && isInertial)
                {
                    if (isMonocular)
                    {
                        if (initializeIMU(1e2, 1e10, true) !=
                            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: initializeIMU returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                    else
                    {
                        if (initializeIMU(1e2, 1e5, true) !=
                            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: initializeIMU returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                }

                // Check redundant local Keyframes
                if (keyFrameCulling() !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: keyFrameCulling returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point keyFrameCullingEndTime =
                    std::chrono::steady_clock::now();

                timeKeyFrameCulling_ms =
                    std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>>(
                        keyFrameCullingEndTime - localBaEndTime)
                        .count();
                keyFrameCullingTimes_ms.push_back(timeKeyFrameCulling_ms);
#endif

                // Staged IMU initialization
                // [Hint] For Visual-Inertial SLAM, the IMU biases and scale are
                // initially unknown (the first 5secs). After 15 seconds, they
                // are locked to avoid drift.
                if ((initializationStartTime < 50.0f) && isInertial)
                {
                    // Enter here everytime local-mapping is called
                    Map *p_currentKeyFrameMap6 = nullptr;
                    if (p_currentKeyFrame->getMap(p_currentKeyFrameMap6) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isImuInitialized4{};
                    if (p_currentKeyFrameMap6->isImuInitialized(
                            isImuInitialized4) != MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isImuInitialized returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (isImuInitialized4 && p_tracker->state == Tracking::OK)
                    {
                        Map *p_currentKeyFrameMap7 = nullptr;
                        if (p_currentKeyFrame->getMap(p_currentKeyFrameMap7) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        bool inertialBA1{};
                        if (p_currentKeyFrameMap7->getInertialBA1(
                                inertialBA1) != MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getInertialBA1 returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (!inertialBA1)
                        {
                            // First stage of IMU initialization (5 seconds
                            // after initialization)
                            if (initializationStartTime > 5.0f)
                            {
                                std::cout
                                    << "[Mapping] Starting IMU bias/scale "
                                       "initialization (stage#1) ..."
                                    << std::endl;
                                Map *p_currentKeyFrameMap8 = nullptr;
                                if (p_currentKeyFrame->getMap(
                                        p_currentKeyFrameMap8) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: getMap returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                if (p_currentKeyFrameMap8->setInertialBA1() !=
                                    MapStatus::MAP_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: setInertialBA1 returned a failure "
                                        "status although it cannot fail; "
                                        "continuing as before.",
                                        __func__);
                                }
                                if (isMonocular)
                                {
                                    if (initializeIMU(1.f, 1e5, true) !=
                                        LocalMappingStatus::
                                            LOCAL_MAPPING_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: initializeIMU returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                }
                                else
                                {
                                    if (initializeIMU(1.f, 1e5, true) !=
                                        LocalMappingStatus::
                                            LOCAL_MAPPING_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: initializeIMU returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                }
                                std::cout << "[Mapping] Ending IMU bias/scale "
                                             "initialization (stage#1) ..."
                                          << std::endl;
                            }
                        }
                        else
                        {
                            Map *p_currentKeyFrameMap9 = nullptr;
                            if (p_currentKeyFrame->getMap(
                                    p_currentKeyFrameMap9) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getMap returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            bool inertialBA22{};
                            if (p_currentKeyFrameMap9->getInertialBA2(
                                    inertialBA22) !=
                                MapStatus::MAP_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getInertialBA2 returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if (!inertialBA22)
                            {
                                // Second stage of IMU initialization (15
                                // seconds after initialization)
                                if (initializationStartTime > 15.0f)
                                {
                                    std::cout
                                        << "[Mapping] Starting IMU bias/scale "
                                           "initialization (stage#2) ..."
                                        << std::endl;
                                    Map *p_currentKeyFrameMap10 = nullptr;
                                    if (p_currentKeyFrame->getMap(
                                            p_currentKeyFrameMap10) !=
                                        KeyFrameStatus::
                                            KEY_FRAME_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: getMap returned a failure "
                                            "status although it cannot fail; "
                                            "continuing as before.",
                                            __func__);
                                    }
                                    if (p_currentKeyFrameMap10
                                            ->setInertialBA2() !=
                                        MapStatus::MAP_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: setInertialBA2 returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                    if (isMonocular)
                                    {
                                        if (initializeIMU(0.f, 0.f, true) !=
                                            LocalMappingStatus::
                                                LOCAL_MAPPING_STATUS_SUCCESS)
                                        {
                                            RCLCPP_ERROR(
                                                rclcpp::get_logger("vs_graphs"),
                                                "%s: initializeIMU returned a "
                                                "failure status although it "
                                                "cannot fail; continuing as "
                                                "before.",
                                                __func__);
                                        }
                                    }
                                    else
                                    {
                                        if (initializeIMU(0.f, 0.f, true) !=
                                            LocalMappingStatus::
                                                LOCAL_MAPPING_STATUS_SUCCESS)
                                        {
                                            RCLCPP_ERROR(
                                                rclcpp::get_logger("vs_graphs"),
                                                "%s: initializeIMU returned a "
                                                "failure status although it "
                                                "cannot fail; continuing as "
                                                "before.",
                                                __func__);
                                        }
                                    }
                                    std::cout
                                        << "[Mapping] Ending IMU bias/scale "
                                           "initialization (stage#2) ..."
                                        << std::endl;
                                }
                            }
                        }

                        // Scale refinement
                        unsigned long atlasKeyFrameCount2{};
                        if (p_atlas->getKeyFrameCount(atlasKeyFrameCount2) !=
                            AtlasStatus::ATLAS_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getKeyFrameCount returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (((atlasKeyFrameCount2) <= 200) &&
                            ((initializationStartTime > 25.0f &&
                              initializationStartTime < 25.5f) ||
                             (initializationStartTime > 35.0f &&
                              initializationStartTime < 35.5f) ||
                             (initializationStartTime > 45.0f &&
                              initializationStartTime < 45.5f) ||
                             (initializationStartTime > 55.0f &&
                              initializationStartTime < 55.5f) ||
                             (initializationStartTime > 65.0f &&
                              initializationStartTime < 65.5f) ||
                             (initializationStartTime > 75.0f &&
                              initializationStartTime < 75.5f)))
                        {
                            if (isMonocular)
                            {
                                if (scaleRefinement() !=
                                    LocalMappingStatus::
                                        LOCAL_MAPPING_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: scaleRefinement returned a "
                                        "failure status although it cannot "
                                        "fail; continuing as before.",
                                        __func__);
                                }
                            }
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            localBaSyncTimes_ms.push_back(timeKeyFrameCulling_ms);
            keyFrameCullingSyncTimes_ms.push_back(timeKeyFrameCulling_ms);
#endif

            if (p_loopCloser->insertKeyFrame(p_currentKeyFrame) !=
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: insertKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point localMapEndTime =
                std::chrono::steady_clock::now();

            double timeLocalMap =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    localMapEndTime - processKeyFrameStartTime)
                    .count();
            localMappingTotalTimes_ms.push_back(timeLocalMap);
#endif
        }
        else
        {
            bool isStopped2{};
            if (stop(isStopped2) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: stop returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (isStopped2 && !isImuBad)
            {
                // Safe area to stop
                for (;;)
                {
                    bool isStopped3{};
                    if (isStopped(isStopped3) !=
                        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isStopped returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    bool shouldFinish{};
                    if ((isStopped3) &&
                        checkFinish(shouldFinish) !=
                            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: checkFinish returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!(isStopped3 && !shouldFinish))
                    {
                        break;
                    }
                    usleep(3000);
                }
                bool isFinishRequested2{};
                if (checkFinish(isFinishRequested2) !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: checkFinish returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (isFinishRequested2)
                {
                    break;
                }
            }
        }

        if (resetIfRequested() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resetIfRequested returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        // Tracking will see that Local Mapping is busy
        if (setAcceptKeyFrames(true) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setAcceptKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool isFinishRequested3{};
        if (checkFinish(isFinishRequested3) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (isFinishRequested3)
            break;

        usleep(3000);
    }

    if (setFinish() != LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
