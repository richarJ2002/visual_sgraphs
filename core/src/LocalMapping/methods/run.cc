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

#include "LocalMapping.h"

#include "Optimizer.h"

#include <chrono>

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
        setAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if (checkNewKeyFrames() && !isImuBad)
        {
#ifdef REGISTER_TIMES
            double timeLocalBa_ms         = 0;
            double timeKeyFrameCulling_ms = 0;

            std::chrono::steady_clock::time_point processKeyFrameStartTime =
                std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            processNewKeyFrame();
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
            mapPointCulling();
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
            createNewMapPoints();

            shouldAbortBa = false;

            if (!checkNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point
                // duplications
                searchInNeighbors();
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

            if (!checkNewKeyFrames() && !stopRequested())
            {
                if (p_atlas->getKeyFrameCount() > 2)
                {
                    if (isInertial &&
                        p_currentKeyFrame->getMap()->isImuInitialized())
                    {
                        float cameraCenterDistance =
                            (p_currentKeyFrame->p_prevKF->getCameraCenter() -
                             p_currentKeyFrame->getCameraCenter())
                                .norm() +
                            (p_currentKeyFrame->p_prevKF->p_prevKF
                                 ->getCameraCenter() -
                             p_currentKeyFrame->p_prevKF->getCameraCenter())
                                .norm();

                        if (cameraCenterDistance > 0.05)
                            initializationStartTime +=
                                p_currentKeyFrame->timeStamp -
                                p_currentKeyFrame->p_prevKF->timeStamp;
                        /* A stationary interval is valid after initialization
                         * and must not invalidate the active map.
                         * Initialization itself is gated by cumulative
                         * translation below. */

                        bool isLargeBundleAdjustment =
                            ((p_tracker->getMatchesInliers() > 75) &&
                             isMonocular) ||
                            ((p_tracker->getMatchesInliers() > 100) &&
                             !isMonocular);
                        Optimizer::localInertialBA(
                            p_currentKeyFrame,
                            &shouldAbortBa,
                            p_currentKeyFrame->getMap(),
                            baFixedKeyFrameCount,
                            baOptimizedKeyFrameCount,
                            baMapPointCount,
                            baEdgeCount,
                            isLargeBundleAdjustment,
                            !p_currentKeyFrame->getMap()->getInertialBA2());
                        wasLocalBaExecuted = true;
                    }
                    else
                    {
                        types::SystemParams *p_params = nullptr;
                        if (types::SystemParams::getParams(p_params) !=
                            types::SystemParamsStatus::
                                SYSTEM_PARAMS_STATUS_SUCCESS)
                        {
                            // getParams cannot fail; continue as before.
                        }
                        Optimizer::localBundleAdjustment(
                            p_currentKeyFrame,
                            &shouldAbortBa,
                            p_currentKeyFrame->getMap(),
                            baFixedKeyFrameCount,
                            baOptimizedKeyFrameCount,
                            baMapPointCount,
                            baEdgeCount,
                            p_params->markers.impact);
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
                if (!p_currentKeyFrame->getMap()->isImuInitialized() &&
                    isInertial)
                {
                    if (isMonocular)
                        initializeIMU(1e2, 1e10, true);
                    else
                        initializeIMU(1e2, 1e5, true);
                }

                // Check redundant local Keyframes
                keyFrameCulling();

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
                    if (p_currentKeyFrame->getMap()->isImuInitialized() &&
                        p_tracker->state == Tracking::OK)
                    {
                        if (!p_currentKeyFrame->getMap()->getInertialBA1())
                        {
                            // First stage of IMU initialization (5 seconds
                            // after initialization)
                            if (initializationStartTime > 5.0f)
                            {
                                std::cout
                                    << "[Mapping] Starting IMU bias/scale "
                                       "initialization (stage#1) ..."
                                    << std::endl;
                                p_currentKeyFrame->getMap()->setInertialBA1();
                                if (isMonocular)
                                    initializeIMU(1.f, 1e5, true);
                                else
                                    initializeIMU(1.f, 1e5, true);
                                std::cout << "[Mapping] Ending IMU bias/scale "
                                             "initialization (stage#1) ..."
                                          << std::endl;
                            }
                        }
                        else if (!p_currentKeyFrame->getMap()->getInertialBA2())
                        {
                            // Second stage of IMU initialization (15 seconds
                            // after initialization)
                            if (initializationStartTime > 15.0f)
                            {
                                std::cout
                                    << "[Mapping] Starting IMU bias/scale "
                                       "initialization (stage#2) ..."
                                    << std::endl;
                                p_currentKeyFrame->getMap()->setInertialBA2();
                                if (isMonocular)
                                    initializeIMU(0.f, 0.f, true);
                                else
                                    initializeIMU(0.f, 0.f, true);
                                std::cout << "[Mapping] Ending IMU bias/scale "
                                             "initialization (stage#2) ..."
                                          << std::endl;
                            }
                        }

                        // Scale refinement
                        if (((p_atlas->getKeyFrameCount()) <= 200) &&
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
                                scaleRefinement();
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            localBaSyncTimes_ms.push_back(timeKeyFrameCulling_ms);
            keyFrameCullingSyncTimes_ms.push_back(timeKeyFrameCulling_ms);
#endif

            p_loopCloser->insertKeyFrame(p_currentKeyFrame);

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
        else if (stop() && !isImuBad)
        {
            // Safe area to stop
            while (isStopped() && !checkFinish())
            {
                usleep(3000);
            }
            if (checkFinish())
                break;
        }

        resetIfRequested();

        // Tracking will see that Local Mapping is busy
        setAcceptKeyFrames(true);

        if (checkFinish())
            break;

        usleep(3000);
    }

    setFinish();
}

} // namespace core
} // namespace vs_graphs
