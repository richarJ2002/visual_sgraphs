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
    finished = false;

    while (1)
    {
        // Tracking will see that Local Mapping is busy
        setAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if (checkNewKeyFrames() && !badImu)
        {
#ifdef REGISTER_TIMES
            double timeLBA_ms       = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF =
                std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            processNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF =
                std::chrono::steady_clock::now();

            double timeProcessKF =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndProcessKF - time_StartProcessKF)
                    .count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            mapPointCulling();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling =
                std::chrono::steady_clock::now();

            double timeMPCulling =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndMPCulling - time_EndProcessKF)
                    .count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif

            // Triangulate new MapPoints
            createNewMapPoints();

            abortBA = false;

            if (!checkNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point
                // duplications
                searchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation =
                std::chrono::steady_clock::now();

            double timeMPCreation =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndMPCreation - time_EndMPCulling)
                    .count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            // Only consumed by the REGISTER_TIMES statistics block below.
            [[maybe_unused]] bool b_doneLBA = false;

            int num_FixedKF_BA = 0;
            int num_OptKF_BA   = 0;
            int num_MPs_BA     = 0;
            int num_edges_BA   = 0;

            if (!checkNewKeyFrames() && !stopRequested())
            {
                if (p_atlas->getKeyFrameCount() > 2)
                {
                    if (inertial &&
                        p_currentKeyFrame->getMap()->isImuInitialized())
                    {
                        float dist =
                            (p_currentKeyFrame->p_prevKF->getCameraCenter() -
                             p_currentKeyFrame->getCameraCenter())
                                .norm() +
                            (p_currentKeyFrame->p_prevKF->p_prevKF
                                 ->getCameraCenter() -
                             p_currentKeyFrame->p_prevKF->getCameraCenter())
                                .norm();

                        if (dist > 0.05)
                            initializationStartTime +=
                                p_currentKeyFrame->timeStamp -
                                p_currentKeyFrame->p_prevKF->timeStamp;
                        /* A stationary interval is valid after initialization
                         * and must not invalidate the active map.
                         * Initialization itself is gated by cumulative
                         * translation below. */

                        bool bLarge = ((p_tracker->getMatchesInliers() > 75) &&
                                       monocular) ||
                                      ((p_tracker->getMatchesInliers() > 100) &&
                                       !monocular);
                        Optimizer::localInertialBA(
                            p_currentKeyFrame,
                            &abortBA,
                            p_currentKeyFrame->getMap(),
                            num_FixedKF_BA,
                            num_OptKF_BA,
                            num_MPs_BA,
                            num_edges_BA,
                            bLarge,
                            !p_currentKeyFrame->getMap()->getInertialBA2());
                        b_doneLBA = true;
                    }
                    else
                    {
                        Optimizer::localBundleAdjustment(
                            p_currentKeyFrame,
                            &abortBA,
                            p_currentKeyFrame->getMap(),
                            num_FixedKF_BA,
                            num_OptKF_BA,
                            num_MPs_BA,
                            num_edges_BA,
                            types::SystemParams::getParams()->markers.impact);
                        b_doneLBA = true;
                    }
                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA =
                    std::chrono::steady_clock::now();

                if (b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<
                                     std::chrono::duration<double, std::milli>>(
                                     time_EndLBA - time_EndMPCreation)
                                     .count();
                    vdLBA_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if (abortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // IMU initialization
                if (!p_currentKeyFrame->getMap()->isImuInitialized() &&
                    inertial)
                {
                    if (monocular)
                        initializeIMU(1e2, 1e10, true);
                    else
                        initializeIMU(1e2, 1e5, true);
                }

                // Check redundant local Keyframes
                keyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling =
                    std::chrono::steady_clock::now();

                timeKFCulling_ms =
                    std::chrono::duration_cast<
                        std::chrono::duration<double, std::milli>>(
                        time_EndKFCulling - time_EndLBA)
                        .count();
                vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                // Staged IMU initialization
                // [Hint] For Visual-Inertial SLAM, the IMU biases and scale are
                // initially unknown (the first 5secs). After 15 seconds, they
                // are locked to avoid drift.
                if ((initializationStartTime < 50.0f) && inertial)
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
                                if (monocular)
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
                                if (monocular)
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
                            if (monocular)
                                scaleRefinement();
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBASync_ms.push_back(timeKFCulling_ms);
            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

            p_loopCloser->insertKeyFrame(p_currentKeyFrame);

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap =
                std::chrono::steady_clock::now();

            double timeLocalMap =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    time_EndLocalMap - time_StartProcessKF)
                    .count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if (stop() && !badImu)
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
