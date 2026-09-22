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
 * @file         LocalMapping.cc
 *
 * @brief        Implements LocalMapping declared in LocalMapping.h.
 */

#include "LocalMapping.h"
#include "Utils/Converter/objects/Converter.h"
#include "GeometricTools.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <chrono>
#include <mutex>

namespace vs_graphs
{
namespace core
{

LocalMapping::LocalMapping(System                        *pSys,
                           Atlas                         *pAtlas,
                           const float                    bMonocular,
                           bool                           bInertial,
                           [[maybe_unused]] const string &_strSeqName) :
    scale(1.0),
    initSection(0),
    initIndex(0),
    iterationIndex(0),
    notBA1(true),
    notBA2(true),
    p_system(pSys),
    monocular(bMonocular),
    inertial(bInertial),
    resetRequested(false),
    resetActiveMapRequested(false),
    finishRequested(false),
    finished(true),
    p_atlas(pAtlas),
    abortBA(false),
    stopped(false),
    stopRequestedFlag(false),
    notStop(false),
    acceptKeyFrames(true),
    bInitializing(false),
    infoInertial(Eigen::MatrixXd::Zero(9, 9))
{
    localMappingCount       = 0;
    initializationStartTime = 0.f;
    badImu                  = false;
    keyFrameCullingCount    = 0;
    matchesInliers          = 0;

#ifdef REGISTER_TIMES
    nLBA_exec  = 0;
    nLBA_abort = 0;
#endif
}

void LocalMapping::setLoopCloser(LoopClosing *pLoopCloser)
{
    p_loopCloser = pLoopCloser;
}

void LocalMapping::setTracker(Tracking *pTracker)
{
    p_tracker = pTracker;
}

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

void LocalMapping::insertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    newKeyFrames.push_back(pKF);
    abortBA = true;
}

bool LocalMapping::checkNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return (!newKeyFrames.empty());
}

void LocalMapping::processNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        p_currentKeyFrame = newKeyFrames.front();
        newKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    p_currentKeyFrame->computeBagOfWords();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint *> vpMapPointMatches =
        p_currentKeyFrame->getMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++)
    {
        MapPoint *pMP = vpMapPointMatches[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                if (!pMP->isInKeyFrame(p_currentKeyFrame))
                {
                    pMP->addObservation(p_currentKeyFrame, i);
                    pMP->updateNormalAndDepth();
                    pMP->computeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by
                     // the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    p_currentKeyFrame->updateConnections();

    // Insert Keyframe in Map
    p_atlas->addKeyFrame(p_currentKeyFrame);
}

void LocalMapping::emptyQueue()
{
    while (checkNewKeyFrames())
        processNewKeyFrame();
}

void LocalMapping::mapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint *>::iterator lit          = mlpRecentAddedMapPoints.begin();
    const unsigned long int    nCurrentKFid = p_currentKeyFrame->mnId;

    int nThObs;
    if (monocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while (lit != mlpRecentAddedMapPoints.end())
    {
        MapPoint *pMP = *lit;

        if (pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if (pMP->getFoundRatio() < 0.25f)
        {
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if (((int)nCurrentKFid - (int)pMP->firstKeyFrameId) >= 2 &&
                 pMP->getObservationCount() <= cnThObs)
        {
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if (((int)nCurrentKFid - (int)pMP->firstKeyFrameId) >= 3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
}

void LocalMapping::createNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    // For stereo inertial case
    if (monocular)
        nn = 30;
    vector<KeyFrame *> vpNeighKFs =
        p_currentKeyFrame->getBestCovisibilityKeyFrames(nn);

    if (inertial)
    {
        KeyFrame *pKF   = p_currentKeyFrame;
        int       count = 0;
        // nn is the fixed covisibility budget set above (10, or 30 when
        // monocular), so it is always positive here.
        while ((vpNeighKFs.size() <= static_cast<std::size_t>(nn)) &&
               (pKF->p_prevKF) && (count++ < nn))
        {
            vector<KeyFrame *>::iterator it =
                std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->p_prevKF);
            if (it == vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->p_prevKF);
            pKF = pKF->p_prevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th, false);

    Sophus::SE3<float>         sophTcw1 = p_currentKeyFrame->getPose();
    Eigen::Matrix<float, 3, 4> eigTcw1  = sophTcw1.matrix3x4();
    Eigen::Matrix<float, 3, 3> Rcw1     = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> Rwc1     = Rcw1.transpose();
    Eigen::Vector3f            tcw1     = sophTcw1.translation();
    Eigen::Vector3f            Ow1      = p_currentKeyFrame->getCameraCenter();

    const float &fx1 = p_currentKeyFrame->fx;
    const float &fy1 = p_currentKeyFrame->fy;
    const float &cx1 = p_currentKeyFrame->cx;
    const float &cy1 = p_currentKeyFrame->cy;

    const float ratioFactor         = 1.5f * p_currentKeyFrame->scaleFactor;
    int         countStereo         = 0;
    int         countStereoGoodProj = 0;
    int         countStereoAttempt  = 0;
    int         totalStereoPts      = 0;
    // Search matches with epipolar restriction and triangulate
    for (size_t i = 0; i < vpNeighKFs.size(); i++)
    {
        if (i > 0 && checkNewKeyFrames())
            return;

        KeyFrame *pKF2 = vpNeighKFs[i];

        camera_models::geometriccamera::GeometricCamera
            *pCamera1 = p_currentKeyFrame->p_camera,
            *pCamera2 = pKF2->p_camera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2       = pKF2->getCameraCenter();
        Eigen::Vector3f vBaseline = Ow2 - Ow1;
        const float     baseline  = vBaseline.norm();

        if (!monocular)
        {
            if (baseline < pKF2->mb)
                continue;
        }
        else
        {
            const float medianDepthKF2     = pKF2->computeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < 0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t, size_t>> vMatchedIndices;
        bool                         bCoarse = inertial &&
                       p_tracker->state == Tracking::RECENTLY_LOST &&
                       p_currentKeyFrame->getMap()->getInertialBA2();

        matcher.searchForTriangulation(p_currentKeyFrame,
                                       pKF2,
                                       vMatchedIndices,
                                       false,
                                       bCoarse);

        Sophus::SE3<float>         sophTcw2 = pKF2->getPose();
        Eigen::Matrix<float, 3, 4> eigTcw2  = sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw2     = eigTcw2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc2     = Rcw2.transpose();
        Eigen::Vector3f            tcw2     = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for (int ikp = 0; ikp < nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 =
                (p_currentKeyFrame->Nleft == -1)
                    ? p_currentKeyFrame->keyPointsUndistorted[idx1]
                : (idx1 < p_currentKeyFrame->Nleft)
                    ? p_currentKeyFrame->keyPoints[idx1]
                    : p_currentKeyFrame
                          ->keyPointsRight[idx1 - p_currentKeyFrame->Nleft];
            const float kp1_ur = p_currentKeyFrame->uRight[idx1];
            bool bStereo1      = (!p_currentKeyFrame->p_camera2 && kp1_ur >= 0);
            const bool bRight1 = (p_currentKeyFrame->Nleft == -1 ||
                                  idx1 < p_currentKeyFrame->Nleft)
                                     ? false
                                     : true;

            const cv::KeyPoint &kp2 =
                (pKF2->Nleft == -1) ? pKF2->keyPointsUndistorted[idx2]
                : (idx2 < pKF2->Nleft)
                    ? pKF2->keyPoints[idx2]
                    : pKF2->keyPointsRight[idx2 - pKF2->Nleft];

            const float kp2_ur   = pKF2->uRight[idx2];
            bool        bStereo2 = (!pKF2->p_camera2 && kp2_ur >= 0);
            const bool  bRight2 =
                (pKF2->Nleft == -1 || idx2 < pKF2->Nleft) ? false : true;

            if (p_currentKeyFrame->p_camera2 && pKF2->p_camera2)
            {
                if (bRight1 && bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getRightPose();
                    Ow1      = p_currentKeyFrame->getRightCameraCenter();

                    sophTcw2 = pKF2->getRightPose();
                    Ow2      = pKF2->getRightCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera2;
                    pCamera2 = pKF2->p_camera2;
                }
                else if (bRight1 && !bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getRightPose();
                    Ow1      = p_currentKeyFrame->getRightCameraCenter();

                    sophTcw2 = pKF2->getPose();
                    Ow2      = pKF2->getCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera2;
                    pCamera2 = pKF2->p_camera;
                }
                else if (!bRight1 && bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getPose();
                    Ow1      = p_currentKeyFrame->getCameraCenter();

                    sophTcw2 = pKF2->getRightPose();
                    Ow2      = pKF2->getRightCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera;
                    pCamera2 = pKF2->p_camera2;
                }
                else
                {
                    sophTcw1 = p_currentKeyFrame->getPose();
                    Ow1      = p_currentKeyFrame->getCameraCenter();

                    sophTcw2 = pKF2->getPose();
                    Ow2      = pKF2->getCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera;
                    pCamera2 = pKF2->p_camera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1    = eigTcw1.block<3, 3>(0, 0);
                Rwc1    = Rcw1.transpose();
                tcw1    = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2    = eigTcw2.block<3, 3>(0, 0);
                Rwc2    = Rcw2.transpose();
                tcw2    = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float     cosParallaxRays =
                ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            float cosParallaxStereo  = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
                cosParallaxStereo1 =
                    cos(2 * atan2(p_currentKeyFrame->mb / 2,
                                  p_currentKeyFrame->depths[idx1]));
            else if (bStereo2)
                cosParallaxStereo2 =
                    cos(2 * atan2(pKF2->mb / 2, pKF2->depths[idx2]));

            if (bStereo1 || bStereo2)
                totalStereoPts++;

            cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj     = false;
            bool bPointStereo = false;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                (bStereo1 || bStereo2 ||
                 (cosParallaxRays < 0.9996 && inertial) ||
                 (cosParallaxRays < 0.9998 && !inertial)))
            {
                goodProj = GeometricTools::triangulate(xn1,
                                                       xn2,
                                                       eigTcw1,
                                                       eigTcw2,
                                                       x3D);
                if (!goodProj)
                    continue;
            }
            else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj     = p_currentKeyFrame->unprojectStereo(idx1, x3D);
            }
            else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj     = pKF2->unprojectStereo(idx2, x3D);
            }
            else
            {
                continue; // No stereo and very low parallax
            }

            if (goodProj && bPointStereo)
                countStereoGoodProj++;

            if (!goodProj)
                continue;

            // Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if (z1 <= 0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if (z2 <= 0)
                continue;

            // Check reprojection error in first keyframe
            const float &sigmaSquare1 =
                p_currentKeyFrame->levelSigmaSquared[kp1.octave];
            const float x1    = Rcw1.row(0).dot(x3D) + tcw1(0);
            const float y1    = Rcw1.row(1).dot(x3D) + tcw1(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1)
            {
                cv::Point2f uv1   = pCamera1->project(cv::Point3f(x1, y1, z1));
                float       errX1 = uv1.x - kp1.pt.x;
                float       errY1 = uv1.y - kp1.pt.y;

                if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                    continue;
            }
            else
            {
                float u1      = fx1 * x1 * invz1 + cx1;
                float u1_r    = u1 - p_currentKeyFrame->mbf * invz1;
                float v1      = fy1 * y1 * invz1 + cy1;
                float errX1   = u1 - kp1.pt.x;
                float errY1   = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
                    7.8 * sigmaSquare1)
                    continue;
            }

            // Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->levelSigmaSquared[kp2.octave];
            const float x2           = Rcw2.row(0).dot(x3D) + tcw2(0);
            const float y2           = Rcw2.row(1).dot(x3D) + tcw2(1);
            const float invz2        = 1.0 / z2;
            if (!bStereo2)
            {
                cv::Point2f uv2   = pCamera2->project(cv::Point3f(x2, y2, z2));
                float       errX2 = uv2.x - kp2.pt.x;
                float       errY2 = uv2.y - kp2.pt.y;
                if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                    continue;
            }
            else
            {
                float u2      = fx2 * x2 * invz2 + cx2;
                float u2_r    = u2 - p_currentKeyFrame->mbf * invz2;
                float v2      = fy2 * y2 * invz2 + cy2;
                float errX2   = u2 - kp2.pt.x;
                float errY2   = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
                    7.8 * sigmaSquare2)
                    continue;
            }

            // Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float           dist1   = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float           dist2   = normal2.norm();

            if (dist1 == 0 || dist2 == 0)
                continue;

            if (farPoints && (dist1 >= farPointsThreshold ||
                              dist2 >= farPointsThreshold)) // MODIFICATION
                continue;

            const float ratioDist = dist2 / dist1;
            const float ratioOctave =
                p_currentKeyFrame->scaleFactors[kp1.octave] /
                pKF2->scaleFactors[kp2.octave];

            if (ratioDist * ratioFactor < ratioOctave ||
                ratioDist > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint *pMP =
                new MapPoint(x3D, p_currentKeyFrame, p_atlas->getCurrentMap());
            if (bPointStereo)
                countStereo++;

            pMP->addObservation(p_currentKeyFrame, idx1);
            pMP->addObservation(pKF2, idx2);

            p_currentKeyFrame->addMapPoint(pMP, idx1);
            pKF2->addMapPoint(pMP, idx2);

            pMP->computeDistinctiveDescriptors();

            pMP->updateNormalAndDepth();

            p_atlas->addMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}

void LocalMapping::searchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if (monocular)
        nn = 30;
    const vector<KeyFrame *> vpNeighKFs =
        p_currentKeyFrame->getBestCovisibilityKeyFrames(nn);
    vector<KeyFrame *> vpTargetKFs;
    for (vector<KeyFrame *>::const_iterator vit  = vpNeighKFs.begin(),
                                            vend = vpNeighKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame *pKFi = *vit;
        if (pKFi->isBad() ||
            pKFi->fuseTargetKeyFrameId == p_currentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++)
    {
        const vector<KeyFrame *> vpSecondNeighKFs =
            vpTargetKFs[i]->getBestCovisibilityKeyFrames(20);
        for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(),
                                                vend2 = vpSecondNeighKFs.end();
             vit2 != vend2;
             vit2++)
        {
            KeyFrame *pKFi2 = *vit2;
            if (pKFi2->isBad() ||
                pKFi2->fuseTargetKeyFrameId == p_currentKeyFrame->mnId ||
                pKFi2->mnId == p_currentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
        }
        if (abortBA)
            break;
    }

    // Extend to temporal neighbors
    if (inertial)
    {
        KeyFrame *pKFi = p_currentKeyFrame->p_prevKF;
        while (vpTargetKFs.size() < 20 && pKFi)
        {
            if (pKFi->isBad() ||
                pKFi->fuseTargetKeyFrameId == p_currentKeyFrame->mnId)
            {
                pKFi = pKFi->p_prevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
            pKFi                       = pKFi->p_prevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher         matcher;
    vector<MapPoint *> vpMapPointMatches =
        p_currentKeyFrame->getMapPointMatches();
    for (vector<KeyFrame *>::iterator vit  = vpTargetKFs.begin(),
                                      vend = vpTargetKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame *pKFi = *vit;

        matcher.fuse(pKFi, vpMapPointMatches);
        if (pKFi->Nleft != -1)
            matcher.fuse(pKFi, vpMapPointMatches, true);
    }

    if (abortBA)
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint *> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

    for (vector<KeyFrame *>::iterator vitKF  = vpTargetKFs.begin(),
                                      vendKF = vpTargetKFs.end();
         vitKF != vendKF;
         vitKF++)
    {
        KeyFrame *pKFi = *vitKF;

        vector<MapPoint *> vpMapPointsKFi = pKFi->getMapPointMatches();

        for (vector<MapPoint *>::iterator vitMP  = vpMapPointsKFi.begin(),
                                          vendMP = vpMapPointsKFi.end();
             vitMP != vendMP;
             vitMP++)
        {
            MapPoint *pMP = *vitMP;
            if (!pMP)
                continue;
            if (pMP->isBad() ||
                pMP->fuseCandidateKeyFrameId == p_currentKeyFrame->mnId)
                continue;
            pMP->fuseCandidateKeyFrameId = p_currentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.fuse(p_currentKeyFrame, vpFuseCandidates);
    if (p_currentKeyFrame->Nleft != -1)
        matcher.fuse(p_currentKeyFrame, vpFuseCandidates, true);

    // Update points
    vpMapPointMatches = p_currentKeyFrame->getMapPointMatches();
    for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
    {
        MapPoint *pMP = vpMapPointMatches[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                pMP->computeDistinctiveDescriptors();
                pMP->updateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    p_currentKeyFrame->updateConnections();
}

void LocalMapping::requestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    stopRequestedFlag = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    abortBA = true;
}

bool LocalMapping::stop()
{
    unique_lock<mutex> lock(mMutexStop);

    // Check the conditions for stopping the Local Mapping
    if (stopRequestedFlag && !notStop)
    {
        stopped = true;
        return true;
    }
    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return stopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return stopRequestedFlag;
}

void LocalMapping::release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if (finished)
        return;
    stopped           = false;
    stopRequestedFlag = false;
    for (list<KeyFrame *>::iterator lit  = newKeyFrames.begin(),
                                    lend = newKeyFrames.end();
         lit != lend;
         lit++)
        delete *lit;
    newKeyFrames.clear();
}

bool LocalMapping::isAcceptingKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return acceptKeyFrames;
}

void LocalMapping::setAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    acceptKeyFrames = flag;
}

bool LocalMapping::setNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if (flag && stopped)
        return false;

    notStop = flag;

    return true;
}

void LocalMapping::interruptBA()
{
    abortBA = true;
}

void LocalMapping::keyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees,
    // are seen in at least other 3 keyframes (in the same or finer scale) We
    // only consider close stereo points
    const int Nd = 21;
    p_currentKeyFrame->updateBestCovisibles();
    vector<KeyFrame *> vpLocalKeyFrames =
        p_currentKeyFrame->getVectorCovisibleKeyFrames();

    float redundant_th;
    if (!inertial)
        redundant_th = 0.9;
    else if (monocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = p_atlas->isImuInitialized();
    int        count    = 0;

    // Compute the oldest keyframe in the optimizable inertial window.
    unsigned long lastOptimizableKeyFrameId = p_currentKeyFrame->mnId;
    if (inertial)
    {
        int       temporalKeyFrameCount = 0;
        KeyFrame *p_oldestKeyFrame      = p_currentKeyFrame;
        while (temporalKeyFrameCount < Nd && p_oldestKeyFrame->p_prevKF)
        {
            p_oldestKeyFrame = p_oldestKeyFrame->p_prevKF;
            temporalKeyFrameCount++;
        }
        lastOptimizableKeyFrameId = p_oldestKeyFrame->mnId;
    }

    for (vector<KeyFrame *>::iterator vit  = vpLocalKeyFrames.begin(),
                                      vend = vpLocalKeyFrames.end();
         vit != vend;
         vit++)
    {
        count++;
        KeyFrame *pKF = *vit;

        if ((pKF->mnId == pKF->getMap()->getInitKeyFrameId()) || pKF->isBad())
            continue;
        const vector<MapPoint *> vpMapPoints = pKF->getMapPointMatches();

        int       nObs                   = 3;
        const int thObs                  = nObs;
        int       nRedundantObservations = 0;
        int       nMPs                   = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (!monocular)
                    {
                        if (pKF->depths[i] > pKF->depthThreshold ||
                            pKF->depths[i] < 0)
                            continue;
                    }

                    nMPs++;
                    if (pMP->getObservationCount() > thObs)
                    {
                        // Reached only when Nleft != -1, i.e. the fisheye
                        // stereo case, where Nleft is a keypoint count >= 0.
                        const int &scaleLevel =
                            (pKF->Nleft == -1)
                                ? pKF->keyPointsUndistorted[i].octave
                            : (i < static_cast<std::size_t>(pKF->Nleft))
                                ? pKF->keyPoints[i].octave
                                : pKF->keyPointsRight[i].octave;
                        const map<KeyFrame *, tuple<int, int>> observations =
                            pMP->getObservations();
                        int nObs = 0;
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                                 mit  = observations.begin(),
                                 mend = observations.end();
                             mit != mend;
                             mit++)
                        {
                            KeyFrame *pKFi = mit->first;
                            if (pKFi == pKF)
                                continue;
                            tuple<int, int> indexes   = mit->second;
                            int             leftIndex = get<0>(indexes),
                                rightIndex            = get<1>(indexes);
                            int scaleLeveli           = -1;
                            if (pKFi->Nleft == -1)
                                scaleLeveli =
                                    pKFi->keyPointsUndistorted[leftIndex]
                                        .octave;
                            else
                            {
                                if (leftIndex != -1)
                                {
                                    scaleLeveli =
                                        pKFi->keyPoints[leftIndex].octave;
                                }
                                if (rightIndex != -1)
                                {
                                    int rightLevel =
                                        pKFi->keyPointsRight[rightIndex -
                                                             pKFi->Nleft]
                                            .octave;
                                    scaleLeveli = (scaleLeveli == -1 ||
                                                   scaleLeveli > rightLevel)
                                                      ? rightLevel
                                                      : scaleLeveli;
                                }
                            }

                            if (scaleLeveli <= scaleLevel + 1)
                            {
                                nObs++;
                                if (nObs > thObs)
                                    break;
                            }
                        }
                        if (nObs > thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if (nRedundantObservations > redundant_th * nMPs)
        {
            if (inertial)
            {
                if (p_atlas->getKeyFrameCount() <= Nd)
                    continue;

                if (pKF->mnId > (p_currentKeyFrame->mnId - 2))
                    continue;

                if (pKF->p_prevKF && pKF->p_nextKF)
                {
                    const float t =
                        pKF->p_nextKF->timeStamp - pKF->p_prevKF->timeStamp;

                    if ((bInitImu && (pKF->mnId < lastOptimizableKeyFrameId) &&
                         t < 3.) ||
                        (t < 0.5))
                    {
                        pKF->p_nextKF->p_imuPreintegrated->mergePrevious(
                            pKF->p_imuPreintegrated);
                        pKF->p_nextKF->p_prevKF = pKF->p_prevKF;
                        pKF->p_prevKF->p_nextKF = pKF->p_nextKF;
                        pKF->p_nextKF           = nullptr;
                        pKF->p_prevKF           = nullptr;
                        pKF->setBadFlag();
                    }
                    else if (!p_currentKeyFrame->getMap()->getInertialBA2() &&
                             ((pKF->getImuPosition() -
                               pKF->p_prevKF->getImuPosition())
                                  .norm() < 0.02) &&
                             (t < 3))
                    {
                        pKF->p_nextKF->p_imuPreintegrated->mergePrevious(
                            pKF->p_imuPreintegrated);
                        pKF->p_nextKF->p_prevKF = pKF->p_prevKF;
                        pKF->p_prevKF->p_nextKF = pKF->p_nextKF;
                        pKF->p_nextKF           = nullptr;
                        pKF->p_prevKF           = nullptr;
                        pKF->setBadFlag();
                    }
                }
            }
            else
            {
                pKF->setBadFlag();
            }
        }
        if ((count > 20 && abortBA) || count > 100)
        {
            break;
        }
    }
}

void LocalMapping::requestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        // Request to reset the map
        resetRequested = true;
    }

    // Wait until the mutex is free
    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!resetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::requestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        // Request to reset the active map
        resetActiveMapRequested = true;
        p_mapToReset            = pMap;
    }

    // Wait until the mutex is free
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

void LocalMapping::resetIfRequested()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        if (resetRequested)
        {
            cout << "[Mapping] Reseting Atlas in 'LocalMapping' ..." << endl;
            newKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            resetRequested          = false;
            resetActiveMapRequested = false;

            // Inertial parameters
            initializationStartTime = 0.f;
            initIndex               = 0;
            notBA2                  = true;
            notBA1                  = true;
            badImu                  = false;
        }

        if (resetActiveMapRequested)
        {
            cout << "[Mapping] Reseting the Current Map in 'LocalMapping' ..."
                 << endl;

            newKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            initializationStartTime = 0.f;
            notBA2                  = true;
            notBA1                  = true;
            badImu                  = false;
            resetRequested          = false;
            resetActiveMapRequested = false;
        }
    }
}

void LocalMapping::requestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    finishRequested = true;
}

bool LocalMapping::checkFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finishRequested;
}

void LocalMapping::setFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    finished = true;
    unique_lock<mutex> lock2(mMutexStop);
    stopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finished;
}

void LocalMapping::initializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (resetRequested)
        return;

    float       minTime;
    std::size_t nMinKF;
    if (monocular)
    {
        minTime = 2.0;
        nMinKF  = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF  = 10;
    }

    if (p_atlas->getKeyFrameCount() < nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFrame *> lpKF;
    KeyFrame        *pKF = p_currentKeyFrame;
    while (pKF->p_prevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->p_prevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame *> vpKF(lpKF.begin(), lpKF.end());

    if (vpKF.size() < nMinKF)
        return;

    firstTimestamp = vpKF.front()->timeStamp;
    if (p_currentKeyFrame->timeStamp - firstTimestamp < minTime)
        return;

    double accumulatedTranslation_m = 0.0;
    for (std::size_t keyFrameIndex = 1; keyFrameIndex < vpKF.size();
         ++keyFrameIndex)
    {
        accumulatedTranslation_m += (vpKF[keyFrameIndex]->getCameraCenter() -
                                     vpKF[keyFrameIndex - 1]->getCameraCenter())
                                        .norm();
    }

    constexpr double minimumInitializationTranslation_m = 0.05;
    if (accumulatedTranslation_m < minimumInitializationTranslation_m)
        return;

    bInitializing = true;

    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        vpKF.push_back(p_currentKeyFrame);
        lpKF.push_back(p_currentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0, 0, 0, 0, 0, 0);

    // Compute and KF velocities mRwg estimation
    if (!p_currentKeyFrame->getMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for (vector<KeyFrame *>::iterator itKF = vpKF.begin();
             itKF != vpKF.end();
             itKF++)
        {
            if (!(*itKF)->p_imuPreintegrated)
                continue;
            if (!(*itKF)->p_prevKF)
                continue;

            dirG -= (*itKF)->p_prevKF->getImuRotation() *
                    (*itKF)->p_imuPreintegrated->getUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = ((*itKF)->getImuPosition() -
                                    (*itKF)->p_prevKF->getImuPosition()) /
                                   (*itKF)->p_imuPreintegrated->dT;
            (*itKF)->setVelocity(_vel);
            (*itKF)->p_prevKF->setVelocity(_vel);
        }

        dirG = dirG / dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v    = gI.cross(dirG);
        const float     nv   = v.norm();
        const float     cosg = gI.dot(dirG);
        const float     ang  = acos(cosg);
        Eigen::Vector3f vzg(0.0f, 0.0f, 0.0f); // = v*ang/nv;
        if (nv != 0 && !isnan(cosg) && !isnan(ang))
            vzg = v * ang / nv;
        Rwg                     = Sophus::SO3f::exp(vzg).matrix();
        mRwg                    = Rwg.cast<double>();
        initializationStartTime = p_currentKeyFrame->timeStamp - firstTimestamp;
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg  = p_currentKeyFrame->getGyroBias().cast<double>();
        mba  = p_currentKeyFrame->getAccBias().cast<double>();
    }

    scale = 1.0;

    initTime = p_tracker->lastFrame.timeStamp - vpKF.front()->timeStamp;

    Optimizer::inertialOptimization(p_atlas->getCurrentMap(),
                                    mRwg,
                                    scale,
                                    mbg,
                                    mba,
                                    monocular,
                                    infoInertial,
                                    false,
                                    false,
                                    priorG,
                                    priorA);

    if (scale < 1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing = false;
        return;
    }

    // Before this line we are not changing the map
    {
        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();
        Map *p_activeMap = p_atlas->getCurrentMap();

        if (p_activeMap == nullptr)
        {
            bInitializing = false;
            return;
        }

        const bool         imuWasInitialized = p_atlas->isImuInitialized();
        unique_lock<mutex> lock(p_activeMap->mMutexMapUpdate);
        if ((fabs(scale - 1.f) > 0.00001) || !monocular)
        {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(),
                             Eigen::Vector3f::Zero());
            p_activeMap->applyScaledRotation(Twg, scale, true);
            p_tracker->updateFrameIMU(scale,
                                      vpKF[0]->getImuBias(),
                                      p_currentKeyFrame);
        }

        // Check if initialization OK
        if (!imuWasInitialized)
            for (int i = 0; i < N; i++)
            {
                KeyFrame *pKF2 = vpKF[i];
                pKF2->isImu    = true;
            }
    }

    p_tracker->updateFrameIMU(1.0, vpKF[0]->getImuBias(), p_currentKeyFrame);
    if (!p_atlas->isImuInitialized())
    {
        p_atlas->setImuInitialized();
        p_tracker->t0IMU         = p_tracker->currentFrame.timeStamp;
        p_currentKeyFrame->isImu = true;
    }

    if (bFIBA)
    {
        if (priorA != 0.f)
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->mnId,
                                      nullptr,
                                      true,
                                      priorG,
                                      priorA);
        else
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->mnId,
                                      nullptr,
                                      false);
    }

    Verbose::printMess("Global Bundle Adjustment finished\nUpdating map ...",
                       Verbose::VERBOSITY_NORMAL);

    /* Keep semantic observations valid while corrected KFs are retired. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    // Get Map Mutex
    unique_lock<mutex> lock(p_atlas->getCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = p_currentKeyFrame->mnId;

    // Process keyframes in the queue
    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        vpKF.push_back(p_currentKeyFrame);
        lpKF.push_back(p_currentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFrame *> lpKFtoCheck(
        p_atlas->getCurrentMap()->keyFrameOrigins.begin(),
        p_atlas->getCurrentMap()->keyFrameOrigins.end());

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

            if (pChild->baGlobalKeyFrameId != GBAid)
            {
                Sophus::SE3f Tchildc = pChild->getPose() * Twc;
                pChild->tcwGBA       = Tchildc * pKF->tcwGBA;

                Sophus::SO3f Rcor =
                    pChild->tcwGBA.so3().inverse() * pChild->getPose().so3();
                if (pChild->isVelocitySet())
                {
                    pChild->vwbGBA = Rcor * pChild->getVelocity();
                }
                else
                {
                    Verbose::printMess("Child velocity empty!! ",
                                       Verbose::VERBOSITY_NORMAL);
                }

                pChild->biasGBA            = pChild->getImuBias();
                pChild->baGlobalKeyFrameId = GBAid;
            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->tcwBefGBA = pKF->getPose();
        pKF->setPose(pKF->tcwGBA);

        if (pKF->isImu)
        {
            pKF->vwbBefGBA = pKF->getVelocity();
            pKF->setVelocity(pKF->vwbGBA);
            pKF->setNewBias(pKF->biasGBA);
        }
        else
        {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPoint *> vpMPs =
        p_atlas->getCurrentMap()->getAllMapPoints();

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        MapPoint *pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        if (pMP->baGlobalKeyFrameId == GBAid)
        {
            // If optimized by Global BA, just update
            pMP->setWorldPos(pMP->posGBA);
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFrame *pRefKF = pMP->getReferenceKeyFrame();

            if (pRefKF->baGlobalKeyFrameId != GBAid)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->tcwBefGBA * pMP->getWorldPos();

            // Backproject using corrected camera
            pMP->setWorldPos(pRefKF->getPoseInverse() * Xc);
        }
    }

    Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    keyFrameCount = vpKF.size();
    initIndex++;

    for (list<KeyFrame *>::iterator lit  = newKeyFrames.begin(),
                                    lend = newKeyFrames.end();
         lit != lend;
         lit++)
    {
        (*lit)->setBadFlag();
        delete *lit;
    }
    newKeyFrames.clear();

    p_tracker->state = Tracking::OK;
    bInitializing    = false;

    p_currentKeyFrame->getMap()->increaseChangeIndex();

    return;
}

void LocalMapping::scaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a
    // solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (resetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame *> lpKF;
    KeyFrame        *pKF = p_currentKeyFrame;
    while (pKF->p_prevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->p_prevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame *> vpKF(lpKF.begin(), lpKF.end());

    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        vpKF.push_back(p_currentKeyFrame);
        lpKF.push_back(p_currentKeyFrame);
    }

    mRwg  = Eigen::Matrix3d::Identity();
    scale = 1.0;

    Optimizer::inertialOptimization(p_atlas->getCurrentMap(), mRwg, scale);

    if (scale < 1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing = false;
        return;
    }

    Sophus::SO3d                 so3wg(mRwg);
    // Before this line we are not changing the map
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();
    Map *p_activeMap = p_atlas->getCurrentMap();

    if (p_activeMap == nullptr)
    {
        bInitializing = false;
        return;
    }

    unique_lock<mutex> lock(p_activeMap->mMutexMapUpdate);
    if ((fabs(scale - 1.f) > 0.002) || !monocular)
    {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),
                         Eigen::Vector3f::Zero());
        p_activeMap->applyScaledRotation(Tgw, scale, true);
        p_tracker->updateFrameIMU(scale,
                                  p_currentKeyFrame->getImuBias(),
                                  p_currentKeyFrame);
    }

    for (list<KeyFrame *>::iterator lit  = newKeyFrames.begin(),
                                    lend = newKeyFrames.end();
         lit != lend;
         lit++)
    {
        (*lit)->setBadFlag();
        delete *lit;
    }
    newKeyFrames.clear();

    // To perform pose-inertial opt w.r.t. last keyframe
    p_currentKeyFrame->getMap()->increaseChangeIndex();

    return;
}

bool LocalMapping::isInitializing()
{
    return bInitializing;
}

double LocalMapping::getCurrentKeyFrameTime()
{

    if (p_currentKeyFrame)
    {
        return p_currentKeyFrame->timeStamp;
    }
    else
        return 0.0;
}

KeyFrame *LocalMapping::getCurrentKeyFrame()
{
    return p_currentKeyFrame;
}

} // namespace core
} // namespace vs_graphs
