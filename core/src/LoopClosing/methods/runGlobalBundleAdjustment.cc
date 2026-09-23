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

#include "Optimizer.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

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

} // namespace core
} // namespace vs_graphs
