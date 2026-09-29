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

void LoopClosing::runGlobalBundleAdjustment(Map          *p_activeMap_inout,
                                            unsigned long loopKeyFrameCount_in,
                                            unsigned int  generation_in)
{
    Verbose::printMess("Starting Global Bundle Adjustment",
                       Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeStartFGba =
        std::chrono::steady_clock::now();

    fullGbaExecutionCount += 1;

    gbaKeyFrameCounts.push_back(p_activeMap_inout->getAllKeyFrames().size());
    gbaMapPointCounts.push_back(p_activeMap_inout->getAllMapPoints().size());
#endif

    /*
     * g2o accepts a plain bool force-stop token. A first-party iteration action
     * copies the atomic cross-thread request into this worker-local flag, so
     * cancellation remains race-free and takes effect between iterations.
     */
    bool optimizerStopRequested = false;

    const bool isImuInitialized = p_activeMap_inout->isImuInitialized();

    if (!isImuInitialized)
        Optimizer::globalBundleAdjustment(
            p_activeMap_inout,
            10,
            &optimizerStopRequested,
            loopKeyFrameCount_in,
            false,
            p_tracker->getMarkerImpact(),
            &isGlobalBundleAdjustmentStopRequested);
    else
        Optimizer::fullInertialBA(p_activeMap_inout,
                                  7,
                                  false,
                                  loopKeyFrameCount_in,
                                  &optimizerStopRequested,
                                  false,
                                  1e2F,
                                  1e6F,
                                  nullptr,
                                  nullptr,
                                  &isGlobalBundleAdjustmentStopRequested);

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
            return;
        }

        if (!isImuInitialized && p_activeMap_inout->isImuInitialized())
        {
            hasGbaFinished = true;
            isGbaRunning   = false;
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
            unique_lock<mutex> lock(p_activeMap_inout->mapUpdateMutex);

            KeyFrameAndPose keyFramePosesBefore_WorldToCamera;
            KeyFrameAndPose keyFramePosesAfter_WorldToCamera;

            //  Correct keyframes starting at map first keyframe
            list<KeyFrame *> keyFramesToCheck(
                p_activeMap_inout->keyFrameOrigins.begin(),
                p_activeMap_inout->keyFrameOrigins.end());

            while (!keyFramesToCheck.empty())
            {
                KeyFrame             *p_keyFrame = keyFramesToCheck.front();
                const set<KeyFrame *> childs     = p_keyFrame->getChilds();
                Sophus::SE3f          Twc        = p_keyFrame->getPoseInverse();
                for (set<KeyFrame *>::const_iterator sit = childs.begin();
                     sit != childs.end();
                     sit++)
                {
                    KeyFrame *p_child = *sit;
                    if (!p_child || p_child->isBad())
                        continue;

                    if (p_child->baGlobalKeyFrameId != loopKeyFrameCount_in)
                    {
                        Sophus::SE3f tchildc = p_child->getPose() * Twc;
                        p_child->tcwGBA =
                            tchildc * p_keyFrame->tcwGBA; //*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = p_child->tcwGBA.so3().inverse() *
                                            p_child->getPose().so3();
                        if (p_child->isVelocitySet())
                        {
                            p_child->vwbGBA = Rcor * p_child->getVelocity();
                        }
                        else
                            Verbose::printMess("Child velocity empty!! ",
                                               Verbose::VERBOSITY_NORMAL);

                        p_child->biasGBA = p_child->getImuBias();

                        p_child->baGlobalKeyFrameId = loopKeyFrameCount_in;
                    }
                    keyFramesToCheck.push_back(p_child);
                }

                p_keyFrame->tcwBefGBA = p_keyFrame->getPose();

                const Sophus::SE3d poseBefore_WorldToCamera =
                    p_keyFrame->tcwBefGBA.cast<double>();

                keyFramePosesBefore_WorldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseBefore_WorldToCamera.unit_quaternion(),
                              poseBefore_WorldToCamera.translation(),
                              1.0));

                p_keyFrame->setPose(p_keyFrame->tcwGBA);

                const Sophus::SE3d poseAfter_WorldToCamera =
                    p_keyFrame->getPose().cast<double>();

                keyFramePosesAfter_WorldToCamera.insert_or_assign(
                    p_keyFrame,
                    g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                              poseAfter_WorldToCamera.translation(),
                              1.0));

                if (p_keyFrame->isImu)
                {
                    p_keyFrame->vwbBefGBA = p_keyFrame->getVelocity();

                    p_keyFrame->setVelocity(p_keyFrame->vwbGBA);
                    p_keyFrame->setNewBias(p_keyFrame->biasGBA);
                }

                keyFramesToCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint *> mapPoints =
                p_activeMap_inout->getAllMapPoints();

            for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
                 mapPointIndex++)
            {
                MapPoint *p_mapPoint = mapPoints[mapPointIndex];

                if (p_mapPoint == nullptr || p_mapPoint->isBad())
                    continue;

                bool mapPointWasCorrected = false;

                if (p_mapPoint->baGlobalKeyFrameId == loopKeyFrameCount_in)
                {
                    // If optimized by Global BA, just update
                    p_mapPoint->setWorldPos(p_mapPoint->posGBA);
                    mapPointWasCorrected = true;
                }
                else
                {
                    // Update according to the correction of its reference
                    // keyframe
                    KeyFrame *p_referenceKeyFrame =
                        p_mapPoint->getReferenceKeyFrame();

                    if (p_referenceKeyFrame == nullptr ||
                        p_referenceKeyFrame->isBad() ||
                        p_referenceKeyFrame->getMap() != p_activeMap_inout ||
                        p_referenceKeyFrame->baGlobalKeyFrameId !=
                            loopKeyFrameCount_in)
                    {
                        p_referenceKeyFrame = nullptr;

                        const auto observations = p_mapPoint->getObservations();

                        for (const auto &[p_observingKeyFrame, featureIndexes] :
                             observations)
                        {
                            (void)featureIndexes;

                            if (p_observingKeyFrame == nullptr ||
                                p_observingKeyFrame->isBad() ||
                                p_observingKeyFrame->getMap() !=
                                    p_activeMap_inout ||
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
                    Eigen::Vector3f Xc = p_referenceKeyFrame->tcwBefGBA *
                                         p_mapPoint->getWorldPos();

                    // Backproject using corrected camera
                    p_mapPoint->setWorldPos(
                        p_referenceKeyFrame->getPoseInverse() * Xc);
                    mapPointWasCorrected = true;
                }

                if (mapPointWasCorrected)
                {
                    p_mapPoint->updateNormalAndDepth();
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
            for (geometric::Plane *p_plane : p_activeMap_inout->getAllPlanes())
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

            p_activeMap_inout->informNewBigChange();
            p_activeMap_inout->increaseChangeIndex();

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
                    time_EndUpdateMap - timeEndGba)
                    .count();
            updateMapTimes_ms.push_back(timeUpdateMap);

            double timeFGba = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndUpdateMap - timeStartFGba)
                                  .count();
            fullGbaTotalTimes_ms.push_back(timeFGba);
#endif
            Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        hasGbaFinished = true;
        isGbaRunning   = false;
    }
}

} // namespace core
} // namespace vs_graphs
