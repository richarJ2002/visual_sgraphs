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
#include "System.h"
#include "Tracking.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::needNewKeyFrame(bool &needNewKeyFrame_out)
{
    bool isImuInitialized2{};
    Map *p_atlasCurrentMap = nullptr;
    if ((((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
           sensor == System::IMU_RGBD))) &&
        p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
          sensor == System::IMU_RGBD)) &&
        p_atlasCurrentMap->isImuInitialized(isImuInitialized2) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        !isImuInitialized2)
    {
        if (sensor == System::IMU_MONOCULAR &&
            (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >= 0.25)
        {
            needNewKeyFrame_out = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
        else if ((sensor == System::IMU_STEREO || sensor == System::IMU_RGBD) &&
                 (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >= 0.25)
        {
            needNewKeyFrame_out = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
        else
        {
            needNewKeyFrame_out = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
    }

    if (isTrackingOnlyMode)
    {
        needNewKeyFrame_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    bool localMapperIsStopped{};
    if (p_localMapper->isStopped(localMapperIsStopped) !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isStopped returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool localMapperIsStopRequested{};
    if (!(localMapperIsStopped) &&
        p_localMapper->stopRequested(localMapperIsStopRequested) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: stopRequested returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (localMapperIsStopped || localMapperIsStopRequested)
    {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        needNewKeyFrame_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    unsigned long keyFrameCountValue{};
    if (p_atlas->getKeyFrameCount(keyFrameCountValue) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const int keyFrameCount = static_cast<int>(keyFrameCountValue);

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    if (currentFrame.id < lastRelocFrameId + maxFrames &&
        keyFrameCount > maxFrames)
    {
        needNewKeyFrame_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    // Tracked MapPoints in the reference keyframe
    int minimumObservationCount = 3;
    if (keyFrameCount <= 2)
        minimumObservationCount = 2;
    int referenceMatchCount{};
    if (p_referenceKF->getTrackedMapPointCount(minimumObservationCount,
                                               referenceMatchCount) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTrackedMapPointCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nonTrackedCloseCount = 0;
    int trackedCloseCount    = 0;

    if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
    {
        int N = (currentFrame.leftKeyPointCount == -1)
                    ? currentFrame.keyPointCount
                    : currentFrame.leftKeyPointCount;
        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            if (currentFrame.depths[keyPointIndex] > 0 &&
                currentFrame.depths[keyPointIndex] < depthThreshold)
            {
                if (currentFrame.mapPoints[keyPointIndex] &&
                    !currentFrame.outlierFlags[keyPointIndex])
                    trackedCloseCount++;
                else
                    nonTrackedCloseCount++;
            }
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose =
        (trackedCloseCount < 100) && (nonTrackedCloseCount > 70);

    // AGGRESSIVE CORRIDOR TRACKING: Stricter KF insertion criteria
    // Require minimum 30 total inliers, 15 close inliers, 1.0s temporal spacing
    const bool hasEnoughTotalInliers = (matchesInliers >= minInliersForKF);
    const bool hasEnoughCloseInliers =
        (trackedCloseCount >= minCloseInliersForKF);
    const bool hasEnoughTimeSinceLastKeyFrame =
        p_lastKeyFrame && (currentFrame.timeStamp - p_lastKeyFrame->timeStamp >=
                           minKeyFrameTemporalSpacing);

    // Thresholds
    float thresholdReferenceRatio = 0.75f;
    if (keyFrameCount < 2)
        thresholdReferenceRatio = 0.4f;

    if (sensor == System::MONOCULAR)
        thresholdReferenceRatio = 0.9f;

    if (p_camera2)
        thresholdReferenceRatio = 0.75f;

    if (sensor == System::IMU_MONOCULAR)
    {
        if (matchesInliers > 350)
            thresholdReferenceRatio = 0.75f;
        else
            thresholdReferenceRatio = 0.90f;
    }

    // Local Mapping accept keyframes?
    bool isLocalMappingIdle{};
    if (p_localMapper->isAcceptingKeyFrames(isLocalMappingIdle) !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isAcceptingKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    const bool c1a = currentFrame.id >= lastKeyFrameId + maxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    int        localMapperKeyFrameCount{};
    if (((currentFrame.id >= lastKeyFrameId + minFrames) &&
         isLocalMappingIdle) &&
        p_localMapper->keyframesInQueue(localMapperKeyFrameCount) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: keyframesInQueue returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const bool c1b = ((currentFrame.id >= lastKeyFrameId + minFrames) &&
                      isLocalMappingIdle && localMapperKeyFrameCount < 5);
    // Condition 1c: tracking is weak
    const bool c1c =
        sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR &&
        sensor != System::IMU_STEREO && sensor != System::IMU_RGBD &&
        (matchesInliers < referenceMatchCount * 0.25 || bNeedToInsertClose) &&
        matchesInliers > 20;
    // Condition 2: Few tracked points compared to reference keyframe.
    const bool c2 =
        (((matchesInliers < referenceMatchCount * thresholdReferenceRatio ||
           bNeedToInsertClose)) &&
         matchesInliers > minInliersForKF);

    // AGGRESSIVE: Additional corridor-specific conditions
    // Condition 3: Temporal spacing (1.0s minimum)
    bool c3 = false;
    if (p_lastKeyFrame)
    {
        if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
             sensor == System::IMU_RGBD) &&
            (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >=
                minKeyFrameTemporalSpacing)
        {
            c3 = true;
        }
    }

    // Condition 4: Enough inliers but temporal spacing met
    bool c4 = false;
    if (hasEnoughTotalInliers && hasEnoughCloseInliers &&
        hasEnoughTimeSinceLastKeyFrame)
    {
        c4 = true;
    }
    // Also insert if tracking is weak (RECENTLY_LOST) and we have minimum
    // inliers
    else if (state == RECENTLY_LOST && matchesInliers > minInliersForKF)
    {
        c4 = true;
    }

    if (((c1a || c1b || c1c) && c2) || c3 || c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        bool localMapperIsInitializing{};
        if (!(isLocalMappingIdle) &&
            p_localMapper->isInitializing(localMapperIsInitializing) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInitializing returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (isLocalMappingIdle || localMapperIsInitializing)
        {
            needNewKeyFrame_out = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
        else
        {
            if (p_localMapper->interruptBA() !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: interruptBA returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
            {
                int localMapperKeyFrameCount2{};
                if (p_localMapper->keyframesInQueue(
                        localMapperKeyFrameCount2) !=
                    LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: keyframesInQueue returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (localMapperKeyFrameCount2 < 8)
                {
                    needNewKeyFrame_out = true;
                    return TrackingStatus::TRACKING_STATUS_SUCCESS;
                }
                else
                {
                    needNewKeyFrame_out = false;
                    return TrackingStatus::TRACKING_STATUS_SUCCESS;
                }
            }
            else
            {
                needNewKeyFrame_out = false;
                return TrackingStatus::TRACKING_STATUS_SUCCESS;
            }
        }
    }
    else
    {
        needNewKeyFrame_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
