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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LocalMappingStatus LocalMapping::keyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees,
    // are seen in at least other 3 keyframes (in the same or finer scale) We
    // only consider close stereo points
    const int temporalWindowSize = 21;
    if (p_currentKeyFrame->updateBestCovisibles() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateBestCovisibles returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<KeyFrame *> neighborKeyFrames{};
    if (p_currentKeyFrame->getVectorCovisibleKeyFrames(neighborKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    float redundancyThreshold;
    if (!isInertial)
        redundancyThreshold = 0.9;
    else if (isMonocular)
        redundancyThreshold = 0.9;
    else
        redundancyThreshold = 0.5;

    bool isImuInitialized{};
    if (p_atlas->isImuInitialized(isImuInitialized) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int processedKeyFrameCount = 0;

    // Compute the oldest keyframe in the optimizable inertial window.
    unsigned long lastOptimizableKeyFrameId = p_currentKeyFrame->id;
    if (isInertial)
    {
        int       temporalKeyFrameCount = 0;
        KeyFrame *p_oldestKeyFrame      = p_currentKeyFrame;
        while (temporalKeyFrameCount < temporalWindowSize &&
               p_oldestKeyFrame->p_prevKF)
        {
            p_oldestKeyFrame = p_oldestKeyFrame->p_prevKF;
            temporalKeyFrameCount++;
        }
        lastOptimizableKeyFrameId = p_oldestKeyFrame->id;
    }

    for (vector<KeyFrame *>::iterator
             neighborKeyFrameIt  = neighborKeyFrames.begin(),
             neighborKeyFrameEnd = neighborKeyFrames.end();
         neighborKeyFrameIt != neighborKeyFrameEnd;
         neighborKeyFrameIt++)
    {
        processedKeyFrameCount++;
        KeyFrame *p_neighborKeyFrame = *neighborKeyFrameIt;

        Map *p_neighborKeyFrameMap = nullptr;
        if (p_neighborKeyFrame->getMap(p_neighborKeyFrameMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long initKeyFrameId{};
        if (p_neighborKeyFrameMap->getInitKeyFrameId(initKeyFrameId) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getInitKeyFrameId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool neighborKeyFrameIsBad{};
        if (!(p_neighborKeyFrame->id == initKeyFrameId) &&
            p_neighborKeyFrame->isBad(neighborKeyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if ((p_neighborKeyFrame->id == initKeyFrameId) || neighborKeyFrameIsBad)
            continue;
        std::vector<MapPoint *> neighborMapPoints{};
        if (p_neighborKeyFrame->getMapPointMatches(neighborMapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        int       observationCount          = 3;
        const int observationThreshold      = observationCount;
        int       redundantObservationCount = 0;
        int       validMapPointCount        = 0;
        for (size_t mapPointIndex = 0, mapPointCount = neighborMapPoints.size();
             mapPointIndex < mapPointCount;
             mapPointIndex++)
        {
            MapPoint *p_mapPoint = neighborMapPoints[mapPointIndex];
            if (p_mapPoint)
            {
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad)
                {
                    if (!isMonocular)
                    {
                        if (p_neighborKeyFrame->depths[mapPointIndex] >
                                p_neighborKeyFrame->depthThreshold ||
                            p_neighborKeyFrame->depths[mapPointIndex] < 0)
                            continue;
                    }

                    validMapPointCount++;
                    int mapPointObservationCount{};
                    if (p_mapPoint->getObservationCount(
                            mapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (mapPointObservationCount > observationThreshold)
                    {
                        // Reached only when Nleft != -1, i.e. the fisheye
                        // stereo case, where Nleft is a keypoint count >= 0.
                        const int &scaleLevel =
                            (p_neighborKeyFrame->leftKeyPointCount == -1)
                                ? p_neighborKeyFrame
                                      ->keyPointsUndistorted[mapPointIndex]
                                      .octave
                            : (mapPointIndex <
                               static_cast<std::size_t>(
                                   p_neighborKeyFrame->leftKeyPointCount))
                                ? p_neighborKeyFrame->keyPoints[mapPointIndex]
                                      .octave
                                : p_neighborKeyFrame
                                      ->keyPointsRight[mapPointIndex]
                                      .octave;
                        std::map<KeyFrame *, std::tuple<int, int>>
                            pointObservations{};
                        if (p_mapPoint->getObservations(pointObservations) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getObservations returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        int observationCount = 0;
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                                 observationIt  = pointObservations.begin(),
                                 observationEnd = pointObservations.end();
                             observationIt != observationEnd;
                             observationIt++)
                        {
                            KeyFrame *p_observingKeyFrame =
                                observationIt->first;
                            if (p_observingKeyFrame == p_neighborKeyFrame)
                                continue;
                            tuple<int, int> observationIndices =
                                observationIt->second;
                            int leftIndex          = get<0>(observationIndices),
                                rightIndex         = get<1>(observationIndices);
                            int observerScaleLevel = -1;
                            if (p_observingKeyFrame->leftKeyPointCount == -1)
                                observerScaleLevel =
                                    p_observingKeyFrame
                                        ->keyPointsUndistorted[leftIndex]
                                        .octave;
                            else
                            {
                                if (leftIndex != -1)
                                {
                                    observerScaleLevel =
                                        p_observingKeyFrame
                                            ->keyPoints[leftIndex]
                                            .octave;
                                }
                                if (rightIndex != -1)
                                {
                                    int rightLevel =
                                        p_observingKeyFrame
                                            ->keyPointsRight
                                                [rightIndex -
                                                 p_observingKeyFrame
                                                     ->leftKeyPointCount]
                                            .octave;
                                    observerScaleLevel =
                                        (observerScaleLevel == -1 ||
                                         observerScaleLevel > rightLevel)
                                            ? rightLevel
                                            : observerScaleLevel;
                                }
                            }

                            if (observerScaleLevel <= scaleLevel + 1)
                            {
                                observationCount++;
                                if (observationCount > observationThreshold)
                                    break;
                            }
                        }
                        if (observationCount > observationThreshold)
                        {
                            redundantObservationCount++;
                        }
                    }
                }
            }
        }

        if (redundantObservationCount >
            redundancyThreshold * validMapPointCount)
        {
            if (isInertial)
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
                if (atlasKeyFrameCount <= temporalWindowSize)
                    continue;

                if (p_neighborKeyFrame->id > (p_currentKeyFrame->id - 2))
                    continue;

                if (p_neighborKeyFrame->p_prevKF &&
                    p_neighborKeyFrame->p_nextKF)
                {
                    const float timeGap =
                        p_neighborKeyFrame->p_nextKF->timeStamp -
                        p_neighborKeyFrame->p_prevKF->timeStamp;

                    if ((isImuInitialized &&
                         (p_neighborKeyFrame->id < lastOptimizableKeyFrameId) &&
                         timeGap < 3.) ||
                        (timeGap < 0.5))
                    {
                        if (p_neighborKeyFrame->p_nextKF->p_imuPreintegrated
                                ->mergePrevious(
                                    p_neighborKeyFrame->p_imuPreintegrated) !=
                            IMU::PreintegratedStatus::
                                PREINTEGRATED_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: mergePrevious returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        p_neighborKeyFrame->p_nextKF->p_prevKF =
                            p_neighborKeyFrame->p_prevKF;
                        p_neighborKeyFrame->p_prevKF->p_nextKF =
                            p_neighborKeyFrame->p_nextKF;
                        p_neighborKeyFrame->p_nextKF = nullptr;
                        p_neighborKeyFrame->p_prevKF = nullptr;
                        if (p_neighborKeyFrame->setBadFlag() !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setBadFlag returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                    else
                    {
                        Map *p_currentKeyFrameMap = nullptr;
                        if (p_currentKeyFrame->getMap(p_currentKeyFrameMap) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getMap returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        bool inertialBA2{};
                        if (p_currentKeyFrameMap->getInertialBA2(inertialBA2) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getInertialBA2 returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f neighborKeyFrameImuPosition{};
                        if ((!inertialBA2) &&
                            p_neighborKeyFrame->getImuPosition(
                                neighborKeyFrameImuPosition) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getImuPosition returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f imuPosition{};
                        if ((!inertialBA2) &&
                            p_neighborKeyFrame->p_prevKF->getImuPosition(
                                imuPosition) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getImuPosition returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (!inertialBA2 &&
                            ((neighborKeyFrameImuPosition - imuPosition)
                                 .norm() < 0.02) &&
                            (timeGap < 3))
                        {
                            if (p_neighborKeyFrame->p_nextKF->p_imuPreintegrated
                                    ->mergePrevious(p_neighborKeyFrame
                                                        ->p_imuPreintegrated) !=
                                IMU::PreintegratedStatus::
                                    PREINTEGRATED_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: mergePrevious returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            p_neighborKeyFrame->p_nextKF->p_prevKF =
                                p_neighborKeyFrame->p_prevKF;
                            p_neighborKeyFrame->p_prevKF->p_nextKF =
                                p_neighborKeyFrame->p_nextKF;
                            p_neighborKeyFrame->p_nextKF = nullptr;
                            p_neighborKeyFrame->p_prevKF = nullptr;
                            if (p_neighborKeyFrame->setBadFlag() !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: setBadFlag returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                        }
                    }
                }
            }
            else
            {
                if (p_neighborKeyFrame->setBadFlag() !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setBadFlag returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        if ((processedKeyFrameCount > 20 && shouldAbortBa) ||
            processedKeyFrameCount > 100)
        {
            break;
        }
    }

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
