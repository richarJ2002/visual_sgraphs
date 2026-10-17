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
 * @file            createNewKeyFrame.cc
 *
 * @brief           Implements Tracking::createNewKeyFrame(), declared in
 *                  Tracking.h.
 */

#include "local_mapping/local_mapping.h"
#include "atlas/objects/AtlasStatusEnum.h"
#include "frame/objects/FrameStatusEnum.h"
#include "imu_types/bias/bias.h"
#include "imu_types/preintegrated/preintegrated.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "local_mapping/objects/LocalMappingStatusEnum.h"
#include "map/map.h"
#include "map_point/map_point.h"
#include "map_point/objects/MapPointStatusEnum.h"
#include "semantic/marker/marker.h"
#include "semantic/marker/objects/MarkerStatusEnum.h"
#include "system/objects/VerboseStatusEnum.h"
#include "system/system.h"
#include "tracking/objects/TrackingClass.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::createNewKeyFrame()
{
    bool localMapperIsInitializing{};
    if (p_localMapper->isInitializing(localMapperIsInitializing) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInitializing returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool atlasIsImuInitialized{};
    if ((localMapperIsInitializing) &&
        p_atlas->isImuInitialized(atlasIsImuInitialized) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (localMapperIsInitializing && !atlasIsImuInitialized)
    {
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    bool localMapperWasSet{};
    if (p_localMapper->setNotStop(true, localMapperWasSet) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setNotStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!localMapperWasSet)
    {
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    map::Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    keyframe::KeyFrame *p_keyFrame = new keyframe::KeyFrame(currentFrame,
                                                            p_atlasCurrentMap,
                                                            p_keyFrameDatabase);

    bool atlasIsImuInitialized2{};
    if (p_atlas->isImuInitialized(atlasIsImuInitialized2) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (atlasIsImuInitialized2)
    {
        p_keyFrame->isImu = true;
    }

    if (p_keyFrame->setNewBias(currentFrame.imuBias) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setNewBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_referenceKF                    = p_keyFrame;
    currentFrame.p_referenceKeyFrame = p_keyFrame;

    if (p_lastKeyFrame)
    {
        p_keyFrame->p_prevKF     = p_lastKeyFrame;
        p_lastKeyFrame->p_nextKF = p_keyFrame;
    }
    else
    {
        if (system::Verbose::printMess("No last KF in KF creation!!",
                               system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Reset preintegration from last KF (Create new object) */
    if (sensor == system::System::IMU_MONOCULAR ||
        sensor == system::System::IMU_STEREO ||
        sensor == system::System::IMU_RGBD)
    {
        imu_types::bias::Bias keyFrameImuBias{};
        if (p_keyFrame->getImuBias(keyFrameImuBias) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_imuPreintegratedFromLastKF =
            new imu_types::preintegrated::Preintegrated(keyFrameImuBias, p_keyFrame->imuCalibration);
    }

    if (sensor != system::System::MONOCULAR &&
        sensor != system::System::IMU_MONOCULAR)
    {
        if (currentFrame.updatePoseMatrices() !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updatePoseMatrices returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        /*!
         * We sort points by the measured depth by the stereo/RGBD sensor.
         * We create all those MapPoints whose depth < mThDepth.
         * If there are less than 100 close points we create the 100 closest.
         * Both sensor branches intentionally use the same cap of 100.
         */
        int maximumPoint = 100;
        if (sensor == system::System::IMU_STEREO ||
            sensor == system::System::IMU_RGBD)
        {
            maximumPoint = 100;
        }

        std::vector<std::pair<float, int>> depthIndices;
        int N = (currentFrame.leftKeyPointCount != -1)
                    ? currentFrame.leftKeyPointCount
                    : currentFrame.keyPointCount;
        depthIndices.reserve(currentFrame.keyPointCount);
        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            float z = currentFrame.depths[keyPointIndex];
            if (z > 0)
            {
                depthIndices.push_back(std::make_pair(z, keyPointIndex));
            }
        }

        if (!depthIndices.empty())
        {
            std::sort(depthIndices.begin(), depthIndices.end());

            int pointCount = 0;
            for (size_t depthIndexIndex = 0;
                 depthIndexIndex < depthIndices.size();
                 depthIndexIndex++)
            {
                bool shouldCreateNewPoint = false;
                int  keyPointIndex = depthIndices[depthIndexIndex].second;

                map_point::MapPoint *p_mapPoint =
                    currentFrame.mapPoints[keyPointIndex];
                if (!p_mapPoint)
                {
                    shouldCreateNewPoint = true;
                }
                else
                {
                    int mapPointObservationCount{};
                    if (p_mapPoint->getObservationCount(
                            mapPointObservationCount) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (mapPointObservationCount < 1)
                    {
                        shouldCreateNewPoint = true;
                        currentFrame.mapPoints[keyPointIndex] =
                            static_cast<map_point::MapPoint *>(nullptr);
                    }
                }

                if (shouldCreateNewPoint)
                {
                    Eigen::Vector3f x3D;

                    if (currentFrame.leftKeyPointCount == -1)
                    {
                        bool currentFrameIsUnprojected{};
                        if (currentFrame.unprojectStereo(
                                keyPointIndex,
                                x3D,
                                currentFrameIsUnprojected) !=
                            frame::FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: unprojectStereo returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                    }
                    else
                    {
                        Eigen::Vector3f currentFrameStereoFishEye{};
                        if (currentFrame.unprojectStereoFishEye(
                                keyPointIndex,
                                currentFrameStereoFishEye) !=
                            frame::FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: unprojectStereoFishEye returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        x3D = currentFrameStereoFishEye;
                    }

                    map::Map *p_atlasCurrentMap2 = nullptr;
                    if (p_atlas->getCurrentMap(p_atlasCurrentMap2) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCurrentMap returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    map_point::MapPoint *p_newMapPoint =
                        new map_point::MapPoint(x3D,
                                                p_keyFrame,
                                                p_atlasCurrentMap2);
                    if (p_newMapPoint->addObservation(p_keyFrame,
                                                      keyPointIndex) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    /*!
                     * Check if it is a stereo observation in order to not
                     * duplicate mappoints
                     */
                    if (currentFrame.leftKeyPointCount != -1 &&
                        currentFrame.leftToRightMatches[keyPointIndex] >= 0)
                    {
                        currentFrame.mapPoints
                            [currentFrame.leftKeyPointCount +
                             currentFrame.leftToRightMatches[keyPointIndex]] =
                            p_newMapPoint;
                        if (p_newMapPoint->addObservation(
                                p_keyFrame,
                                currentFrame.leftKeyPointCount +
                                    currentFrame
                                        .leftToRightMatches[keyPointIndex]) !=
                            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: addObservation returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (p_keyFrame->addMapPoint(
                                p_newMapPoint,
                                currentFrame.leftKeyPointCount +
                                    currentFrame
                                        .leftToRightMatches[keyPointIndex]) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: addMapPoint returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }

                    if (p_keyFrame->addMapPoint(p_newMapPoint, keyPointIndex) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_newMapPoint->computeDistinctiveDescriptors() !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDistinctiveDescriptors "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    if (p_newMapPoint->updateNormalAndDepth() !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    if (p_atlas->addMapPoint(p_newMapPoint) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    currentFrame.mapPoints[keyPointIndex] = p_newMapPoint;
                    pointCount++;
                }
                else
                {
                    pointCount++;
                }

                if (depthIndices[depthIndexIndex].first > depthThreshold &&
                    pointCount > maximumPoint)
                {
                    break;
                }
            }
        }
    }

    /*!
     * Check if the marker ids fromt he current frame exist in all the previous
     * keyframes first get the mapped marker from the keyframes
     */
    std::vector<semantic::Marker *> atlasAllMarkers{};
    if (p_atlas->getAllMarkers(atlasAllMarkers) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Marker *const p_currentMapMarker : atlasAllMarkers)
    {
        /* Check if the marker is already in the Global map */
        for (semantic::Marker *p_currentFrameMaker : currentFrame.mapMarkers)
        {
            int currentFrameMakerId{};
            if (p_currentFrameMaker->getId(currentFrameMakerId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int currentMapMarkerId{};
            if (p_currentMapMarker->getId(currentMapMarkerId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (currentFrameMakerId == currentMapMarkerId)
            {
                if (p_currentFrameMaker->setMarkerInGMap(true) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setMarkerInGMap returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    if (p_localMapper->insertKeyFrame(p_keyFrame) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: insertKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    bool localMapperWasSet2{};
    if (p_localMapper->setNotStop(false, localMapperWasSet2) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setNotStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    lastKeyFrameId = currentFrame.id;
    p_lastKeyFrame = p_keyFrame;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
