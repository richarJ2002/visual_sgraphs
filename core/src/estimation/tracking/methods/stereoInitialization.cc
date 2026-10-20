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
 * @file            stereoInitialization.cc
 *
 * @brief           Implements Tracking::stereoInitialization(), declared in
 *                  Tracking.h.
 */

#include "estimation/tracking/objects/TrackingClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/frame/frame.h"
#include "map_state/frame/objects/FrameStatusEnum.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/preintegrated.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "map_state/map/map.h"
#include "views_io/map_drawer/objects/MapDrawerStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "execution/reset_cause/objects/ResetCauseEnum.h"
#include "execution/system/objects/SystemStatusEnum.h"

#include "execution/reset_cause/reset_cause.h"
#include "mapping/local_mapping/local_mapping.h"
#include "execution/system/system.h"

#include <iomanip>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

/* map::Map initialization for Stereo and RGB-D (with/without IMU) setups */
TrackingStatus Tracking::stereoInitialization()
{
    /* Require more points for robust initialization in corridors */
    if (currentFrame.keyPointCount > initializationMinPoints)
    {
        if (sensor == system::System::IMU_STEREO ||
            sensor == system::System::IMU_RGBD)
        {
            if (!currentFrame.p_imuPreintegrated ||
                !lastFrame.p_imuPreintegrated)
            {
                std::cout << "[Tracking] IMU measurements are not available "
                             "for the current frame!"
                          << std::endl;
                return TrackingStatus::TRACKING_STATUS_SUCCESS;
            }

            /* Check acceleration difference for fast initialization */
            if (!isFastInitEnabled)
            {
                const double accelDiff =
                    (currentFrame.p_imuPreintegratedFrame->avgA -
                     lastFrame.p_imuPreintegratedFrame->avgA)
                        .norm();

                if (accelDiff < imuThresh)
                {
                    std::cout << "[Tracking] Low IMU acceleration changes: "
                              << std::fixed << std::setprecision(2) << accelDiff
                              << " (threshold: " << imuThresh
                              << ")! Skipping ..." << std::endl;
                    return TrackingStatus::TRACKING_STATUS_SUCCESS;
                }
            }

            if (p_imuPreintegratedFromLastKF)
                delete p_imuPreintegratedFromLastKF;

            /* Reset IMU preintegration from last keyframe */
            p_imuPreintegratedFromLastKF =
                new imu_types::preintegrated::Preintegrated(imu_types::bias::Bias(), *p_imuCalibration);
            currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
        }

        /* Set frame::Frame pose to the origin */
        if (sensor == system::System::IMU_STEREO ||
            sensor == system::System::IMU_RGBD)
        {
            Eigen::Matrix3f bodyRotation_body0ToWorld =
                currentFrame.imuCalibration.mTcb.rotationMatrix();
            Eigen::Vector3f bodyTranslation_body0ToWorld =
                currentFrame.imuCalibration.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            if (currentFrame.setImuPoseVelocity(bodyRotation_body0ToWorld,
                                                bodyTranslation_body0ToWorld,
                                                Vwb0) !=
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setImuPoseVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            if (currentFrame.setPose(Sophus::SE3f()) !=
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /* Create keyframe::KeyFrame */
        map::Map *p_atlasCurrentMap = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::keyframe::KeyFrame *p_keyFrameInitial =
            new vs_graphs::core::keyframe::KeyFrame(currentFrame,
                                                    p_atlasCurrentMap,
                                                    p_keyFrameDatabase);

        /* Insert keyframe::KeyFrame in the map */
        if (p_atlas->addKeyFrame(p_keyFrameInitial) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addKeyFrame returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Create MapPoints and asscoiate to keyframe::KeyFrame */
        int pointsCreatedCount = 0;
        if (!p_camera2)
        {
            for (int keyPointIndex = 0;
                 keyPointIndex < currentFrame.keyPointCount;
                 keyPointIndex++)
            {
                float z = currentFrame.depths[keyPointIndex];
                if (z > 0)
                {
                    Eigen::Vector3f x3D;
                    bool            currentFrameIsUnprojected{};
                    if (currentFrame.unprojectStereo(
                            keyPointIndex,
                            x3D,
                            currentFrameIsUnprojected) !=
                        frame::FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: unprojectStereo returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
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
                                                p_keyFrameInitial,
                                                p_atlasCurrentMap2);
                    if (p_newMapPoint->addObservation(p_keyFrameInitial,
                                                      keyPointIndex) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrameInitial->addMapPoint(p_newMapPoint,
                                                       keyPointIndex) !=
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
                    pointsCreatedCount++;
                }
            }
        }
        else
        {
            for (int keyPointIndex = 0;
                 keyPointIndex < currentFrame.leftKeyPointCount;
                 keyPointIndex++)
            {
                int rightIndex = currentFrame.leftToRightMatches[keyPointIndex];
                if (rightIndex != -1)
                {
                    Eigen::Vector3f x3D =
                        currentFrame.stereoPoints3D[keyPointIndex];

                    map::Map *p_atlasCurrentMap3 = nullptr;
                    if (p_atlas->getCurrentMap(p_atlasCurrentMap3) !=
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
                                                p_keyFrameInitial,
                                                p_atlasCurrentMap3);

                    if (p_newMapPoint->addObservation(p_keyFrameInitial,
                                                      keyPointIndex) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_newMapPoint->addObservation(
                            p_keyFrameInitial,
                            rightIndex + currentFrame.leftKeyPointCount) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (p_keyFrameInitial->addMapPoint(p_newMapPoint,
                                                       keyPointIndex) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrameInitial->addMapPoint(
                            p_newMapPoint,
                            rightIndex + currentFrame.leftKeyPointCount) !=
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
                    currentFrame.mapPoints[rightIndex +
                                           currentFrame.leftKeyPointCount] =
                        p_newMapPoint;
                    pointsCreatedCount++;
                }
            }
        }

        unsigned long atlasMapPointCount{};
        if (p_atlas->getMapPointCount(atlasMapPointCount) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "\n[Tracking] New map created with #" +
                         std::to_string(atlasMapPointCount) + " points!"
                  << std::endl;

        /* Require minimum points for successful initialization */
        if (pointsCreatedCount < initializationMinPoints)
        {
            std::cout << "[Tracking] Insufficient points for initialization ("
                      << pointsCreatedCount << " < " << initializationMinPoints
                      << "), resetting..." << std::endl;
            if (p_system->requestResetActiveMapWithCause(
                    reset_cause::ResetCause::INITIALIZATION_INSUFFICIENT_POINTS) !=
                system::SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: requestResetActiveMapWithCause returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        if (p_localMapper->insertKeyFrame(p_keyFrameInitial) !=
            local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: insertKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        lastFrame      = frame::Frame(currentFrame);
        lastKeyFrameId = currentFrame.id;
        p_lastKeyFrame = p_keyFrameInitial;

        localKeyFrames.push_back(p_keyFrameInitial);
        std::vector<map_point::MapPoint *> atlasAllMapPoints{};
        if (p_atlas->getAllMapPoints(atlasAllMapPoints) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        localMapPoints                   = atlasAllMapPoints;
        p_referenceKF                    = p_keyFrameInitial;
        currentFrame.p_referenceKeyFrame = p_keyFrameInitial;

        if (p_atlas->setReferenceMapPoints(localMapPoints) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setReferenceMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        map::Map *p_atlasCurrentMap4 = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap4) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_atlasCurrentMap4->keyFrameOrigins.push_back(p_keyFrameInitial);

        Sophus::SE3<float> currentFrameGetPose{};
        if (currentFrame.getPose(currentFrameGetPose) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapDrawer->setCurrentCameraPose(currentFrameGetPose) !=
            map_drawer::MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCurrentCameraPose returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        state = OK;
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
