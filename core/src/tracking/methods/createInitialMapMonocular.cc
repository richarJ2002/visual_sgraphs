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
 * @file            createInitialMapMonocular.cc
 *
 * @brief           Implements Tracking::createInitialMapMonocular(), declared
 *                  in Tracking.h.
 */

#include "tracking/objects/TrackingClass.h"
#include "atlas/objects/AtlasStatusEnum.h"
#include "frame/frame.h"
#include "frame/objects/FrameStatusEnum.h"
#include "imu_types/bias/bias.h"
#include "imu_types/preintegrated/objects/PreintegratedStatusEnum.h"
#include "imu_types/preintegrated/preintegrated.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "local_mapping/objects/LocalMappingStatusEnum.h"
#include "map/map.h"
#include "map_drawer/objects/MapDrawerStatusEnum.h"
#include "map_point/map_point.h"
#include "map_point/objects/MapPointStatusEnum.h"
#include "optimizer/objects/OptimizerStatusEnum.h"
#include "reset_cause/objects/ResetCauseEnum.h"
#include "system/objects/SystemStatusEnum.h"
#include "system/objects/VerboseStatusEnum.h"

#include "reset_cause/reset_cause.h"
#include "local_mapping/local_mapping.h"
#include "optimizer/optimizer.h"
#include "system/system.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::createInitialMapMonocular()
{
    /* Create KeyFrames */
    map::Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    keyframe::KeyFrame *p_keyFrameInitial =
        new keyframe::KeyFrame(initialFrame,
                               p_atlasCurrentMap,
                               p_keyFrameDatabase);
    map::Map *p_atlasCurrentMap2 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap2) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    keyframe::KeyFrame *p_keyFrameCurrent =
        new keyframe::KeyFrame(currentFrame,
                               p_atlasCurrentMap2,
                               p_keyFrameDatabase);

    if (sensor == system::System::IMU_MONOCULAR)
        p_keyFrameInitial->p_imuPreintegrated = nullptr;

    if (p_keyFrameInitial->computeBagOfWords() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameCurrent->computeBagOfWords() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Insert KFs in the map */
    if (p_atlas->addKeyFrame(p_keyFrameInitial) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlas->addKeyFrame(p_keyFrameCurrent) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (size_t initialMatchIndex = 0; initialMatchIndex < iniMatches.size();
         initialMatchIndex++)
    {
        if (iniMatches[initialMatchIndex] < 0)
            continue;

        /* Create map_point::MapPoint. */
        Eigen::Vector3f worldPosition;
        worldPosition << iniP3D[initialMatchIndex].x,
            iniP3D[initialMatchIndex].y, iniP3D[initialMatchIndex].z;
        Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), worldPosition);
        Sophus::SE3f Twmp            = poseTc0w.inverse() * Tc0mp;
        worldPosition                = Twmp.translation();
        map::Map *p_atlasCurrentMap3 = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap3) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        map_point::MapPoint *p_mapPoint =
            new map_point::MapPoint(worldPosition,
                                    p_keyFrameCurrent,
                                    p_atlasCurrentMap3);

        if (p_keyFrameInitial->addMapPoint(p_mapPoint, initialMatchIndex) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrameCurrent->addMapPoint(p_mapPoint,
                                           iniMatches[initialMatchIndex]) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->addObservation(p_keyFrameInitial, initialMatchIndex) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addObservation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint->addObservation(p_keyFrameCurrent,
                                       iniMatches[initialMatchIndex]) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addObservation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->computeDistinctiveDescriptors() !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeDistinctiveDescriptors returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_mapPoint->updateNormalAndDepth() !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Fill Current frame::Frame structure */
        currentFrame.mapPoints[iniMatches[initialMatchIndex]]    = p_mapPoint;
        currentFrame.outlierFlags[iniMatches[initialMatchIndex]] = false;

        /* Add to map::Map */
        if (p_atlas->addMapPoint(p_mapPoint) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Update Connections */
    if (p_keyFrameInitial->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameCurrent->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::set<map_point::MapPoint *> mapPoints;
    std::set<map_point::MapPoint *> keyFrameInitialMapPoints{};
    if (p_keyFrameInitial->getMapPoints(keyFrameInitialMapPoints) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPoints returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    mapPoints = keyFrameInitialMapPoints;

    /* Bundle Adjustment */
    std::cout << "\n[Tracking]" << std::endl;
    unsigned long atlasMapPointCount{};
    if (p_atlas->getMapPointCount(atlasMapPointCount) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "- New map created with #"
              << std::to_string(atlasMapPointCount) << " points!" << std::endl;
    map::Map *p_atlasCurrentMap4 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap4) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (optimizer::Optimizer::globalBundleAdjustment(p_atlasCurrentMap4,
                                          20,
                                          nullptr,
                                          0,
                                          true) !=
        optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: globalBundleAdjustment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    float medianDepth{};
    if (p_keyFrameInitial->computeSceneMedianDepth(2, medianDepth) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        /* No scene depth: the negative value takes the reset path below. */
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the initial key frame has no scene depth; "
                     "resetting the initialisation.",
                     __func__);
        medianDepth = -1.0F;
    }
    float invMedianDepth;
    if (sensor == system::System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;
    else
        invMedianDepth = 1.0f / medianDepth;

    int keyFrameCurrentTrackedMapPointCount{};
    if (!(medianDepth < 0) && p_keyFrameCurrent->getTrackedMapPointCount(
                                  1,
                                  keyFrameCurrentTrackedMapPointCount) !=
                                  keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTrackedMapPointCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    /* TODO Check, originally 100 tracks */
    if (medianDepth < 0 || keyFrameCurrentTrackedMapPointCount < 50)
    {
        if (system::Verbose::printMess("Wrong initialization, reseting...",
                               system::Verbose::VERBOSITY_QUIET) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_system->requestResetActiveMapWithCause(
                reset_cause::ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP) !=
            system::SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: requestResetActiveMapWithCause returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    /* Scale initial baseline */
    Sophus::SE3f Tc2w{};
    if (p_keyFrameCurrent->getPose(Tc2w) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Tc2w.translation() *= invMedianDepth;
    if (p_keyFrameCurrent->setPose(Tc2w) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Scale points */
    std::vector<map_point::MapPoint *> allMapPoints{};
    if (p_keyFrameInitial->getMapPointMatches(allMapPoints) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (size_t mapPointIndex = 0; mapPointIndex < allMapPoints.size();
         mapPointIndex++)
    {
        if (allMapPoints[mapPointIndex])
        {
            map_point::MapPoint *p_mapPoint = allMapPoints[mapPointIndex];
            Eigen::Vector3f      mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->setWorldPos(mapPointWorldPos * invMedianDepth) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->updateNormalAndDepth() !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    if (sensor == system::System::IMU_MONOCULAR)
    {
        p_keyFrameCurrent->p_prevKF           = p_keyFrameInitial;
        p_keyFrameInitial->p_nextKF           = p_keyFrameCurrent;
        p_keyFrameCurrent->p_imuPreintegrated = p_imuPreintegratedFromLastKF;

        imu_types::bias::Bias updatedBias{};
        if (p_keyFrameCurrent->p_imuPreintegrated->getUpdatedBias(
                updatedBias) !=
            imu_types::preintegrated::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getUpdatedBias returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_imuPreintegratedFromLastKF =
            new imu_types::preintegrated::Preintegrated(updatedBias,
                                   p_keyFrameCurrent->imuCalibration);
    }

    if (p_localMapper->insertKeyFrame(p_keyFrameInitial) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: insertKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_localMapper->insertKeyFrame(p_keyFrameCurrent) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: insertKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_localMapper->firstTimestamp = p_keyFrameCurrent->timeStamp;

    Sophus::SE3f keyFrameCurrentPose{};
    if (p_keyFrameCurrent->getPose(keyFrameCurrentPose) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (currentFrame.setPose(keyFrameCurrentPose) !=
        frame::FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    lastKeyFrameId = currentFrame.id;
    p_lastKeyFrame = p_keyFrameCurrent;

    localKeyFrames.push_back(p_keyFrameCurrent);
    localKeyFrames.push_back(p_keyFrameInitial);
    std::vector<map_point::MapPoint *> atlasAllMapPoints{};
    if (p_atlas->getAllMapPoints(atlasAllMapPoints) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    localMapPoints                   = atlasAllMapPoints;
    p_referenceKF                    = p_keyFrameCurrent;
    currentFrame.p_referenceKeyFrame = p_keyFrameCurrent;

    /* Compute here initial velocity */
    std::vector<keyframe::KeyFrame *> keyFrames{};
    if (p_atlas->getAllKeyFrames(keyFrames) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    Sophus::SE3f pose{};
    if (keyFrames.back()->getPose(pose) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f poseInverse{};
    if (keyFrames.front()->getPoseInverse(poseInverse) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f deltaT = pose * poseInverse;
    isVelocityAvailable = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (currentFrame.timeStamp - lastFrame.timeStamp) /
                 (currentFrame.timeStamp - initialFrame.timeStamp);
    phi *= aux;

    lastFrame = frame::Frame(currentFrame);

    if (p_atlas->setReferenceMapPoints(localMapPoints) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setReferenceMapPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    Sophus::SE3f keyFrameCurrentPose2{};
    if (p_keyFrameCurrent->getPose(keyFrameCurrentPose2) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_mapDrawer->setCurrentCameraPose(keyFrameCurrentPose2) !=
        map_drawer::MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCurrentCameraPose returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    map::Map *p_atlasCurrentMap5 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap5) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_atlasCurrentMap5->keyFrameOrigins.push_back(p_keyFrameInitial);

    state = OK;

    initId = p_keyFrameCurrent->id;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
