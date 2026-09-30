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

#include "Tracking.h"

#include "LocalMapping.h"
#include "Optimizer.h"
#include "ResetCause.h"
#include "System.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::createInitialMapMonocular()
{
    // Create KeyFrames
    Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    KeyFrame *p_keyFrameInitial =
        new KeyFrame(initialFrame, p_atlasCurrentMap, p_keyFrameDatabase);
    Map *p_atlasCurrentMap2 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap2) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    KeyFrame *p_keyFrameCurrent =
        new KeyFrame(currentFrame, p_atlasCurrentMap2, p_keyFrameDatabase);

    if (sensor == System::IMU_MONOCULAR)
        p_keyFrameInitial->p_imuPreintegrated = nullptr;

    if (p_keyFrameInitial->computeBagOfWords() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameCurrent->computeBagOfWords() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Insert KFs in the map
    if (p_atlas->addKeyFrame(p_keyFrameInitial) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlas->addKeyFrame(p_keyFrameCurrent) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
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

        // Create MapPoint.
        Eigen::Vector3f worldPosition;
        worldPosition << iniP3D[initialMatchIndex].x,
            iniP3D[initialMatchIndex].y, iniP3D[initialMatchIndex].z;
        Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), worldPosition);
        Sophus::SE3f Twmp       = poseTc0w.inverse() * Tc0mp;
        worldPosition           = Twmp.translation();
        Map *p_atlasCurrentMap3 = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap3) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        MapPoint *p_mapPoint =
            new MapPoint(worldPosition, p_keyFrameCurrent, p_atlasCurrentMap3);

        if (p_keyFrameInitial->addMapPoint(p_mapPoint, initialMatchIndex) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrameCurrent->addMapPoint(p_mapPoint,
                                           iniMatches[initialMatchIndex]) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->addObservation(p_keyFrameInitial, initialMatchIndex) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addObservation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint->addObservation(p_keyFrameCurrent,
                                       iniMatches[initialMatchIndex]) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addObservation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->computeDistinctiveDescriptors() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeDistinctiveDescriptors returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_mapPoint->updateNormalAndDepth() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        // Fill Current Frame structure
        currentFrame.mapPoints[iniMatches[initialMatchIndex]]    = p_mapPoint;
        currentFrame.outlierFlags[iniMatches[initialMatchIndex]] = false;

        // Add to Map
        if (p_atlas->addMapPoint(p_mapPoint) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Update Connections
    if (p_keyFrameInitial->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrameCurrent->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::set<MapPoint *> mapPoints;
    std::set<MapPoint *> keyFrameInitialMapPoints{};
    if (p_keyFrameInitial->getMapPoints(keyFrameInitialMapPoints) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPoints returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    mapPoints = keyFrameInitialMapPoints;

    // Bundle Adjustment
    std::cout << "\n[Tracking]" << std::endl;
    unsigned long atlasMapPointCount{};
    if (p_atlas->getMapPointCount(atlasMapPointCount) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "- New map created with #"
              << std::to_string(atlasMapPointCount) << " points!" << std::endl;
    Map *p_atlasCurrentMap4 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap4) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (Optimizer::globalBundleAdjustment(p_atlasCurrentMap4,
                                          20,
                                          nullptr,
                                          0,
                                          true) !=
        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: globalBundleAdjustment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    float medianDepth{};
    if (p_keyFrameInitial->computeSceneMedianDepth(2, medianDepth) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeSceneMedianDepth returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    float invMedianDepth;
    if (sensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;
    else
        invMedianDepth = 1.0f / medianDepth;

    int keyFrameCurrentTrackedMapPointCount{};
    if (!(medianDepth < 0) && p_keyFrameCurrent->getTrackedMapPointCount(
                                  1,
                                  keyFrameCurrentTrackedMapPointCount) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTrackedMapPointCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (medianDepth < 0 ||
        keyFrameCurrentTrackedMapPointCount < 50) // TODO Check, originally 100
                                                  // tracks
    {
        if (Verbose::printMess("Wrong initialization, reseting...",
                               Verbose::VERBOSITY_QUIET) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_system->requestResetActiveMapWithCause(
                ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP) !=
            SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: requestResetActiveMapWithCause returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w{};
    if (p_keyFrameCurrent->getPose(Tc2w) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Tc2w.translation() *= invMedianDepth;
    if (p_keyFrameCurrent->setPose(Tc2w) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    // Scale points
    std::vector<MapPoint *> allMapPoints{};
    if (p_keyFrameInitial->getMapPointMatches(allMapPoints) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            MapPoint       *p_mapPoint = allMapPoints[mapPointIndex];
            Eigen::Vector3f mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->setWorldPos(mapPointWorldPos * invMedianDepth) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->updateNormalAndDepth() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    if (sensor == System::IMU_MONOCULAR)
    {
        p_keyFrameCurrent->p_prevKF           = p_keyFrameInitial;
        p_keyFrameInitial->p_nextKF           = p_keyFrameCurrent;
        p_keyFrameCurrent->p_imuPreintegrated = p_imuPreintegratedFromLastKF;

        IMU::Bias updatedBias{};
        if (p_keyFrameCurrent->p_imuPreintegrated->getUpdatedBias(
                updatedBias) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getUpdatedBias returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(updatedBias,
                                   p_keyFrameCurrent->imuCalibration);
    }

    if (p_localMapper->insertKeyFrame(p_keyFrameInitial) !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: insertKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_localMapper->insertKeyFrame(p_keyFrameCurrent) !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: insertKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_localMapper->firstTimestamp = p_keyFrameCurrent->timeStamp;

    Sophus::SE3f keyFrameCurrentPose{};
    if (p_keyFrameCurrent->getPose(keyFrameCurrentPose) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (currentFrame.setPose(keyFrameCurrentPose) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    lastKeyFrameId = currentFrame.id;
    p_lastKeyFrame = p_keyFrameCurrent;
    // mnLastRelocFrameId = mInitialFrame.id;

    localKeyFrames.push_back(p_keyFrameCurrent);
    localKeyFrames.push_back(p_keyFrameInitial);
    std::vector<MapPoint *> atlasAllMapPoints{};
    if (p_atlas->getAllMapPoints(atlasAllMapPoints) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    localMapPoints                   = atlasAllMapPoints;
    p_referenceKF                    = p_keyFrameCurrent;
    currentFrame.p_referenceKeyFrame = p_keyFrameCurrent;

    // Compute here initial velocity
    std::vector<KeyFrame *> keyFrames{};
    if (p_atlas->getAllKeyFrames(keyFrames) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    Sophus::SE3f pose{};
    if (keyFrames.back()->getPose(pose) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f poseInverse{};
    if (keyFrames.front()->getPoseInverse(poseInverse) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

    lastFrame = Frame(currentFrame);

    if (p_atlas->setReferenceMapPoints(localMapPoints) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setReferenceMapPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    Sophus::SE3f keyFrameCurrentPose2{};
    if (p_keyFrameCurrent->getPose(keyFrameCurrentPose2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_mapDrawer->setCurrentCameraPose(keyFrameCurrentPose2) !=
        MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCurrentCameraPose returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    Map *p_atlasCurrentMap5 = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap5) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
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

} // namespace core
} // namespace vs_graphs
