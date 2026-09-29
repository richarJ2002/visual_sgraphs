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

#include "Tracking.h"

#include "LocalMapping.h"
#include "ResetCause.h"
#include "System.h"

#include <iomanip>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

// Map initialization for Stereo and RGB-D (with/without IMU) setups
void Tracking::stereoInitialization()
{
    // Require more points for robust initialization in corridors
    if (currentFrame.keyPointCount > initializationMinPoints)
    {
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            if (!currentFrame.p_imuPreintegrated ||
                !lastFrame.p_imuPreintegrated)
            {
                std::cout << "[Tracking] IMU measurements are not available "
                             "for the current frame!"
                          << std::endl;
                return;
            }

            // Check acceleration difference for fast initialization
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
                    return;
                }
            }

            if (p_imuPreintegratedFromLastKF)
                delete p_imuPreintegratedFromLastKF;

            // Reset IMU preintegration from last keyframe
            p_imuPreintegratedFromLastKF =
                new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
            currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 =
                currentFrame.imuCalibration.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 =
                currentFrame.imuCalibration.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            if (currentFrame.setImuPoseVelocity(Rwb0, twb0, Vwb0) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
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
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }

        // Create KeyFrame
        vs_graphs::core::KeyFrame *p_keyFrameInitial =
            new vs_graphs::core::KeyFrame(currentFrame,
                                          p_atlas->getCurrentMap(),
                                          p_keyFrameDatabase);

        // Insert KeyFrame in the map
        p_atlas->addKeyFrame(p_keyFrameInitial);

        // Create MapPoints and asscoiate to KeyFrame
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
                        FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: unprojectStereo returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    MapPoint *p_newMapPoint =
                        new MapPoint(x3D,
                                     p_keyFrameInitial,
                                     p_atlas->getCurrentMap());
                    if (p_newMapPoint->addObservation(p_keyFrameInitial,
                                                      keyPointIndex) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrameInitial->addMapPoint(p_newMapPoint,
                                                       keyPointIndex) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_newMapPoint->computeDistinctiveDescriptors() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDistinctiveDescriptors "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    if (p_newMapPoint->updateNormalAndDepth() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    p_atlas->addMapPoint(p_newMapPoint);

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

                    MapPoint *p_newMapPoint =
                        new MapPoint(x3D,
                                     p_keyFrameInitial,
                                     p_atlas->getCurrentMap());

                    if (p_newMapPoint->addObservation(p_keyFrameInitial,
                                                      keyPointIndex) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (p_keyFrameInitial->addMapPoint(p_newMapPoint,
                                                       keyPointIndex) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (p_newMapPoint->computeDistinctiveDescriptors() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDistinctiveDescriptors "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    if (p_newMapPoint->updateNormalAndDepth() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: updateNormalAndDepth returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    p_atlas->addMapPoint(p_newMapPoint);

                    currentFrame.mapPoints[keyPointIndex] = p_newMapPoint;
                    currentFrame.mapPoints[rightIndex +
                                           currentFrame.leftKeyPointCount] =
                        p_newMapPoint;
                    pointsCreatedCount++;
                }
            }
        }

        std::cout << "\n[Tracking] New map created with #" +
                         to_string(p_atlas->getMapPointCount()) + " points!"
                  << std::endl;

        // Require minimum points for successful initialization
        if (pointsCreatedCount < initializationMinPoints)
        {
            std::cout << "[Tracking] Insufficient points for initialization ("
                      << pointsCreatedCount << " < " << initializationMinPoints
                      << "), resetting..." << std::endl;
            p_system->requestResetActiveMapWithCause(
                ResetCause::INITIALIZATION_INSUFFICIENT_POINTS);
            return;
        }

        p_localMapper->insertKeyFrame(p_keyFrameInitial);

        lastFrame      = Frame(currentFrame);
        lastKeyFrameId = currentFrame.id;
        p_lastKeyFrame = p_keyFrameInitial;

        localKeyFrames.push_back(p_keyFrameInitial);
        localMapPoints                   = p_atlas->getAllMapPoints();
        p_referenceKF                    = p_keyFrameInitial;
        currentFrame.p_referenceKeyFrame = p_keyFrameInitial;

        p_atlas->setReferenceMapPoints(localMapPoints);

        p_atlas->getCurrentMap()->keyFrameOrigins.push_back(p_keyFrameInitial);

        Sophus::SE3<float> currentFrameGetPose{};
        if (currentFrame.getPose(currentFrameGetPose) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_mapDrawer->setCurrentCameraPose(currentFrameGetPose);

        state = OK;
    }
}

} // namespace core
} // namespace vs_graphs
