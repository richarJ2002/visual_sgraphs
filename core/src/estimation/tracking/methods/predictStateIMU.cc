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
 * @file            predictStateIMU.cc
 *
 * @brief           Implements Tracking::predictStateIMU(), declared in
 *                  Tracking.h.
 */

#include "estimation/tracking/objects/TrackingClass.h"

#include <iostream>

#include <rclcpp/logging.hpp>

#include "map_state/frame/objects/FrameStatusEnum.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/objects/ImuConstants.h"
#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"
#include "sensor_models/imu_types/private_functions/private_functions.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"
#include "execution/system/system.h"

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::predictStateIMU(bool &isPredicted_out)
{
    if (!currentFrame.p_previousFrame)
    {
        if (system::Verbose::printMess("No last frame",
                                       system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isPredicted_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    if (isMapUpdated && p_lastKeyFrame)
    {
        Eigen::Vector3f bodyTranslation_body1ToWorld{};
        if (p_lastKeyFrame->getImuPosition(bodyTranslation_body1ToWorld) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body1ToWorld{};
        if (p_lastKeyFrame->getImuRotation(bodyRotation_body1ToWorld) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (p_lastKeyFrame->getVelocity(Vwb1) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const Eigen::Vector3f Gz(0,
                                 0,
                                 -imu_types::preintegrated::GRAVITY_VALUE);
        const float           t12 = p_imuPreintegratedFromLastKF->dT;

        imu_types::bias::Bias lastKeyFrameImuBias{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f imuPreintegratedFromLastKFDeltaRotation{};
        if (p_imuPreintegratedFromLastKF->getDeltaRotation(
                lastKeyFrameImuBias,
                imuPreintegratedFromLastKFDeltaRotation) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body2ToWorld{};
        if (imu_types::normalizeRotation(
                bodyRotation_body1ToWorld *
                    imuPreintegratedFromLastKFDeltaRotation,
                bodyRotation_body2ToWorld) !=
            imu_types::ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        imu_types::bias::Bias lastKeyFrameImuBias2{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f imuPreintegratedFromLastKFDeltaPosition{};
        if (p_imuPreintegratedFromLastKF->getDeltaPosition(
                lastKeyFrameImuBias2,
                imuPreintegratedFromLastKFDeltaPosition) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f bodyTranslation_body2ToWorld =
            bodyTranslation_body1ToWorld + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            bodyRotation_body1ToWorld * imuPreintegratedFromLastKFDeltaPosition;
        imu_types::bias::Bias lastKeyFrameImuBias3{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias3) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f imuPreintegratedFromLastKFDeltaVelocity{};
        if (p_imuPreintegratedFromLastKF->getDeltaVelocity(
                lastKeyFrameImuBias3,
                imuPreintegratedFromLastKFDeltaVelocity) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            bodyRotation_body1ToWorld * imuPreintegratedFromLastKFDeltaVelocity;
        if (currentFrame.setImuPoseVelocity(bodyRotation_body2ToWorld,
                                            bodyTranslation_body2ToWorld,
                                            Vwb2) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        imu_types::bias::Bias lastKeyFrameImuBias4{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias4) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        currentFrame.imuBias       = lastKeyFrameImuBias4;
        currentFrame.predictedBias = currentFrame.imuBias;
        isPredicted_out            = true;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
    else if (!isMapUpdated)
    {
        Eigen::Vector3f bodyTranslation_body1ToWorld{};
        if (lastFrame.getImuPosition(bodyTranslation_body1ToWorld) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body1ToWorld{};
        if (lastFrame.getImuRotation(bodyRotation_body1ToWorld) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (lastFrame.getVelocity(Vwb1) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3f Gz(0,
                                 0,
                                 -imu_types::preintegrated::GRAVITY_VALUE);
        const float           t12 = currentFrame.p_imuPreintegratedFrame->dT;

        Eigen::Matrix3f deltaRotation{};
        if (currentFrame.p_imuPreintegratedFrame->getDeltaRotation(
                lastFrame.imuBias,
                deltaRotation) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body2ToWorld{};
        if (imu_types::normalizeRotation(bodyRotation_body1ToWorld *
                                             deltaRotation,
                                         bodyRotation_body2ToWorld) !=
            imu_types::ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f deltaPosition{};
        if (currentFrame.p_imuPreintegratedFrame->getDeltaPosition(
                lastFrame.imuBias,
                deltaPosition) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f bodyTranslation_body2ToWorld =
            bodyTranslation_body1ToWorld + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            bodyRotation_body1ToWorld * deltaPosition;
        Eigen::Vector3f deltaVelocity{};
        if (currentFrame.p_imuPreintegratedFrame->getDeltaVelocity(
                lastFrame.imuBias,
                deltaVelocity) !=
            imu_types::preintegrated::PreintegratedStatus::
                PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDeltaVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz + bodyRotation_body1ToWorld * deltaVelocity;

        if (currentFrame.setImuPoseVelocity(bodyRotation_body2ToWorld,
                                            bodyTranslation_body2ToWorld,
                                            Vwb2) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        currentFrame.imuBias       = lastFrame.imuBias;
        currentFrame.predictedBias = currentFrame.imuBias;
        isPredicted_out            = true;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
    else
        std::cout << "not IMU prediction!!" << std::endl;

    isPredicted_out = false;
    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
