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

#include "System.h"
#include "Tracking.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool Tracking::predictStateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        Verbose::printMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if (isMapUpdated && p_lastKeyFrame)
    {
        Eigen::Vector3f twb1{};
        if (p_lastKeyFrame->getImuPosition(twb1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f Rwb1{};
        if (p_lastKeyFrame->getImuRotation(Rwb1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (p_lastKeyFrame->getVelocity(Vwb1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = p_imuPreintegratedFromLastKF->dT;

        IMU::Bias lastKeyFrameImuBias{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f Rwb2 = IMU::normalizeRotation(
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaRotation(
                       lastKeyFrameImuBias));
        IMU::Bias lastKeyFrameImuBias2{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaPosition(
                       lastKeyFrameImuBias2);
        IMU::Bias lastKeyFrameImuBias3{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias3) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaVelocity(
                       lastKeyFrameImuBias3);
        if (currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        IMU::Bias lastKeyFrameImuBias4{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias4) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        currentFrame.imuBias       = lastKeyFrameImuBias4;
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else if (!isMapUpdated)
    {
        Eigen::Vector3f twb1{};
        if (lastFrame.getImuPosition(twb1) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f Rwb1{};
        if (lastFrame.getImuRotation(Rwb1) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (lastFrame.getVelocity(Vwb1) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = currentFrame.p_imuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::normalizeRotation(
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaRotation(
                       lastFrame.imuBias));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaPosition(
                       lastFrame.imuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaVelocity(
                       lastFrame.imuBias);

        if (currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        currentFrame.imuBias       = lastFrame.imuBias;
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

} // namespace core
} // namespace vs_graphs
