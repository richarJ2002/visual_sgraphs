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
 * @file            updateFrameIMU.cc
 *
 * @brief           Implements Tracking::updateFrameIMU(), declared in
 *                  Tracking.h.
 */

#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::updateFrameIMU(const float      s_in,
                                        const IMU::Bias &b_in,
                                        KeyFrame        *p_currentKeyFrame_in)
{
    Map *p_map = nullptr;
    if (p_currentKeyFrame_in->getMap(p_map) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::list<vs_graphs::core::KeyFrame *>::iterator rits =
        referenceKeyFrames.begin();
    std::list<bool>::iterator lbL = lostFlags.begin();
    for (std::list<Sophus::SE3f>::iterator lit  = relativeFramePoses.begin(),
                                           lend = relativeFramePoses.end();
         lit != lend;
         lit++, rits++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *p_keyFrame = *rits;

        for (;;)
        {
            bool keyFrameIsBad{};
            if (p_keyFrame->isBad(keyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            KeyFrame *p_keyFrameParent = nullptr;
            if ((keyFrameIsBad) && p_keyFrame->getParent(p_keyFrameParent) !=
                                       KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParent returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (!(keyFrameIsBad && p_keyFrameParent))
            {
                break;
            }
            KeyFrame *p_keyFrameParent2 = nullptr;
            if (p_keyFrame->getParent(p_keyFrameParent2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParent returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame = p_keyFrameParent2;
        }

        Map *p_keyFrameMap = nullptr;
        if (p_keyFrame->getMap(p_keyFrameMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrameMap == p_map)
        {
            (*lit).translation() *= s_in;
        }
    }

    lastBias = b_in;

    p_lastKeyFrame = p_currentKeyFrame_in;

    if (lastFrame.setNewBias(lastBias) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setNewBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentFrame.setNewBias(lastBias) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setNewBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (;;)
    {
        bool currentFrameIsImuPreintegrated{};
        if (currentFrame.isImuPreintegrated(currentFrameIsImuPreintegrated) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuPreintegrated returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrameIsImuPreintegrated)
        {
            break;
        }
        usleep(500);
    }

    if (lastFrame.id == lastFrame.p_lastKeyFrame->frameId)
    {
        Eigen::Matrix3f imuRotation{};
        if (lastFrame.p_lastKeyFrame->getImuRotation(imuRotation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f imuPosition{};
        if (lastFrame.p_lastKeyFrame->getImuPosition(imuPosition) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f lastKeyFrameVelocity{};
        if (lastFrame.p_lastKeyFrame->getVelocity(lastKeyFrameVelocity) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (lastFrame.setImuPoseVelocity(imuRotation,
                                         imuPosition,
                                         lastKeyFrameVelocity) !=
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
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        Eigen::Vector3f       bodyTranslation_body1ToWorld{};
        if (lastFrame.p_lastKeyFrame->getImuPosition(
                bodyTranslation_body1ToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body1ToWorld{};
        if (lastFrame.p_lastKeyFrame->getImuRotation(
                bodyRotation_body1ToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (lastFrame.p_lastKeyFrame->getVelocity(Vwb1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float t12 = lastFrame.p_imuPreintegrated->dT;

        Eigen::Matrix3f updatedDeltaRotation{};
        if (lastFrame.p_imuPreintegrated->getUpdatedDeltaRotation(
                updatedDeltaRotation) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaRotation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Matrix3f rotation{};
        if (IMU::normalizeRotation(
                bodyRotation_body1ToWorld * updatedDeltaRotation,
                rotation) != IMU::ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f updatedDeltaPosition{};
        if (lastFrame.p_imuPreintegrated->getUpdatedDeltaPosition(
                updatedDeltaPosition) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaPosition returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector3f updatedDeltaVelocity{};
        if (lastFrame.p_imuPreintegrated->getUpdatedDeltaVelocity(
                updatedDeltaVelocity) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaVelocity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (lastFrame.setImuPoseVelocity(
                rotation,
                bodyTranslation_body1ToWorld + Vwb1 * t12 +
                    0.5f * t12 * t12 * Gz +
                    bodyRotation_body1ToWorld * updatedDeltaPosition,
                Vwb1 + Gz * t12 +
                    bodyRotation_body1ToWorld * updatedDeltaVelocity) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (currentFrame.p_imuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        Eigen::Vector3f bodyTranslation_body1ToWorld{};
        if (currentFrame.p_lastKeyFrame->getImuPosition(
                bodyTranslation_body1ToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f bodyRotation_body1ToWorld{};
        if (currentFrame.p_lastKeyFrame->getImuRotation(
                bodyRotation_body1ToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vwb1{};
        if (currentFrame.p_lastKeyFrame->getVelocity(Vwb1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float t12 = currentFrame.p_imuPreintegrated->dT;

        Eigen::Matrix3f updatedDeltaRotation2{};
        if (currentFrame.p_imuPreintegrated->getUpdatedDeltaRotation(
                updatedDeltaRotation2) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaRotation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Matrix3f rotation2{};
        if (IMU::normalizeRotation(
                bodyRotation_body1ToWorld * updatedDeltaRotation2,
                rotation2) != IMU::ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f updatedDeltaPosition2{};
        if (currentFrame.p_imuPreintegrated->getUpdatedDeltaPosition(
                updatedDeltaPosition2) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaPosition returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector3f updatedDeltaVelocity2{};
        if (currentFrame.p_imuPreintegrated->getUpdatedDeltaVelocity(
                updatedDeltaVelocity2) !=
            IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getUpdatedDeltaVelocity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (currentFrame.setImuPoseVelocity(
                rotation2,
                bodyTranslation_body1ToWorld + Vwb1 * t12 +
                    0.5f * t12 * t12 * Gz +
                    bodyRotation_body1ToWorld * updatedDeltaPosition2,
                Vwb1 + Gz * t12 +
                    bodyRotation_body1ToWorld * updatedDeltaVelocity2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuPoseVelocity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
