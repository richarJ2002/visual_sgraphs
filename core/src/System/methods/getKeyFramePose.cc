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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            getKeyFramePose.cc
 *
 * @brief           Implements System::getKeyFramePose(), declared in System.h.
 */

#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::getKeyFramePose(KeyFrame     *p_keyFrame_in,
                                     Sophus::SE3f &keyFramePose_out)
{
    bool keyFrameIsBad{};
    if (p_keyFrame_in->isBad(keyFrameIsBad) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (keyFrameIsBad)
    {
        keyFramePose_out = Sophus::SE3f();
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    // The "body" is the IMU when the sensor has one, otherwise the first
    // camera (cam0).
    Sophus::SE3f pose_bodyToWorld;
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
        sensor == IMU_RGBD) // with IMU
    {
        Sophus::SE3f keyFrameImuPose{};
        if (p_keyFrame_in->getImuPose(keyFrameImuPose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        pose_bodyToWorld = keyFrameImuPose;
    }
    else // without IMU
    {
        Sophus::SE3f keyFramePoseInverse{};
        if (p_keyFrame_in->getPoseInverse(keyFramePoseInverse) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        pose_bodyToWorld = keyFramePoseInverse;
    }

    keyFramePose_out = pose_bodyToWorld;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
