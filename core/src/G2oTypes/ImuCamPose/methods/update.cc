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
 * @file            update.cc
 *
 * @brief           Implements ImuCamPose::update(), declared in G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ImuCamPoseStatus ImuCamPose::update(const double *p_updateVector_in)
{
    Eigen::Vector3d rotationUpdate, translationUpdate;
    rotationUpdate << p_updateVector_in[0], p_updateVector_in[1],
        p_updateVector_in[2];
    translationUpdate << p_updateVector_in[3], p_updateVector_in[4],
        p_updateVector_in[5];

    // Update body pose
    twb += Rwb * translationUpdate;
    Eigen::Matrix3d rotation2{};
    if (expSO3(rotationUpdate, rotation2) !=
        G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: expSO3 returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Rwb = Rwb * rotation2;

    // Normalize rotation after 5 updates
    its++;
    if (its >= 3)
    {
        Eigen::Matrix<double, 3, 3> rotation3{};
        if (normalizeRotation(Rwb, rotation3) !=
            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        its = 0;
    }

    // Update camera poses
    const Eigen::Matrix3d rotation_worldToBody    = Rwb.transpose();
    const Eigen::Vector3d translation_worldToBody = -rotation_worldToBody * twb;

    for (std::size_t cameraIndex = 0; cameraIndex < pCamera.size();
         cameraIndex++)
    {
        Rcw[cameraIndex] = Rcb[cameraIndex] * rotation_worldToBody;
        tcw[cameraIndex] =
            Rcb[cameraIndex] * translation_worldToBody + tcb[cameraIndex];
    }

    return ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
