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

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

G2oTypesStatus expSO3(const double     angleAxisX_in,
                      const double     angleAxisY_in,
                      const double     angleAxisZ_in,
                      Eigen::Matrix3d &rotation_out)
{
    const double angleSquared = angleAxisX_in * angleAxisX_in +
                                angleAxisY_in * angleAxisY_in +
                                angleAxisZ_in * angleAxisZ_in;
    const double    angle = sqrt(angleSquared);
    Eigen::Matrix3d skewMatrix;
    skewMatrix << 0.0, -angleAxisZ_in, angleAxisY_in, angleAxisZ_in, 0.0,
        -angleAxisX_in, -angleAxisY_in, angleAxisX_in, 0.0;
    if (angle < 1e-5)
    {
        Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity() +
                                         skewMatrix +
                                         0.5 * skewMatrix * skewMatrix;
        Eigen::Matrix<double, 3, 3> rotation{};
        if (normalizeRotation(rotationMatrix, rotation) !=
            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        rotation_out = rotation;
        return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
    }
    else
    {
        Eigen::Matrix3d rotationMatrix =
            Eigen::Matrix3d::Identity() + skewMatrix * sin(angle) / angle +
            skewMatrix * skewMatrix * (1.0 - cos(angle)) / angleSquared;
        Eigen::Matrix<double, 3, 3> rotation2{};
        if (normalizeRotation(rotationMatrix, rotation2) !=
            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: normalizeRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        rotation_out = rotation2;
        return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
