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
 * @file            getHessian3.cc
 *
 * @brief           Implements EdgeInertialGS::getHessian3().
 */

#include "g2o_types/edge_inertial_gs/objects/EdgeInertialGSClass.h"

#include <Eigen/Dense>

#include <rclcpp/logging.hpp>

#include "g2o_types/imu_cam_pose/imu_cam_pose.h"
#include "g2o_types/vertex_pose/vertex_pose.h"

#include "g2o_types/edge_inertial_gs/objects/EdgeInertialGSStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_inertial_gs
{

[[nodiscard]] EdgeInertialGSStatus
    EdgeInertialGS::getHessian3(Eigen::Matrix<double, 9, 9> &hessian3_out)
{
    linearizeOplus();
    Eigen::Matrix<double, 9, 9> J;
    J.block<9, 3>(0, 0) = _jacobianOplus[2];
    J.block<9, 3>(0, 3) = _jacobianOplus[3];
    J.block<9, 2>(0, 6) = _jacobianOplus[6];
    J.block<9, 1>(0, 8) = _jacobianOplus[7];
    hessian3_out        = J.transpose() * information() * J;
    return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
}

} /* namespace edge_inertial_gs */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */
