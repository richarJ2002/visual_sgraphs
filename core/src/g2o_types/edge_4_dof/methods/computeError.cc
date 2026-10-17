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
 * @file            computeError.cc
 *
 * @brief           Implements Edge4DoF::computeError().
 */

#include "g2o_types/edge_4_dof/edge_4_dof.h"
#include "g2o_types/vertex_pose_4_dof/vertex_pose_4_dof.h"
#include "g2o_types/g2o_types/public_functions/public_functions.h"

namespace vs_graphs::core::g2o_types::edge_4_dof
{

void Edge4DoF::computeError()
{
        const g2o_types::vertex_pose_4_dof::VertexPose4DoF *VPi =
            static_cast<const g2o_types::vertex_pose_4_dof::VertexPose4DoF *>(_vertices[0]);
        const g2o_types::vertex_pose_4_dof::VertexPose4DoF *VPj =
            static_cast<const g2o_types::vertex_pose_4_dof::VertexPose4DoF *>(_vertices[1]);
        Eigen::Vector3d rotationVector{};
        if (logSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() *
                       dRij.transpose(),
                   rotationVector) != G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: logSO3 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error << rotationVector,
            VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() *
                                      VPj->estimate().tcw[0]) +
                VPi->estimate().tcw[0] - dtij;
    }

} /* namespace vs_graphs::core::g2o_types::edge_4_dof */
