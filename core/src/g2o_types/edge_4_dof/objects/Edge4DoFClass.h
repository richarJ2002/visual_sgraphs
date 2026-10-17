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

#ifndef VS_GRAPHS_G2O_TYPES_EDGE_4_DOF_H
#define VS_GRAPHS_G2O_TYPES_EDGE_4_DOF_H

#include "g2o_types/imu_cam_pose/imu_cam_pose.h"
#include "g2o_types/g2o_types/public_functions/public_functions.h"
#include "g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include "g2o_types/vertex_pose_4_dof/vertex_pose_4_dof.h"
#include <Eigen/Dense>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/core/base_vertex.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::g2o_types::edge_4_dof
{

/*!
 * @brief           g2o binary edge for pose-graph optimisation between two
 *                  VertexPose4DoF vertices (vertex 0 and vertex 1). The
 *                  six-value error is the rotation error followed by the
 *                  translation error against a measured relative transform.
 */
class Edge4DoF
    : public unsupported_graph_io::UnsupportedGraphIo<
          g2o::BaseBinaryEdge<6,
                              Eigen::Matrix<double, 6, 1>,
                              g2o_types::vertex_pose_4_dof::VertexPose4DoF,
                              g2o_types::vertex_pose_4_dof::VertexPose4DoF>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge from a measured relative transform and
     *                  splits it into dRij and dtij.
     *
     * @param[in]       deltaT_in
     *                  Measured transform from the camera of vertex 1 to the
     *                  camera of vertex 0, homogeneous 4 by 4.
     */
    Edge4DoF(const Eigen::Matrix4d &deltaT_in)
    {
        dTij = deltaT_in;
        dRij = deltaT_in.block<3, 3>(0, 0);
        dtij = deltaT_in.block<3, 1>(0, 3);
    }

    /*!
     * @brief           Sets the error to the rotation vector of the relative
     *                  camera rotation against dRij, followed by the position
     *                  of the second camera in the first camera frame minus
     *                  dtij. Failed logarithm maps are only logged.
     */
    void computeError() override;

    /* No linearizeOplus override: g2o estimates the Jacobian numerically */

    /*!
     * @brief           Measured relative transform from the camera of vertex 1
     *                  to the camera of vertex 0, homogeneous.
     */
    Eigen::Matrix4d dTij;
    /*!
     * @brief           Rotation part of dTij.
     */
    Eigen::Matrix3d dRij;
    /*!
     * @brief           Translation part of dTij, metres.
     */
    Eigen::Vector3d dtij;
};

} /* namespace vs_graphs::core::g2o_types::edge_4_dof */

#endif /* VS_GRAPHS_G2O_TYPES_EDGE_4_DOF_H */
