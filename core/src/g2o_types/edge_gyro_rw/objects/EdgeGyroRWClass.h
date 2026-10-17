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

#ifndef VS_GRAPHS_G2O_TYPES_EDGE_GYRO_RW_H
#define VS_GRAPHS_G2O_TYPES_EDGE_GYRO_RW_H

#include "g2o_types/edge_gyro_rw/objects/EdgeGyroRWStatusEnum.h"
#include "g2o_types/imu_cam_pose/imu_cam_pose.h"
#include "g2o_types/g2o_types/public_functions/public_functions.h"
#include "g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include <Eigen/Dense>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/core/base_vertex.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::g2o_types::edge_gyro_rw
{

/*!
 * @brief           g2o binary edge that models the gyroscope bias as a random
 *                  walk: it penalises the change of the bias between two
 *                  consecutive frames (vertex 0 and vertex 1, both
 *                  VertexGyroBias). The three-value error is the second bias
 *                  minus the first.
 */
class EdgeGyroRW
    : public unsupported_graph_io::UnsupportedGraphIo<
          g2o::BaseBinaryEdge<3,
                              Eigen::Vector3d,
                              g2o_types::vertex_gyro_bias::VertexGyroBias,
                              g2o_types::vertex_gyro_bias::VertexGyroBias>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}

    /*!
     * @brief           Sets the error to the second bias estimate minus the
     *                  first.
     */
    void computeError() override;

    /*!
     * @brief           Sets the Jacobians of the error: minus the identity for
     *                  the first bias and the identity for the second.
     */
    void linearizeOplus() override;

    /*!
     * @brief           Returns the 6 by 6 Gauss-Newton Hessian over both
     *                  biases, ordered first (3) then second (3). Relinearises
     *                  at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_GYRO_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeGyroRWStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out);

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the second
     *                  bias only. Relinearises at the current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_GYRO_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeGyroRWStatus getHessian2(Eigen::Matrix3d &hessian2_out);
};

} /* namespace vs_graphs::core::g2o_types::edge_gyro_rw */

#endif /* VS_GRAPHS_G2O_TYPES_EDGE_GYRO_RW_H */
