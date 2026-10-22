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
 * @file            EdgePriorGyroClass.h
 *
 * @brief           Declares the EdgePriorGyro
 *                  Unary edge holding the gyroscope bias prior error.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_GYRO_OBJECTS_EDGE_PRIOR_GYRO_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_GYRO_OBJECTS_EDGE_PRIOR_GYRO_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_unary_edge.h>

#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "optimization/g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"

#include "optimization/g2o_types/edge_prior_gyro/objects/EdgePriorGyroStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_prior_gyro
{

/*!
 * @brief           Unary edge holding the gyroscope bias prior error.
 */
class EdgePriorGyro : public unsupported_graph_io::UnsupportedGraphIo<
                          g2o::BaseUnaryEdge<3,
                                             Eigen::Vector3d,
                                             vertex_gyro_bias::VertexGyroBias>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the edge with the prior bias.
     *
     * @param[in]       bprior_in
     *                  Prior gyroscope bias, radians per second.
     */
    EdgePriorGyro(const Eigen::Vector3f &bprior_in) :
        bprior(bprior_in.cast<double>())
    {}

    /*!
     * @brief           Sets the error to the prior bias minus the bias
     *                  estimate.
     */
    void computeError() override;

    /*!
     * @brief           Sets the Jacobian of the error with respect to the bias
     *                  to the identity.
     */
    void linearizeOplus() override;

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the bias
     *                  vertex. Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_GYRO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorGyroStatus
        getHessian(Eigen::Matrix<double, 3, 3> &hessian_out);

    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    const Eigen::Vector3d bprior;
};

} /* namespace edge_prior_gyro */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_GYRO_OBJECTS_EDGE_PRIOR_GYRO_CLASS_H \
        */
