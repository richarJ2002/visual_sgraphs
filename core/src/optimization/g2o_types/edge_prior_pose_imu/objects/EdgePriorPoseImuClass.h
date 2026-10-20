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
 * @file            EdgePriorPoseImuClass.h
 *
 * @brief           Declares the EdgePriorPoseImu
 *                  Multi edge holding the IMU prior residual over pose,
 *                  velocity and biases.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_EDGE_PRIOR_POSE_IMU_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_EDGE_PRIOR_POSE_IMU_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_multi_edge.h>

#include "optimization/g2o_types/edge_prior_pose_imu/objects/ConstraintPoseImuClass.h"
#include "optimization/g2o_types/edge_prior_pose_imu/objects/EdgePriorPoseImuStatusEnum.h"
#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_prior_pose_imu
{

/*!
 * @brief           IMU residual vector used by this edge.
 */
typedef Eigen::Matrix<double, 15, 1> Vector15d;

/*!
 * @brief           Multi edge holding pose, velocity and bias IMU priors.
 */
class EdgePriorPoseImu : public unsupported_graph_io::UnsupportedGraphIo<
                             g2o::BaseMultiEdge<15, Vector15d>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the edge from a prior: copies its values and
     *                  uses its information matrix.
     *
     * @param[in]       p_c_inout
     *                  Prior to copy; dereferenced without a null check, only
     *                  read and not kept.
     */
    EdgePriorPoseImu(ConstraintPoseImu *p_c_inout)
    {
        resize(4);
        Rwb = p_c_inout->Rwb;
        twb = p_c_inout->twb;
        vwb = p_c_inout->vwb;
        bg  = p_c_inout->bg;
        ba  = p_c_inout->ba;
        setInformation(p_c_inout->H);
    }

    /*!
     * @brief           Sets the error to the current estimates minus the prior:
     *                  rotation error, translation error in the prior body
     *                  frame, then velocity and the two biases.
     */
    void computeError() override;

    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..3],
     *                  one fifteen-row block for each vertex.
     */
    void linearizeOplus() override;

    /*!
     * @brief           Returns the 15 by 15 Gauss-Newton Hessian over all four
     *                  vertices, ordered pose (6), velocity (3), gyroscope bias
     *                  (3), accelerometer bias (3). Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorPoseImuStatus
        getHessian(Eigen::Matrix<double, 15, 15> &hessian_out);

    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over velocity
     *                  (3), gyroscope bias (3) and accelerometer bias (3),
     *                  leaving out the pose. Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessianNoPose_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorPoseImuStatus
        getHessianNoPose(Eigen::Matrix<double, 9, 9> &hessianNoPose_out);

    /*!
     * @brief           Prior rotation of the IMU (body) frame in the world
     *                  frame.
     */
    Eigen::Matrix3d Rwb;

    /*!
     * @brief           Prior position of the IMU origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d twb;

    /*!
     * @brief           Prior velocity of the IMU in the world frame, metres per
     *                  second.
     */
    Eigen::Vector3d vwb;

    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    Eigen::Vector3d bg;

    /*!
     * @brief           Prior accelerometer bias, metres per second squared.
     */
    Eigen::Vector3d ba;
};

} /* namespace edge_prior_pose_imu */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_EDGE_PRIOR_POSE_IMU_CLASS_H \
        */
