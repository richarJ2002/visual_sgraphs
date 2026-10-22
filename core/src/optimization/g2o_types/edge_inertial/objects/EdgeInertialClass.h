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
 * @file            EdgeInertialClass.h
 *
 * @brief           Declares the EdgeInertial
 *                  Multi edge over six vertices constraining two consecutive
 *                  frames with one IMU preintegration.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_OBJECTS_EDGE_INERTIAL_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_OBJECTS_EDGE_INERTIAL_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_multi_edge.h>

#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/preintegrated.h"

#include "optimization/g2o_types/edge_inertial/objects/EdgeInertialStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_inertial
{

/*!
 * @brief           IMU residual vector used by this edge.
 */
typedef Eigen::Matrix<double, 9, 1> Vector9d;

/*!
 * @brief           Multi edge over six vertices constraining two consecutive
 *                  frames with one IMU preintegration.
 */
class EdgeInertial : public unsupported_graph_io::UnsupportedGraphIo<
                         g2o::BaseMultiEdge<9, Vector9d>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the edge from a preintegration. The information
     *                  matrix is the inverse of the preintegrated covariance
     *                  (rotation, velocity and position block) with eigenvalues
     *                  below 1e-12 set to zero.
     *
     * @param[in]       p_int_inout
     *                  IMU preintegration between two consecutive frames; the
     *                  edge keeps this pointer (borrowed, never deleted) and
     *                  copies its bias Jacobians and time span. Dereferenced
     *                  without a null check.
     */
    EdgeInertial(imu_types::preintegrated::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        /* This edge links 6 vertices */
        resize(6);
        g << 0, 0, -imu_types::preintegrated::GRAVITY_VALUE;

        Eigen::Matrix<double, 9, 9> Info =
            p_int_inout->C.block<9, 9>(0, 0).cast<double>().inverse();
        Info = (Info + Info.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(Info);
        Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 9; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        Info = es.eigenvectors() * eigs.asDiagonal() *
               es.eigenvectors().transpose();
        setInformation(Info);
    }
    /*!
     * @brief           Sets the error to the difference between the motion of
     *                  the two frames and the preintegrated motion corrected
     *                  with the current bias estimates; the nine values are
     *                  rotation, velocity and position.
     */
    void computeError() override;
    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..5],
     *                  one nine-row block for each vertex in the order of the
     *                  class description.
     */
    void linearizeOplus() override;
    /*!
     * @brief           Returns the 24 by 24 Gauss-Newton Hessian over all six
     *                  vertices, ordered previous pose (6), previous velocity
     *                  (3), gyroscope bias (3), accelerometer bias (3), current
     *                  pose (6), current velocity (3). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessian(Eigen::Matrix<double, 24, 24> &hessian_out);
    /*!
     * @brief           Returns the 18 by 18 Gauss-Newton Hessian over all
     *                  vertices except the previous pose, ordered previous
     *                  velocity (3), gyroscope bias (3), accelerometer bias
     *                  (3), current pose (6), current velocity (3).
     *                  Relinearises at the current estimates first.
     *
     * @param[out]      hessianNoPose1_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessianNoPose1(Eigen::Matrix<double, 18, 18> &hessianNoPose1_out);
    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over the current
     *                  frame only, ordered current pose (6), current velocity
     *                  (3). Relinearises at the current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessian2(Eigen::Matrix<double, 9, 9> &hessian2_out);
    /*!
     * @brief           Jacobian of the preintegrated rotation with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d                    JRg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d                    JVg;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d                    JPg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d                    JVa;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d                    JPa;
    /*!
     * @brief           IMU preintegration this edge compares against; borrowed,
     *                  never deleted here.
     */
    imu_types::preintegrated::Preintegrated *p_preintegrated;
    /*!
     * @brief           Time span of the preintegration, seconds.
     */
    const double                             dt;
    /*!
     * @brief           Gravity in the world frame, metres per second squared;
     *                  fixed at (0, 0, -GRAVITY_VALUE).
     */
    Eigen::Vector3d                          g;
};

} /* namespace edge_inertial */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_OBJECTS_EDGE_INERTIAL_CLASS_H \
        */
