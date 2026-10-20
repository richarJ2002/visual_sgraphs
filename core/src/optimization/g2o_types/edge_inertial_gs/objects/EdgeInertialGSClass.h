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
 * @file            EdgeInertialGSClass.h
 *
 * @brief           Declares the IMU, gravity and scale constraint edge.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_GS_OBJECTS_EDGE_INERTIAL_G_S_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_GS_OBJECTS_EDGE_INERTIAL_G_S_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_multi_edge.h>

#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/preintegrated.h"

#include "optimization/g2o_types/edge_inertial_gs/objects/EdgeInertialGSStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_inertial_gs
{

/*!
 * @brief           IMU residual vector used by this edge.
 */
typedef Eigen::Matrix<double, 9, 1> Vector9d;

/*!
 * @brief           Constrains motion, gravity and scale with IMU data.
 */
class EdgeInertialGS : public unsupported_graph_io::UnsupportedGraphIo<
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
    EdgeInertialGS(imu_types::preintegrated::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        /* This edge links 8 vertices */
        resize(8);
        gI << 0, 0, -imu_types::preintegrated::GRAVITY_VALUE;

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
     *                  the two frames, with the gravity direction and scale
     *                  estimates applied, and the preintegrated motion
     *                  corrected with the current bias estimates; the nine
     *                  values are rotation, velocity and position. Also
     *                  refreshes g from the gravity direction vertex.
     */
    void                                     computeError() override;
    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..7],
     *                  one nine-row block for each vertex in the order of the
     *                  class description.
     */
    void                                     linearizeOplus() override;
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
     *                  recomputed by computeError as the gravity rotation times
     *                  gI.
     */
    Eigen::Vector3d                          g;
    /*!
     * @brief           Gravity in the gravity-aligned frame, metres per second
     *                  squared; fixed at (0, 0, -GRAVITY_VALUE).
     */
    Eigen::Vector3d                          gI;
    /*!
     * @brief           Returns the 27 by 27 Gauss-Newton Hessian over all eight
     *                  vertices, ordered previous pose (6), previous velocity
     *                  (3), gyroscope bias (3), accelerometer bias (3), current
     *                  pose (6), current velocity (3), gravity direction (2),
     *                  scale (1). Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian(Eigen::Matrix<double, 27, 27> &hessian_out);
    /*!
     * @brief           Returns the 27 by 27 Gauss-Newton Hessian over all eight
     *                  vertices in a different order: gyroscope bias (3),
     *                  accelerometer bias (3), gravity direction (2), scale
     *                  (1), previous velocity (3), current velocity (3),
     *                  previous pose (6), current pose (6). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian2(Eigen::Matrix<double, 27, 27> &hessian2_out);
    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over the
     *                  gyroscope bias (3), accelerometer bias (3), gravity
     *                  direction (2) and scale (1), in that order. Relinearises
     *                  at the current estimates first.
     *
     * @param[out]      hessian3_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian3(Eigen::Matrix<double, 9, 9> &hessian3_out);
    /*!
     * @brief           Returns the 1 by 1 Gauss-Newton Hessian of the scale
     *                  vertex. Relinearises at the current estimates first.
     *
     * @param[out]      hessianScale_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianScale(Eigen::Matrix<double, 1, 1> &hessianScale_out);
    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the gyroscope
     *                  bias vertex. Relinearises at the current estimates
     *                  first.
     *
     * @param[out]      hessianBiasGyro_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasGyro(Eigen::Matrix<double, 3, 3> &hessianBiasGyro_out);
    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the
     *                  accelerometer bias vertex. Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessianBiasAcc_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasAcc(Eigen::Matrix<double, 3, 3> &hessianBiasAcc_out);
    /*!
     * @brief           Returns the 2 by 2 Gauss-Newton Hessian of the gravity
     *                  direction vertex. Relinearises at the current estimates
     *                  first.
     *
     * @param[out]      hessianGDir_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus getHessianGravityDirection(
        Eigen::Matrix<double, 2, 2> &hessianGDir_out);
};

} /* namespace edge_inertial_gs */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_INERTIAL_GS_OBJECTS_EDGE_INERTIAL_G_S_CLASS_H \
        */
