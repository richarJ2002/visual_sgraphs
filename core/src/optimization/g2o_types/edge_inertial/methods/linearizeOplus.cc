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
 * @file            linearizeOplus.cc
 *
 * @brief           Implements EdgeInertial::linearizeOplus(), declared in
 *                  G2oTypes.h.
 */

#include "optimization/g2o_types/edge_inertial/objects/EdgeInertialClass.h"
#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"
#include "optimization/g2o_types/g2o_types/public_functions/public_functions.h"
#include "optimization/g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "optimization/g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include "optimization/g2o_types/vertex_velocity/vertex_velocity.h"

#include <Eigen/Dense>

#include <rclcpp/logging.hpp>

#include "optimization/g2o_types/imu_cam_pose/imu_cam_pose.h"
#include "optimization/g2o_types/vertex_pose/vertex_pose.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/preintegrated.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_inertial
{

void EdgeInertial::linearizeOplus()
{
    const vertex_pose::VertexPose *p_previousPoseVertex =
        static_cast<const vertex_pose::VertexPose *>(_vertices[0]);
    const vertex_velocity::VertexVelocity *p_previousVelocityVertex =
        static_cast<const vertex_velocity::VertexVelocity *>(_vertices[1]);
    const vertex_gyro_bias::VertexGyroBias *p_previousGyroBiasVertex =
        static_cast<const vertex_gyro_bias::VertexGyroBias *>(_vertices[2]);
    const vertex_acc_bias::VertexAccBias *p_previousAccBiasVertex =
        static_cast<const vertex_acc_bias::VertexAccBias *>(_vertices[3]);
    const vertex_pose::VertexPose *p_currentPoseVertex =
        static_cast<const vertex_pose::VertexPose *>(_vertices[4]);
    const vertex_velocity::VertexVelocity *p_currentVelocityVertex =
        static_cast<const vertex_velocity::VertexVelocity *>(_vertices[5]);
    const imu_types::bias::Bias biasEstimate(
        p_previousAccBiasVertex->estimate()[0],
        p_previousAccBiasVertex->estimate()[1],
        p_previousAccBiasVertex->estimate()[2],
        p_previousGyroBiasVertex->estimate()[0],
        p_previousGyroBiasVertex->estimate()[1],
        p_previousGyroBiasVertex->estimate()[2]);
    imu_types::bias::Bias deltaBias{};
    if (p_preintegrated->getDeltaBias(biasEstimate, deltaBias) !=
        imu_types::preintegrated::PreintegratedStatus::
            PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d deltaGyroBias;
    deltaGyroBias << deltaBias.bwx, deltaBias.bwy, deltaBias.bwz;

    const Eigen::Matrix3d bodyRotation_body1ToWorld =
        p_previousPoseVertex->estimate().Rwb;
    const Eigen::Matrix3d bodyRotation_worldToBody1 =
        bodyRotation_body1ToWorld.transpose();
    const Eigen::Matrix3d bodyRotation_body2ToWorld =
        p_currentPoseVertex->estimate().Rwb;

    Eigen::Matrix3f preintegratedDeltaRotation{};
    if (p_preintegrated->getDeltaRotation(biasEstimate,
                                          preintegratedDeltaRotation) !=
        imu_types::preintegrated::PreintegratedStatus::
            PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaRotation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Matrix3d deltaRotation =
        preintegratedDeltaRotation.cast<double>();
    const Eigen::Matrix3d rotationErrorMatrix = deltaRotation.transpose() *
                                                bodyRotation_worldToBody1 *
                                                bodyRotation_body2ToWorld;
    Eigen::Vector3d rotationError{};
    if (logSO3(rotationErrorMatrix, rotationError) !=
        G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: logSO3 returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Matrix3d inverseRightJacobian{};
    if (inverseRightJacobianSO3(rotationError, inverseRightJacobian) !=
        G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: inverseRightJacobianSO3 returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Jacobians wrt Pose 1 */
    _jacobianOplus[0].setZero();
    /* rotation */
    _jacobianOplus[0].block<3, 3>(0, 0) =
        -inverseRightJacobian * bodyRotation_body2ToWorld.transpose() *
        bodyRotation_body1ToWorld; /* OK */
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(
        bodyRotation_worldToBody1 *
        (p_currentVelocityVertex->estimate() -
         p_previousVelocityVertex->estimate() - g * dt)); /* OK */
    /* OK */
    _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
        bodyRotation_worldToBody1 *
        (p_currentPoseVertex->estimate().twb -
         p_previousPoseVertex->estimate().twb -
         p_previousVelocityVertex->estimate() * dt - 0.5 * g * dt * dt));
    /* translation */
    _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity(); /* OK */

    /* Jacobians wrt Velocity 1 */
    _jacobianOplus[1].setZero();
    /* OK */
    _jacobianOplus[1].block<3, 3>(3, 0) = -bodyRotation_worldToBody1;
    /* OK */
    _jacobianOplus[1].block<3, 3>(6, 0) = -bodyRotation_worldToBody1 * dt;

    /* Jacobians wrt Gyro 1 */
    _jacobianOplus[2].setZero();
    Eigen::Matrix3d rightJacobian{};
    if (rightJacobianSO3(JRg * deltaGyroBias, rightJacobian) !=
        G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rightJacobianSO3 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    _jacobianOplus[2].block<3, 3>(0, 0) = -inverseRightJacobian *
                                          rotationErrorMatrix.transpose() *
                                          rightJacobian * JRg; /* OK */
    _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;                /* OK */
    _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;                /* OK */

    /* Jacobians wrt Accelerometer 1 */
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -JVa; /* OK */
    _jacobianOplus[3].block<3, 3>(6, 0) = -JPa; /* OK */

    /* Jacobians wrt Pose 2 */
    _jacobianOplus[4].setZero();
    /* rotation */
    _jacobianOplus[4].block<3, 3>(0, 0) = inverseRightJacobian; /* OK */
    /* translation */
    _jacobianOplus[4].block<3, 3>(6, 3) =
        bodyRotation_worldToBody1 * bodyRotation_body2ToWorld; /* OK */

    /* Jacobians wrt Velocity 2 */
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = bodyRotation_worldToBody1; /* OK */
}

} /* namespace edge_inertial */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */
