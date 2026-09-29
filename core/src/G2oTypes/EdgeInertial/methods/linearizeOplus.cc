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

void EdgeInertial::linearizeOplus()
{
    const VertexPose *p_previousPoseVertex =
        static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *p_previousVelocityVertex =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *p_previousGyroBiasVertex =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias *p_previousAccBiasVertex =
        static_cast<const VertexAccBias *>(_vertices[3]);
    const VertexPose *p_currentPoseVertex =
        static_cast<const VertexPose *>(_vertices[4]);
    const VertexVelocity *p_currentVelocityVertex =
        static_cast<const VertexVelocity *>(_vertices[5]);
    const IMU::Bias biasEstimate(p_previousAccBiasVertex->estimate()[0],
                                 p_previousAccBiasVertex->estimate()[1],
                                 p_previousAccBiasVertex->estimate()[2],
                                 p_previousGyroBiasVertex->estimate()[0],
                                 p_previousGyroBiasVertex->estimate()[1],
                                 p_previousGyroBiasVertex->estimate()[2]);
    IMU::Bias       deltaBias{};
    if (p_preintegrated->getDeltaBias(biasEstimate, deltaBias) !=
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d deltaGyroBias;
    deltaGyroBias << deltaBias.bwx, deltaBias.bwy, deltaBias.bwz;

    const Eigen::Matrix3d Rwb1 = p_previousPoseVertex->estimate().Rwb;
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = p_currentPoseVertex->estimate().Rwb;

    Eigen::Matrix3f preintegratedDeltaRotation{};
    if (p_preintegrated->getDeltaRotation(biasEstimate,
                                          preintegratedDeltaRotation) !=
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaRotation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Matrix3d deltaRotation =
        preintegratedDeltaRotation.cast<double>();
    const Eigen::Matrix3d rotationErrorMatrix =
        deltaRotation.transpose() * Rbw1 * Rwb2;
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

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
    // rotation
    _jacobianOplus[0].block<3, 3>(0, 0) =
        -inverseRightJacobian * Rwb2.transpose() * Rwb1; // OK
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(
        Rbw1 * (p_currentVelocityVertex->estimate() -
                p_previousVelocityVertex->estimate() - g * dt)); // OK
    _jacobianOplus[0].block<3, 3>(6, 0) =
        Sophus::SO3d::hat(Rbw1 * (p_currentPoseVertex->estimate().twb -
                                  p_previousPoseVertex->estimate().twb -
                                  p_previousVelocityVertex->estimate() * dt -
                                  0.5 * g * dt * dt)); // OK
    // translation
    _jacobianOplus[0].block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity(); // OK

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -Rbw1;      // OK
    _jacobianOplus[1].block<3, 3>(6, 0) = -Rbw1 * dt; // OK

    // Jacobians wrt Gyro 1
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
                                          rightJacobian * JRg; // OK
    _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;                // OK
    _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;                // OK

    // Jacobians wrt Accelerometer 1
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -JVa; // OK
    _jacobianOplus[3].block<3, 3>(6, 0) = -JPa; // OK

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3, 3>(0, 0) = inverseRightJacobian; // OK
    // translation
    _jacobianOplus[4].block<3, 3>(6, 3) = Rbw1 * Rwb2; // OK

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = Rbw1; // OK
}

} // namespace core
} // namespace vs_graphs
