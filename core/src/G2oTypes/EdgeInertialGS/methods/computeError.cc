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
 * @brief           Implements EdgeInertialGS::computeError(), declared in
 *                  G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void EdgeInertialGS::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between
    // linearization point and current estimate is too big
    const VertexPose *p_previousPoseVertex =
        static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *p_previousVelocityVertex =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *p_gyroBiasVertex =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias *p_accBiasVertex =
        static_cast<const VertexAccBias *>(_vertices[3]);
    const VertexPose *p_currentPoseVertex =
        static_cast<const VertexPose *>(_vertices[4]);
    const VertexVelocity *p_currentVelocityVertex =
        static_cast<const VertexVelocity *>(_vertices[5]);
    const VertexGDir *p_gravityDirectionVertex =
        static_cast<const VertexGDir *>(_vertices[6]);
    const VertexScale *p_scaleVertex =
        static_cast<const VertexScale *>(_vertices[7]);
    const IMU::Bias biasEstimate(p_accBiasVertex->estimate()[0],
                                 p_accBiasVertex->estimate()[1],
                                 p_accBiasVertex->estimate()[2],
                                 p_gyroBiasVertex->estimate()[0],
                                 p_gyroBiasVertex->estimate()[1],
                                 p_gyroBiasVertex->estimate()[2]);
    g = p_gravityDirectionVertex->estimate().Rwg * gI;
    const double    scaleEstimate = p_scaleVertex->estimate();
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
    Eigen::Vector3f preintegratedDeltaVelocity{};
    if (p_preintegrated->getDeltaVelocity(biasEstimate,
                                          preintegratedDeltaVelocity) !=
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaVelocity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d deltaVelocity =
        preintegratedDeltaVelocity.cast<double>();
    Eigen::Vector3f preintegratedDeltaPosition{};
    if (p_preintegrated->getDeltaPosition(biasEstimate,
                                          preintegratedDeltaPosition) !=
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getDeltaPosition returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d deltaPosition =
        preintegratedDeltaPosition.cast<double>();

    Eigen::Vector3d rotationError{};
    if (logSO3(deltaRotation.transpose() *
                   p_previousPoseVertex->estimate().Rwb.transpose() *
                   p_currentPoseVertex->estimate().Rwb,
               rotationError) != G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: logSO3 returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d velocityError =
        p_previousPoseVertex->estimate().Rwb.transpose() *
            (scaleEstimate * (p_currentVelocityVertex->estimate() -
                              p_previousVelocityVertex->estimate()) -
             g * dt) -
        deltaVelocity;
    const Eigen::Vector3d positionError =
        p_previousPoseVertex->estimate().Rwb.transpose() *
            (scaleEstimate * (p_currentPoseVertex->estimate().twb -
                              p_previousPoseVertex->estimate().twb -
                              p_previousVelocityVertex->estimate() * dt) -
             g * dt * dt / 2) -
        deltaPosition;

    _error << rotationError, velocityError, positionError;
}

} // namespace core
} // namespace vs_graphs
