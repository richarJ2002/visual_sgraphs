/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            private_functions.h
 *
 * @brief           Declares the imu_types free helpers and constants.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "dependency_lineage/sophus/objects/so3.hpp"

#include "sensor_models/imu_types/preintegrated/objects/ImuConstants.h"
#include "sensor_models/imu_types/private_functions/ImuTypesStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{

/*!
 * @brief           Returns the SO(3) right Jacobian from angle-axis components.
 *
 * @param[in]       rotationVectorX_in
 *                  X rotation component, radians.
 * @param[in]       rotationVectorY_in
 *                  Y rotation component, radians.
 * @param[in]       rotationVectorZ_in
 *                  Z rotation component, radians.
 * @param[out]      rightJacobian_out
 *                  Three-by-three right Jacobian.
 * @return          IMU_TYPES_STATUS_SUCCESS.
 */
[[nodiscard]] ImuTypesStatus rightJacobianSO3(
    const float &rotationVectorX_in, const float &rotationVectorY_in,
    const float &rotationVectorZ_in, Eigen::Matrix3f &rightJacobian_out);

/*!
 * @brief           Returns the right Jacobian of SO3 at the
 *                  given rotation vector.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector in radians.
 *
 * @param[out]      rightJacobian_out
 *                  Three-by-three right Jacobian.
 *
 * @return          IMU_TYPES_STATUS_SUCCESS.
 */
[[nodiscard]] ImuTypesStatus
    rightJacobianSO3(const Eigen::Vector3f &rotationVector_in,
                     Eigen::Matrix3f       &rightJacobian_out);

/*!
 * @brief           Returns the inverse right Jacobian of SO3 at
 *                  the given rotation vector.
 *
 * @param[in]       angleAxisX_in
 *                  Rotation x in radians.
 *
 * @param[in]       angleAxisY_in
 *                  Rotation y in radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Rotation z in radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Three-by-three inverse right Jacobian.
 *
 * @return          IMU_TYPES_STATUS_SUCCESS.
 */
[[nodiscard]] ImuTypesStatus
    inverseRightJacobianSO3(const float     &angleAxisX_in,
                            const float     &angleAxisY_in,
                            const float     &angleAxisZ_in,
                            Eigen::Matrix3f &inverseRightJacobian_out);
/*!
 * @brief           Returns the inverse right Jacobian of SO3 at
 *                  the given rotation vector.
 *
 * @param[in]       angleAxisVector_in
 *                  Rotation vector in radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Three-by-three inverse right Jacobian.
 *
 * @return          IMU_TYPES_STATUS_SUCCESS.
 */
[[nodiscard]] ImuTypesStatus
    inverseRightJacobianSO3(const Eigen::Vector3f &angleAxisVector_in,
                            Eigen::Matrix3f       &inverseRightJacobian_out);

/*!
 * @brief           Re-orthonormalizes a rotation estimate.
 *
 * @param[in]       rotationMatrix_in
 *                  Rotation to normalize.
 *
 * @param[out]      rotation_out
 *                  Closest rotation in the Frobenius sense.
 *
 * @return          IMU_TYPES_STATUS_SUCCESS.
 */
[[nodiscard]] ImuTypesStatus
    normalizeRotation(const Eigen::Matrix3f &rotationMatrix_in,
                      Eigen::Matrix3f       &rotation_out);

} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_PRIVATE_FUNCTIONS_PRIVATE_FUNCTIONS_H   \
        */
