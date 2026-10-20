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
 * @file            public_functions.h
 *
 * @brief           Declares the g2o_types public free SO(3) helpers.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "optimization/g2o_types/g2o_types/objects/G2oTypesStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{

/*!
 * @brief           Converts a rotation vector given as three components into
 *                  the rotation matrix it describes (exponential map of SO(3)).
 *                  Angles below 1e-5 rad use a second-order approximation; the
 *                  result is re-orthogonalised.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      rotation_out
 *                  Rotation matrix for that rotation vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus expSO3(const double     angleAxisX_in,
                                    const double     angleAxisY_in,
                                    const double     angleAxisZ_in,
                                    Eigen::Matrix3d &rotation_out);

/*!
 * @brief           Converts a rotation vector into the rotation matrix it
 *                  describes (exponential map of SO(3)); same as the overload
 *                  taking three components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector (axis times angle), radians.
 *
 * @param[out]      rotation_out
 *                  Rotation matrix for that rotation vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus expSO3(const Eigen::Vector3d &rotationVector_in,
                                    Eigen::Matrix3d       &rotation_out);

/*!
 * @brief           Converts a rotation matrix into its rotation vector
 *                  (logarithmic map of SO(3)). When the matrix is not a valid
 *                  rotation or its angle is within 1e-5 of 0 or pi, the vector
 *                  part of the matrix is returned without scaling by the angle.
 *
 * @param[in]       rotationMatrix_in
 *                  Rotation matrix to convert.
 *
 * @param[out]      rotationVector_out
 *                  Rotation vector (axis times angle), radians.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus logSO3(const Eigen::Matrix3d &rotationMatrix_in,
                                    Eigen::Vector3d       &rotationVector_out);

/*!
 * @brief           Computes the inverse of the right Jacobian of SO(3) at a
 *                  rotation vector; same as the overload taking three
 *                  components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector the Jacobian is evaluated at, radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Inverse right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                            Eigen::Matrix3d       &inverseRightJacobian_out);
/*!
 * @brief           Computes the right Jacobian of SO(3) at a rotation vector;
 *                  same as the overload taking three components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector the Jacobian is evaluated at, radians.
 *
 * @param[out]      rightJacobian_out
 *                  Right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                     Eigen::Matrix3d       &rightJacobian_out);
/*!
 * @brief           Computes the right Jacobian of SO(3), which relates a small
 *                  change of a rotation vector to the change of the rotation on
 *                  its right side. It is the identity for angles below 1e-5
 *                  rad.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      rightJacobian_out
 *                  Right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const double     angleAxisX_in,
                     const double     angleAxisY_in,
                     const double     angleAxisZ_in,
                     Eigen::Matrix3d &rightJacobian_out);

/*!
 * @brief           Builds the skew-symmetric matrix of a 3D vector, so that the
 *                  matrix times another vector equals the cross product of the
 *                  two.
 *
 * @param[in]       angularVelocity_in
 *                  Vector to convert; the caller passes an angular velocity or
 *                  a rotation vector.
 *
 * @param[out]      skewMatrix_out
 *                  Skew-symmetric matrix of that vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    computeSkewMatrix(const Eigen::Vector3d &angularVelocity_in,
                      Eigen::Matrix3d       &skewMatrix_out);
/*!
 * @brief           Computes the inverse of the right Jacobian of SO(3) at a
 *                  rotation vector given as three components. It is the
 *                  identity for angles below 1e-5 rad.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Inverse right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const double     angleAxisX_in,
                            const double     angleAxisY_in,
                            const double     angleAxisZ_in,
                            Eigen::Matrix3d &inverseRightJacobian_out);

/*!
 * @brief           Replaces a nearly orthogonal matrix by the closest
 *                  orthogonal matrix, which is the product of the two
 *                  orthogonal factors of its singular value decomposition. Used
 *                  to remove numerical drift from rotation matrices.
 *
 * @param[in]       rotationMatrix_in
 *                  Matrix to orthogonalise, expected to be close to a rotation.
 *
 * @param[out]      rotation_out
 *                  Closest orthogonal matrix to R.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
template <typename T = double>
[[nodiscard]] G2oTypesStatus
    normalizeRotation(const Eigen::Matrix<T, 3, 3> &rotationMatrix_in,
                      Eigen::Matrix<T, 3, 3>       &rotation_out);

} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#include "optimization/g2o_types/g2o_types/public_functions/normalizeRotation.tpp"

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_PUBLIC_FUNCTIONS_PUBLIC_FUNCTIONS_H     \
        */
