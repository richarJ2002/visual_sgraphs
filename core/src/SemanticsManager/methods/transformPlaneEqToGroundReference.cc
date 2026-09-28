/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "SemanticsManager.h"

namespace vs_graphs
{
namespace core
{

Eigen::Vector3f SemanticsManager::transformPlaneEqToGroundReference(
    const Eigen::Vector4d &planeEq_in)
{
    /* extract the rotation matrix from the transformation matrix */
    Eigen::Matrix3f rotationMatrix = planePoseMat.block<3, 3>(0, 0);

    /* Compute the inverse transpose of the rotation matrix */
    Eigen::Matrix3f inverseTransposeRotationMatrix =
        rotationMatrix.inverse().transpose();

    /* Transform the coefficients of the plane equation */
    Eigen::Vector3f transformedPlaneCoefficients =
        inverseTransposeRotationMatrix * planeEq_in.head<3>().cast<float>();

    /* Find the normalized coefficients */
    transformedPlaneCoefficients.normalize();

    return transformedPlaneCoefficients;
}

} // namespace core
} // namespace vs_graphs
