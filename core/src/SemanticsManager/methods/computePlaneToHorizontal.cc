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

Eigen::Matrix4f SemanticsManager::computePlaneToHorizontal(
    const geometric::Plane *p_plane_in)
{
    // initialize the transformation with translation set to a zero vector
    Eigen::Isometry3d planePose;
    planePose.translation() = Eigen::Vector3d(0, 0, 0);

    // normalize the normal vector
    g2o::Plane3D planeGetGlobalEquation{};
    if (p_plane_in->getGlobalEquation(planeGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getGlobalEquation cannot fail; continue as before.
    }
    Eigen::Vector3d planeNormal = planeGetGlobalEquation.coeffs().head<3>();

    // get the rotation from the ground plane to the plane with y-facing
    // vertical downwards
    Eigen::Vector3d    verticalAxis = Eigen::Vector3d(0, -1, 0);
    Eigen::Quaterniond rotationQuaternion;
    rotationQuaternion.setFromTwoVectors(planeNormal, verticalAxis);
    planePose.linear() = rotationQuaternion.toRotationMatrix();

    // form homogenous transformation matrix
    Eigen::Matrix4f planePoseMatrix = planePose.matrix().cast<float>();
    planePoseMatrix(3, 3)           = 1.0;

    return planePoseMatrix;
}

} // namespace core
} // namespace vs_graphs
