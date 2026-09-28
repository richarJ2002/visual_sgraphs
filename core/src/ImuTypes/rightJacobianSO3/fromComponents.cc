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

#include "ImuTypes.h"

namespace vs_graphs
{
namespace core
{
namespace IMU
{

Eigen::Matrix3f rightJacobianSO3(const float &rotationVectorX_in,
                                 const float &rotationVectorY_in,
                                 const float &rotationVectorZ_in)
{
    Eigen::Matrix3f identityMatrix;
    identityMatrix.setIdentity();
    const float angleSquared = rotationVectorX_in * rotationVectorX_in +
                               rotationVectorY_in * rotationVectorY_in +
                               rotationVectorZ_in * rotationVectorZ_in;
    const float     rotationAngle = sqrt(angleSquared);
    Eigen::Vector3f rotationVector;
    rotationVector << rotationVectorX_in, rotationVectorY_in,
        rotationVectorZ_in;
    Eigen::Matrix3f skewMatrix = Sophus::SO3f::hat(rotationVector);
    if (rotationAngle < eps)
    {
        return identityMatrix;
    }
    else
    {
        return identityMatrix -
               skewMatrix * (1.0f - cos(rotationAngle)) / angleSquared +
               skewMatrix * skewMatrix * (rotationAngle - sin(rotationAngle)) /
                   (angleSquared * rotationAngle);
    }
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
