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

ImuTypesStatus
    inverseRightJacobianSO3(const float     &angleAxisX_in,
                            const float     &angleAxisY_in,
                            const float     &angleAxisZ_in,
                            Eigen::Matrix3f &inverseRightJacobian_out)
{
    Eigen::Matrix3f identityMatrix;
    identityMatrix.setIdentity();
    const float rotationAngleSquared = angleAxisX_in * angleAxisX_in +
                                       angleAxisY_in * angleAxisY_in +
                                       angleAxisZ_in * angleAxisZ_in;
    const float     rotationAngle = sqrt(rotationAngleSquared);
    Eigen::Vector3f angleAxisVector;
    angleAxisVector << angleAxisX_in, angleAxisY_in, angleAxisZ_in;
    Eigen::Matrix3f skewSymmetricMatrix = Sophus::SO3f::hat(angleAxisVector);

    if (rotationAngle < eps)
    {
        inverseRightJacobian_out = identityMatrix;
        return ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS;
    }
    else
    {
        inverseRightJacobian_out =
            identityMatrix + skewSymmetricMatrix / 2 +
            skewSymmetricMatrix * skewSymmetricMatrix *
                (1.0f / rotationAngleSquared -
                 (1.0f + cos(rotationAngle)) /
                     (2.0f * rotationAngle * sin(rotationAngle)));
        return ImuTypesStatus::IMU_TYPES_STATUS_SUCCESS;
    }
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
