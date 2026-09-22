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

Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float     d2 = x * x + y * y + z * z;
    const float     d  = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if (d < eps)
    {
        return I;
    }
    else
    {
        return I - W * (1.0f - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
    }
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
