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

/*!
 * @file            applyPoseToPlane.cc
 *
 * @brief           Implements Utils::applyPoseToPlane(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

g2o::Plane3D Utils::applyPoseToPlane(const Eigen::Matrix4d &keyframePose_in,
                                     const g2o::Plane3D    &plane_in)
{
    Eigen::Vector4d v = plane_in.coeffs();
    Eigen::Vector4d v2;
    Eigen::Matrix3d R = keyframePose_in.block<3, 3>(0, 0);
    v2.head<3>()      = R * v.head<3>();
    v2(3) = v(3) - keyframePose_in.block<3, 1>(0, 3).dot(v2.head<3>());
    return g2o::Plane3D(v2);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
