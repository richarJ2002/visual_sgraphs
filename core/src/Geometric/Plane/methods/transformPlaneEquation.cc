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
 * @file            transformPlaneEquation.cc
 *
 * @brief           Implements Plane::transformPlaneEquation(), declared in
 *                  Geometric/Plane.h.
 */

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::transformPlaneEquation(
    const g2o::Plane3D &plane_in,
    const g2o::Sim3    &transform_oldWorldToNewWorld_in,
    g2o::Plane3D       &transformedEquation_out)
{
    /*!
     * Transform a plane equation:
     *
     *     n^T x + d = 0
     *
     * under a Sim3 transformation:
     *
     *     x' = s R x + t
     *
     * The transformed plane is:
     *
     *     n'^T x' + d' = 0
     *
     */

    const Eigen::Matrix3d rotation = transform_oldWorldToNewWorld_in.rotation()
                                         .toRotationMatrix()
                                         .cast<double>();

    const Eigen::Vector3d translation =
        transform_oldWorldToNewWorld_in.translation().cast<double>();

    const double scale = transform_oldWorldToNewWorld_in.scale();

    /* Original plane parameters */
    const Eigen::Vector3d normal = plane_in.normal().cast<double>();

    /* g2o::Plane3D::distance() returns -d; use the equation coefficient. */
    const double equationOffset = plane_in.coeffs()(3);

    /*!
     * Transform the normal. For a similarity transform, the scale does not
     * affect the direction of the normal.
     */
    const Eigen::Vector3d transformedNormal = rotation * normal;

    /*!
     * Transform the plane offset.
     *
     * x' = sRx + t
     *
     * therefore:
     *
     * d' = s*d - n'^T*t
     */
    const double transformedEquationOffset =
        scale * equationOffset - transformedNormal.dot(translation);

    g2o::Plane3D transformedPlane(Eigen::Vector4d(transformedNormal.x(),
                                                  transformedNormal.y(),
                                                  transformedNormal.z(),
                                                  transformedEquationOffset));

    transformedEquation_out = transformedPlane;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
