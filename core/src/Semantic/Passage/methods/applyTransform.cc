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

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void Passage::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::lock_guard<std::mutex> lock(geometryMutex);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    Eigen::Vector4d passageEquation = globalEquation.coeffs();
    const double    normalNorm      = passageEquation.head<3>().norm();

    if (std::isfinite(normalNorm) && normalNorm > 1e-8)
    {
        passageEquation /= normalNorm;

        const Eigen::Vector3d transformedNormal =
            transform_oldWorldToNewWorld_in.rotation().toRotationMatrix() *
            passageEquation.head<3>();

        const Eigen::Vector3d translation =
            transform_oldWorldToNewWorld_in.translation();

        const double transformedDistance =
            transform_oldWorldToNewWorld_in.scale() * passageEquation(3) -
            transformedNormal.dot(translation);

        globalEquation = g2o::Plane3D(Eigen::Vector4d(transformedNormal.x(),
                                                      transformedNormal.y(),
                                                      transformedNormal.z(),
                                                      transformedDistance));
    }

    if (knownSideProvenance.hasDirection())
    {
        const Eigen::Vector3d transformedDirection =
            transform_oldWorldToNewWorld_in.rotation().toRotationMatrix() *
            knownSideProvenance.direction_World;
        if (transformedDirection.allFinite() &&
            transformedDirection.norm() > 1e-8)
        {
            knownSideProvenance.direction_World =
                transformedDirection.normalized();
        }
    }

    const double absoluteScale =
        std::abs(transform_oldWorldToNewWorld_in.scale());

    width *= absoluteScale;
    height *= absoluteScale;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
