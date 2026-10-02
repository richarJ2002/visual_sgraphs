/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            transformPlaneIdentity.cc
 *
 * @brief           Implements Floor::transformPlaneIdentity(), declared in
 *                  Semantic/Floor.h.
 */

#include "Semantic/Floor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus Floor::transformPlaneIdentity(
    const PlaneIdentity &planeIdentity_oldWorld_in,
    const g2o::Sim3     &alignmentTransform_oldWorldToNewWorld_in,
    std::optional<Floor::PlaneIdentity> &transformedIdentity_out)
{
    Eigen::Vector4d planeEquation_oldWorld =
        planeIdentity_oldWorld_in.planeEquation_world;
    const double oldNormalNorm = planeEquation_oldWorld.head<3>().norm();

    if (!planeEquation_oldWorld.allFinite() || !std::isfinite(oldNormalNorm) ||
        oldNormalNorm < 1e-8)
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    planeEquation_oldWorld /= oldNormalNorm;

    const Eigen::Matrix3d alignmentRotation_oldWorldToNewWorld =
        alignmentTransform_oldWorldToNewWorld_in.rotation().toRotationMatrix();
    const Eigen::Vector3d alignmentTranslation_oldWorldToNewWorld =
        alignmentTransform_oldWorldToNewWorld_in.translation();
    const double scale = alignmentTransform_oldWorldToNewWorld_in.scale();

    if (!alignmentRotation_oldWorldToNewWorld.allFinite() ||
        !alignmentTranslation_oldWorldToNewWorld.allFinite() ||
        !std::isfinite(scale))
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    const Eigen::Vector3d planeNormal_newWorld =
        alignmentRotation_oldWorldToNewWorld * planeEquation_oldWorld.head<3>();
    Eigen::Vector4d planeEquation_newWorld;
    planeEquation_newWorld << planeNormal_newWorld,
        scale * planeEquation_oldWorld(3) -
            planeNormal_newWorld.dot(alignmentTranslation_oldWorldToNewWorld);

    const double newNormalNorm = planeEquation_newWorld.head<3>().norm();
    if (!planeEquation_newWorld.allFinite() || !std::isfinite(newNormalNorm) ||
        newNormalNorm < 1e-8)
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    planeEquation_newWorld /= newNormalNorm;
    transformedIdentity_out =
        PlaneIdentity{planeEquation_newWorld,
                      planeIdentity_oldWorld_in.finiteSupportCount,
                      planeIdentity_oldWorld_in.observationCount};
    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
