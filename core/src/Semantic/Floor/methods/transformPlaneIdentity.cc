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
    const PlaneIdentity                 &identity_oldWorld_in,
    const g2o::Sim3                     &transform_oldWorldToNewWorld_in,
    std::optional<Floor::PlaneIdentity> &transformedIdentity_out)
{
    Eigen::Vector4d equation_oldWorld = identity_oldWorld_in.equation_world;
    const double    oldNormalNorm     = equation_oldWorld.head<3>().norm();

    if (!equation_oldWorld.allFinite() || !std::isfinite(oldNormalNorm) ||
        oldNormalNorm < 1e-8)
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    equation_oldWorld /= oldNormalNorm;

    const Eigen::Matrix3d rotation_oldWorldToNewWorld =
        transform_oldWorldToNewWorld_in.rotation().toRotationMatrix();
    const Eigen::Vector3d translation_newWorld =
        transform_oldWorldToNewWorld_in.translation();
    const double scale = transform_oldWorldToNewWorld_in.scale();

    if (!rotation_oldWorldToNewWorld.allFinite() ||
        !translation_newWorld.allFinite() || !std::isfinite(scale))
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    const Eigen::Vector3d normal_newWorld =
        rotation_oldWorldToNewWorld * equation_oldWorld.head<3>();
    Eigen::Vector4d equation_newWorld;
    equation_newWorld << normal_newWorld,
        scale * equation_oldWorld(3) -
            normal_newWorld.dot(translation_newWorld);

    const double newNormalNorm = equation_newWorld.head<3>().norm();
    if (!equation_newWorld.allFinite() || !std::isfinite(newNormalNorm) ||
        newNormalNorm < 1e-8)
    {
        transformedIdentity_out = std::nullopt;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    equation_newWorld /= newNormalNorm;
    transformedIdentity_out =
        PlaneIdentity{equation_newWorld,
                      identity_oldWorld_in.finiteSupportCount,
                      identity_oldWorld_in.observationCount};
    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
