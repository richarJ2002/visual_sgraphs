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

std::optional<Floor::PlaneIdentity> Floor::transformPlaneIdentity(
    const PlaneIdentity &identity_OldWorld_in,
    const g2o::Sim3     &transform_oldWorldToNewWorld_in)
{
    Eigen::Vector4d equation_OldWorld = identity_OldWorld_in.equation_World;
    const double    oldNormalNorm     = equation_OldWorld.head<3>().norm();

    if (!equation_OldWorld.allFinite() || !std::isfinite(oldNormalNorm) ||
        oldNormalNorm < 1e-8)
    {
        return std::nullopt;
    }

    equation_OldWorld /= oldNormalNorm;

    const Eigen::Matrix3d rotation_oldWorldToNewWorld =
        transform_oldWorldToNewWorld_in.rotation().toRotationMatrix();
    const Eigen::Vector3d translation_NewWorld =
        transform_oldWorldToNewWorld_in.translation();
    const double scale = transform_oldWorldToNewWorld_in.scale();

    if (!rotation_oldWorldToNewWorld.allFinite() ||
        !translation_NewWorld.allFinite() || !std::isfinite(scale))
    {
        return std::nullopt;
    }

    const Eigen::Vector3d normal_NewWorld =
        rotation_oldWorldToNewWorld * equation_OldWorld.head<3>();
    Eigen::Vector4d equation_NewWorld;
    equation_NewWorld << normal_NewWorld,
        scale * equation_OldWorld(3) -
            normal_NewWorld.dot(translation_NewWorld);

    const double newNormalNorm = equation_NewWorld.head<3>().norm();
    if (!equation_NewWorld.allFinite() || !std::isfinite(newNormalNorm) ||
        newNormalNorm < 1e-8)
    {
        return std::nullopt;
    }

    equation_NewWorld /= newNormalNorm;
    return PlaneIdentity{equation_NewWorld,
                         identity_OldWorld_in.finiteSupportCount,
                         identity_OldWorld_in.observationCount};
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
