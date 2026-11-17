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

/*!
 * @file            resolveBootstrapCameraPosition.cc
 *
 * @brief           Implements
 *                  SemanticsManager::resolveBootstrapCameraPosition(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{

std::optional<Eigen::Vector3d> SemanticsManager::resolveBootstrapCameraPosition(
    map::Map                             *p_activeMap_in,
    const std::optional<Eigen::Vector3d> &cameraPositionOverride_world_m_in,
    const std::optional<semantic::RoomContextSnapshot> &recoveryContext_in)
    const
{
    if (cameraPositionOverride_world_m_in.has_value() &&
        cameraPositionOverride_world_m_in->allFinite())
    {
        return cameraPositionOverride_world_m_in;
    }

    /* Refreshed at the start of this cycle; an exactly-zero centre is never
     * accepted there (refreshCurrentCameraCenter()): accepting it misplaces
     * the bootstrap room and poisons centroid-distance matching for the
     * cycles until walls correct it. */
    if (hasCameraCenter && p_cameraCenterMap == p_activeMap_in)
    {
        return currentCameraCenter_world_m;
    }

    /* Same-map reset clears keyframes, so no camera pose exists yet while a
     * valid recovery snapshot does. Recreate the last-known hierarchy at the
     * snapshot centroid now (refined once keyframes return) rather than
     * yielding the cycle to a free-space room with a fresh id. */
    if (recoveryContext_in.has_value() &&
        recoveryContext_in->centroid.allFinite())
    {
        return recoveryContext_in->centroid;
    }
    return std::nullopt;
}

} /* namespace core */
} /* namespace vs_graphs */
