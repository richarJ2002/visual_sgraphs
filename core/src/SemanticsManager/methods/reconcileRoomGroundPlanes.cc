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

#include "SemanticsManager.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::reconcileRoomGroundPlanes(void)
{
    Map *p_currentMap = p_atlas->getCurrentMap();
    if (p_currentMap == nullptr)
    {
        return;
    }

    std::vector<vs_graphs::core::semantic::Floor *> floors =
        p_currentMap->getAllFloors();
    semantic::Floor *p_canonicalFloor =
        semantic::Floor::selectBestObservedFloor(floors);
    if (p_canonicalFloor == nullptr || !p_canonicalFloor->hasPlaneIdentity())
    {
        /* No canonical identity to reconcile against yet. */
        return;
    }

    const std::optional<semantic::Floor::PlaneIdentity> canonicalIdentity =
        p_canonicalFloor->getPlaneIdentity();
    if (!canonicalIdentity.has_value())
    {
        return;
    }

    geometric::Plane *p_canonicalGroundPlane =
        p_currentMap->getBiggestGroundPlane();

    for (vs_graphs::core::semantic::Room *p_room :
         p_atlas->getAllDetectedMapRooms())
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        geometric::Plane *p_roomGroundPlane = p_room->getGroundPlane();
        if (p_roomGroundPlane == nullptr || p_roomGroundPlane->isBad() ||
            p_roomGroundPlane == p_canonicalGroundPlane)
        {
            /* Nothing to reconcile: no ground plane yet, or already the
             * canonical one. */
            continue;
        }

        const geometric::Plane::GeometrySnapshot roomGroundGeometry =
            p_roomGroundPlane->getGeometrySnapshot();
        const double roomGroundNormalNorm =
            roomGroundGeometry.equation_World.head<3>().norm();
        if (roomGroundGeometry.cloudGeneration !=
                roomGroundGeometry.successfulRefitGeneration ||
            roomGroundGeometry.finiteSupportCount == 0U ||
            !roomGroundGeometry.equation_World.allFinite() ||
            !std::isfinite(roomGroundNormalNorm) ||
            std::abs(roomGroundNormalNorm - 1.0) > 1e-3)
        {
            /* Room's ground plane geometry isn't settled yet -- nothing
             * reliable to compare. */
            continue;
        }

        const semantic::Floor::PlaneIdentity roomIdentity{
            roomGroundGeometry.equation_World,
            roomGroundGeometry.finiteSupportCount,
            roomGroundGeometry.observationCount};

        double normalAngle_deg = 0.0;
        double offset_m        = 0.0;
        if (semantic::Floor::planeIdentitiesMatch(
                canonicalIdentity.value(),
                roomIdentity,
                semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
                semantic::Floor::kMergeMaxPlaneOffset_m,
                normalAngle_deg,
                offset_m))
        {
            /* Within tolerance -- nothing to reconcile. */
            continue;
        }

        /* A real flatness disagreement. Re-point the less-observed side to
         * the canonical plane -- a pure pointer rewire, never a geometry
         * mutation, so it can never fight a plane's own cloud refit. */
        if (roomIdentity.observationCount <=
                canonicalIdentity->observationCount &&
            p_canonicalGroundPlane != nullptr)
        {
            p_room->setGroundPlane(p_canonicalGroundPlane);
            std::cout << "[SemMgr] semantic::Room#" << p_room->getId()
                      << "'s ground plane disagreed with semantic::Floor#"
                      << p_canonicalFloor->getId()
                      << "'s canonical level (normal " << normalAngle_deg
                      << " deg, offset " << offset_m
                      << " m) -- re-pointed to the canonical plane."
                      << std::endl;
        }
        else
        {
            std::cout
                << "[SemMgr] semantic::Room#" << p_room->getId()
                << "'s ground plane is more observed than semantic::Floor#"
                << p_canonicalFloor->getId()
                << "'s current canonical level (normal " << normalAngle_deg
                << " deg, offset " << offset_m
                << " m) -- left as-is; the floor will re-select its "
                   "canonical identity next cycle."
                << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
