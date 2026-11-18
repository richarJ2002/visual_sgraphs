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
 * @file            isSeenThroughSolidWall.cc
 *
 * @brief           Implements the solid-wall visibility rule.
 */

#include "scene_understanding/semantic_constraints/visibility_rules/public_functions/public_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isSeenThroughSolidWall(const VisibilityRuleInput &visibilityInput_in,
                            const types::SystemParams &systemParams_in,
                            geometric::Plane         *&p_blockingWall_out)
{
    p_blockingWall_out = nullptr;
    if (!systemParams_in.roomSeg.isSeeThroughGateEnabled ||
        !visibilityInput_in.isCameraViewpoint)
    {
        return false;
    }

    /* Blockers: the room's walls, then every mapped plane whose leading vote
     * is WALL before its type is confirmed. */
    const std::vector<geometric::Plane *> &roomWalls =
        *visibilityInput_in.p_roomWalls;
    std::vector<geometric::Plane *> blockerWalls = roomWalls;
    if (visibilityInput_in.p_mapPlanes != nullptr)
    {
        for (geometric::Plane *p_mappedPlane : *visibilityInput_in.p_mapPlanes)
        {
            bool                           mappedPlaneIsBad{true};
            geometric::Plane::PlaneVariant mappedPlaneType{};
            geometric::Plane::PlaneVariant leadingPlaneType{};
            if (p_mappedPlane == nullptr ||
                p_mappedPlane->isBad(mappedPlaneIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                mappedPlaneIsBad ||
                p_mappedPlane->getPlaneType(mappedPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                p_mappedPlane->getExpectedPlaneType(leadingPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                continue;
            }
            if (mappedPlaneType != geometric::Plane::PlaneVariant::WALL &&
                leadingPlaneType == geometric::Plane::PlaneVariant::WALL &&
                std::find(roomWalls.begin(), roomWalls.end(), p_mappedPlane) ==
                    roomWalls.end())
            {
                blockerWalls.push_back(p_mappedPlane);
            }
        }
    }

    for (geometric::Plane *p_nearerWall : blockerWalls)
    {
        if (p_nearerWall == nullptr ||
            p_nearerWall == visibilityInput_in.p_candidateWall ||
            p_nearerWall == visibilityInput_in.p_candidateTwinWall)
        {
            continue;
        }
        if (hasSeparatingFiniteWall(
                {p_nearerWall},
                visibilityInput_in.roomViewpoint_world_m,
                visibilityInput_in.candidateCentroid_world_m,
                static_cast<double>(systemParams_in.semSeg.reassociate
                                        .wallExtension.maximumInPlaneGap_m),
                true))
        {
            p_blockingWall_out = p_nearerWall;
            return true;
        }
    }
    return false;
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
