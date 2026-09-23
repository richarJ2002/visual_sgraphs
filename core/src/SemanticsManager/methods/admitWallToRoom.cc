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

#include "../private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

bool SemanticsManager::admitWallToRoom(semantic::Room   *p_room_inout,
                                       geometric::Plane *p_candidateWall_in)
{
    if (p_room_inout == nullptr || p_room_inout->isBad() ||
        p_candidateWall_in == nullptr || p_candidateWall_in->isBad())
    {
        return false;
    }

    const std::vector<geometric::Plane *> existingWalls =
        p_room_inout->getWalls();
    const bool alreadyPresent =
        std::find(existingWalls.begin(),
                  existingWalls.end(),
                  p_candidateWall_in) != existingWalls.end();

    geometric::Plane *p_farSideGroundPlane = p_atlas->getBiggestGroundPlane();
    Eigen::Vector3d   farSideGroundNormal_World = Eigen::Vector3d::Zero();
    if (p_farSideGroundPlane != nullptr && !p_farSideGroundPlane->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_farSideGroundPlane->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            farSideGroundNormal_World = groundEq.head<3>() / groundNorm;
        }
    }

    std::vector<semantic::Passage *> allPassages = p_atlas->getAllPassages();
    std::sort(
        allPassages.begin(),
        allPassages.end(),
        [](const semantic::Passage *p_first, const semantic::Passage *p_second)
        {
            if (p_first == nullptr)
            {
                return false;
            }
            if (p_second == nullptr)
            {
                return true;
            }
            return p_first->getId() < p_second->getId();
        });

    switch (enforcePassageApertureBackstop(p_room_inout,
                                           p_candidateWall_in,
                                           allPassages,
                                           farSideGroundNormal_World))
    {
    case PassageSideEnforcementOutcome::RemovedUnbound:
        return false;
    case PassageSideEnforcementOutcome::Rerouted:
        return true;
    case PassageSideEnforcementOutcome::NoViolation:
        break;
    }

    if (alreadyPresent)
    {
        return true;
    }

    if (!evaluateWallAdmissionEvidence(p_candidateWall_in,
                                       p_sysParams,
                                       farSideGroundNormal_World)
             .admissible)
    {
        return false;
    }

    if (isWallFaceForeignToRoom(p_room_inout, p_candidateWall_in))
    {
        std::cout << "[SemMgr] Wall#" << p_candidateWall_in->getId()
                  << " rejected from semantic::Room#" << p_room_inout->getId()
                  << ": this face was observed from the opposite side, so it "
                     "bounds the neighbouring room."
                  << std::endl;
        return false;
    }

    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_sysParams->roomSeg.boundaryTopology;
    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

    if (!topologyParameters.enabled || p_groundPlane == nullptr ||
        p_groundPlane->isBad())
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_World =
        groundNormal_World.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_World =
        groundNormal_World.cross(groundAxisU_World).normalized();
    FiniteWallSegment2d candidateSegment;

    if (!buildFiniteWallSegment2d(p_candidateWall_in,
                                  groundNormal_World,
                                  groundAxisU_World,
                                  groundAxisV_World,
                                  topologyParameters.endpointTrimRatio,
                                  topologyParameters.minimumWallLength_m,
                                  candidateSegment))
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    std::vector<geometric::Plane *> weakerClashingWalls;

    for (geometric::Plane *p_existingWall : existingWalls)
    {
        FiniteWallSegment2d existingSegment;

        if (!buildFiniteWallSegment2d(p_existingWall,
                                      groundNormal_World,
                                      groundAxisU_World,
                                      groundAxisV_World,
                                      topologyParameters.endpointTrimRatio,
                                      topologyParameters.minimumWallLength_m,
                                      existingSegment))
        {
            continue;
        }

        Eigen::Vector2d intersection_World_m;
        double          candidateParameter = 0.0;
        double          existingParameter  = 0.0;

        if (!intersectSupportingLines(candidateSegment,
                                      existingSegment,
                                      intersection_World_m,
                                      candidateParameter,
                                      existingParameter) ||
            candidateParameter < 0.0 || candidateParameter > 1.0 ||
            existingParameter < 0.0 || existingParameter > 1.0)
        {
            continue;
        }

        const double candidateInteriorDistance_m =
            std::min(candidateParameter, 1.0 - candidateParameter) *
            candidateSegment.length_m;
        const double existingInteriorDistance_m =
            std::min(existingParameter, 1.0 - existingParameter) *
            existingSegment.length_m;

        if (std::max(candidateInteriorDistance_m, existingInteriorDistance_m) <=
            topologyParameters.maximumInteriorIntersection_m)
        {
            continue;
        }

        const double candidateSupport = candidateSegment.supportScore;
        const double existingSupport  = existingSegment.supportScore;
        const double weakerSupport =
            std::max(std::min(candidateSupport, existingSupport), 1e-8);
        const double supportRatio =
            std::max(candidateSupport, existingSupport) / weakerSupport;

        /*
         * Ambiguous or weaker candidates stay outside this room. Their Plane
         * objects remain valid and the orphan-wall pass can attach them to a
         * different room as additional free-space evidence becomes available.
         */
        if (supportRatio < topologyParameters.decisiveConflictSupportRatio ||
            candidateSupport <= existingSupport)
        {
            return false;
        }

        weakerClashingWalls.push_back(p_existingWall);
    }

    for (geometric::Plane *p_weakerWall : weakerClashingWalls)
    {
        if (p_room_inout->removeWall(p_weakerWall))
        {
            std::cout << "[SemMgr] Replaced clashing Wall#"
                      << p_weakerWall->getId() << " in semantic::Room#"
                      << p_room_inout->getId() << " with stronger Wall#"
                      << p_candidateWall_in->getId() << "." << std::endl;
        }
    }

    /* Cross-room check: real walls only meet at shared corners -- a
     * candidate whose finite segment decisively crosses the interior of
     * another room's already-admitted wall is a modeling error, not a
     * legitimate admission. (Twin faces from reconcileWallFacePairs() are
     * parallel by construction and cannot trigger this.) Reject outright
     * rather than perturb the foreign room's wall: ownership of an
     * already-admitted wall is never taken by another room's admission
     * attempt, only by enforceUniqueWallOwnership()/validateRoomBoundaries()'
     * own intra-room repair. */
    for (vs_graphs::core::semantic::Room *p_otherRoom : p_atlas->getAllRooms())
    {
        if (p_otherRoom == nullptr || p_otherRoom->isBad() ||
            p_otherRoom == p_room_inout)
        {
            continue;
        }

        for (geometric::Plane *p_otherWall : p_otherRoom->getWalls())
        {
            if (p_otherWall == nullptr || p_otherWall == p_candidateWall_in)
            {
                continue;
            }

            FiniteWallSegment2d otherRoomSegment;

            if (!buildFiniteWallSegment2d(
                    p_otherWall,
                    groundNormal_World,
                    groundAxisU_World,
                    groundAxisV_World,
                    topologyParameters.endpointTrimRatio,
                    topologyParameters.minimumWallLength_m,
                    otherRoomSegment))
            {
                continue;
            }

            Eigen::Vector2d otherIntersection_World_m;
            double          candidateOtherParameter = 0.0;
            double          otherRoomParameter      = 0.0;

            if (!intersectSupportingLines(candidateSegment,
                                          otherRoomSegment,
                                          otherIntersection_World_m,
                                          candidateOtherParameter,
                                          otherRoomParameter) ||
                candidateOtherParameter < 0.0 ||
                candidateOtherParameter > 1.0 || otherRoomParameter < 0.0 ||
                otherRoomParameter > 1.0)
            {
                continue;
            }

            const double candidateOtherInteriorDistance_m =
                std::min(candidateOtherParameter,
                         1.0 - candidateOtherParameter) *
                candidateSegment.length_m;
            const double otherRoomInteriorDistance_m =
                std::min(otherRoomParameter, 1.0 - otherRoomParameter) *
                otherRoomSegment.length_m;

            if (std::max(candidateOtherInteriorDistance_m,
                         otherRoomInteriorDistance_m) <=
                topologyParameters.maximumInteriorIntersection_m)
            {
                continue;
            }

            std::cout << "[SemMgr] Wall#" << p_candidateWall_in->getId()
                      << " rejected from semantic::Room#"
                      << p_room_inout->getId() << ": crosses semantic::Room#"
                      << p_otherRoom->getId() << "'s already-admitted Wall#"
                      << p_otherWall->getId() << "." << std::endl;
            return false;
        }
    }

    p_room_inout->setWalls(p_candidateWall_in);
    return true;
}

} // namespace core
} // namespace vs_graphs
