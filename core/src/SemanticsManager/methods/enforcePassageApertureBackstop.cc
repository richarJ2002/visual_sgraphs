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
#include <cmath>

namespace vs_graphs
{
namespace core
{

SemanticsManager::PassageSideEnforcementOutcome
    SemanticsManager::enforcePassageApertureBackstop(
        semantic::Room                         *p_room_inout,
        geometric::Plane                       *p_wall_in,
        const std::vector<semantic::Passage *> &allPassages_in,
        const Eigen::Vector3d                  &groundNormal_World_in)
{
    if (p_room_inout == nullptr || p_room_inout->isBad() ||
        p_wall_in == nullptr || p_wall_in->isBad())
    {
        return PassageSideEnforcementOutcome::NO_VIOLATION;
    }

    for (semantic::Passage *p_passage : allPassages_in)
    {
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }

        const double minimumSideDistance_m = static_cast<double>(
            p_sysParams->roomSeg.passagePartition.minimumSideDistance_m);

        /* B2 fix: segmentCrossesPassageOpening silently reports "no crossing"
         * whenever its segment-start point sits within minimumSideDistance_m
         * of the passage plane -- which the room's own centroid commonly
         * does for a sparsely-observed room. Rather than let that ambiguity
         * masquerade as "not crossing" (silently admitting a genuine
         * far-side wall to the near room), substitute a point pushed out
         * along the passage's known near side when the raw centroid is too
         * close to call. Only apply this when a reliable near-side direction
         * is actually available (Passage::KnownSideProvenance, built up from
         * other walls' admission history for this passage): the ambiguous
         * centroid's own residual sign is noise, not a signal, and guessing
         * from it can just as easily push the synthesized point to the
         * WRONG side as the right one -- worse than the original silent
         * no-crossing report, not better. With no known side yet, this
         * degenerate case is left exactly as before the fix. */
        Eigen::Vector3d segmentStart_World_m = p_room_inout->getCentroid();
        Eigen::Vector4d passageEquation_World =
            p_passage->getGlobalEquation().coeffs();
        const double passageNormalNorm = passageEquation_World.head<3>().norm();
        if (passageEquation_World.allFinite() && passageNormalNorm > 1e-8)
        {
            passageEquation_World /= passageNormalNorm;
            const Eigen::Vector3d passageNormal_World =
                passageEquation_World.head<3>();
            const double roomCentroidSide_m =
                passageNormal_World.dot(segmentStart_World_m) +
                passageEquation_World(3);

            if (std::abs(roomCentroidSide_m) < minimumSideDistance_m)
            {
                const semantic::Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (knownSide.hasDirection())
                {
                    segmentStart_World_m = p_passage->getCentroid() +
                                           (minimumSideDistance_m * 2.0) *
                                               knownSide.direction_World;
                }
            }
        }

        if (!segmentCrossesPassageOpening(
                segmentStart_World_m,
                p_wall_in->getCentroid().cast<double>(),
                p_passage,
                groundNormal_World_in,
                static_cast<double>(
                    p_sysParams->roomSeg.passagePartition.openingMargin_m),
                minimumSideDistance_m))
        {
            continue;
        }

        /* Never steal a wall already claimed by a distinct confirmed room. */
        bool ownedByConfirmedRoom = false;
        for (vs_graphs::core::semantic::Room *p_other : p_atlas->getAllRooms())
        {
            if (p_other == nullptr || p_other->isBad() ||
                p_other == p_room_inout ||
                p_other->getRoomVariant() ==
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
            {
                continue;
            }
            const std::vector<geometric::Plane *> otherWalls =
                p_other->getWalls();
            if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
                otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            /* A distinct confirmed room already owns this wall. Leave it on
             * that owner rather than re-binding it to the near room. */
            p_room_inout->removeWall(p_wall_in);
            return PassageSideEnforcementOutcome::REMOVED_UNBOUND;
        }

        vs_graphs::core::semantic::Room *p_prospective =
            p_passage->getProspectiveRoom();

        /* The wall is already sitting in the room this exact aperture
         * crossing would route it to -- there is nothing to enforce. Live-
         * observed 2026-09-04: this branch previously fell through the same
         * eviction as "no prospective room exists at all", so a wall that
         * had ALREADY been correctly rerouted to its far-side prospective
         * kept getting evicted from it every single cycle this sweep re-ran
         * (enforcePassageSideInvariant runs every Run() cycle), leaving it
         * permanently homeless even though Passage#0's own SemMgrSummary
         * line showed a perfectly live p_prospectiveRoom the whole time. */
        if (p_prospective == p_room_inout)
        {
            continue;
        }

        if (p_prospective == nullptr || p_prospective->isBad())
        {
            p_room_inout->removeWall(p_wall_in);
            std::cout << "[SemMgr] Far-side Wall#" << p_wall_in->getId()
                      << " at semantic::Passage#" << p_passage->getId()
                      << " has no opposite stable room; left unbound."
                      << std::endl;
            return PassageSideEnforcementOutcome::REMOVED_UNBOUND;
        }

        p_room_inout->removeWall(p_wall_in);
        if (p_atlas->getRoomWallPlaneById(p_wall_in->getId()) == nullptr)
        {
            p_atlas->addRoomWallPlane(p_wall_in);
        }
        p_prospective->setWalls(p_wall_in);
        std::cout << "[SemMgr] Redirected far-side Wall#" << p_wall_in->getId()
                  << " to prospective semantic::Room#" << p_prospective->getId()
                  << "." << std::endl;
        return PassageSideEnforcementOutcome::REROUTED;
    }

    /* No CONFIRMED passage caught this wall -- but confirmation lags real
     * exploration time behind the skeleton-crossing evidence itself (see
     * segmentCrossesOpenPassageEvidence's own comment). Re-run the same
     * aperture test against each pending hypothesis so this continuous
     * re-check sweep (enforcePassageSideInvariant) catches a wall that slips
     * in during that window just as reliably as it catches one that slips in
     * against an already-confirmed passage. */
    for (const OpenPassageEvidence &evidence : openPassageEvidence)
    {
        if (!segmentCrossesOpenPassageEvidence(
                p_room_inout->getCentroid(),
                p_wall_in->getCentroid().cast<double>(),
                evidence.p_supportingWall,
                evidence.centroid_World_m,
                evidence.openingRadius_m,
                evidence.heightSpan_m,
                groundNormal_World_in,
                static_cast<double>(
                    p_sysParams->roomSeg.passagePartition.openingMargin_m),
                static_cast<double>(p_sysParams->roomSeg.passagePartition
                                        .minimumSideDistance_m)))
        {
            continue;
        }

        bool ownedByConfirmedRoom = false;
        for (vs_graphs::core::semantic::Room *p_other : p_atlas->getAllRooms())
        {
            if (p_other == nullptr || p_other->isBad() ||
                p_other == p_room_inout ||
                p_other->getRoomVariant() ==
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
            {
                continue;
            }
            const std::vector<geometric::Plane *> otherWalls =
                p_other->getWalls();
            if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
                otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            continue;
        }

        p_room_inout->removeWall(p_wall_in);
        std::cout << "[SemMgr] Far-side Wall#" << p_wall_in->getId()
                  << " crosses an unconfirmed passage opening (evidence at "
                     "wall "
                  << (evidence.p_supportingWall != nullptr
                          ? evidence.p_supportingWall->getId()
                          : -1)
                  << "); removed from semantic::Room#" << p_room_inout->getId()
                  << " pending confirmation." << std::endl;
        return PassageSideEnforcementOutcome::REMOVED_UNBOUND;
    }

    return PassageSideEnforcementOutcome::NO_VIOLATION;
}

} // namespace core
} // namespace vs_graphs
