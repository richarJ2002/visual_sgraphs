/**
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
 * @file            canonicalRoomFloorResultFor.cc
 *
 * @brief           Implements canonicalRoomFloorResultFor(), declared in
 *                  private_functions.h.
 *
 *                  Checkpoint-A residual repair (checkpoint 10): replaces
 *                  hasFailingRoomFloorReciprocity()'s lossy bool return
 *                  (FAIL-or-not) with the full aggregate AxiomResult
 *                  (FAIL/UNKNOWN/PASS) of the endpoint room's own canonical
 *                  evaluateOneRoomFloorReciprocity() findings, so
 *                  evaluatePassageFloorAgreement() can propagate a canonical
 *                  UNKNOWN (not only FAIL) before ever comparing floor keys.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

AxiomResult
    canonicalRoomFloorResultFor(const ResolvedRoomEndpoint  &endpoint_in,
                                const SemanticGraphSnapshot &snapshot_in)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != endpoint_in.key->mapId)
        {
            continue;
        }
        const RoomRecord *p_room =
            findRecordByKey(mapSnapshot.rooms, *endpoint_in.key);
        if (p_room == nullptr)
        {
            return AxiomResult::PASS;
        }
        std::vector<Finding> scratch;
        evaluateOneRoomFloorReciprocity(*p_room,
                                        snapshot_in,
                                        mapSnapshot,
                                        scratch);
        if (anyFindingIs(scratch, AxiomResult::FAIL))
        {
            return AxiomResult::FAIL;
        }
        if (anyFindingIs(scratch, AxiomResult::UNKNOWN))
        {
            return AxiomResult::UNKNOWN;
        }
        return AxiomResult::PASS;
    }
    return AxiomResult::PASS;
}

} // namespace semantic
} // namespace ORB_SLAM3
