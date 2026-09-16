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
 * @file            evaluateRoomMalformedPassageReferences.cc
 *
 * @brief           Implements evaluateRoomMalformedPassageReferences(),
 *                  declared in private_functions.h.
 *
 *                  A room's own passageRefs entry with a local id but no
 *                  key is genuine evidence that *some* Passage object this
 *                  room references is malformed, but never proof of *which*
 *                  passage -- local ids are unique only within one map
 *                  (EntityKey.h) and are not themselves a map-qualified
 *                  identity. This function therefore runs once per map (not
 *                  once per evaluated passage) and reports the evidence
 *                  scoped to the room alone.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateRoomMalformedPassageReferences(
    const MapSnapshot    &mapSnapshot_in,
    std::vector<Finding> &findings_inout)
{
    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        if (!room.isLive)
        {
            continue;
        }
        bool anyMalformed = false;
        for (const EntityRef &passageRef : room.passageRefs)
        {
            if (!passageRef.key.has_value() && passageRef.localId.has_value())
            {
                anyMalformed = true;
                break;
            }
        }
        if (anyMalformed)
        {
            findings_inout.push_back(makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE,
                {room.key}));
        }
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
