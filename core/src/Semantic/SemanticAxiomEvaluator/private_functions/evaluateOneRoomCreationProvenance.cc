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
 * @file         evaluateOneRoomCreationProvenance.cc
 *
 * @brief        Implements evaluateOneRoomCreationProvenance(),
 *               declared in private_functions.h.
 *
 *               Shared per-room AX-ROOM-01 leaf used by both
 *               evaluateAxRoom01() and computeConservativeMapCompleteness()
 *               (mirroring AX-PASS-01's own evaluateOnePassageProvenance()
 *               split), reporting one UNKNOWN Finding with the room's own
 *               key. RoomRecord::creationProvenanceReason is
 *               always NOT_TRACKED_BY_CURRENT_SCHEMA (see
 *               RoomRecord.h), so this leaf can only ever report
 *               UNKNOWN; a future extension that adds a real
 *               provenance field on Room is the owner of resolving
 *               this to a positive or negative result.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOneRoomCreationProvenance(const RoomRecord     &room_in,
                                       std::vector<Finding> &findings_inout)
{
    findings_inout.push_back(
        makeFinding(AxiomCode::AX_ROOM_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::ROOM_CREATION_PROVENANCE_UNAVAILABLE,
                    {room_in.key}));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
