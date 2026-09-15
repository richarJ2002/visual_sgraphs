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
 * @file            CapabilityLevel.h
 *
 * @brief           Declares how completely the current SemanticGraphSnapshot
 *                  schema can prove one axiom code, independent of whether a
 *                  specific instance's evidence happens to be present.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_CAPABILITY_LEVEL_H
#define SEMANTIC_AXIOM_EVALUATOR_CAPABILITY_LEVEL_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Schema-level capability classification for one axiom code.
 *
 *              This describes the *evaluator's* structural ability to prove
 *              the axiom's contract, not any one fixture's outcome: a FULL
 *              axiom can still report UNKNOWN for a particular instance
 *              whose ordinary, nullable relationship (e.g. a room not yet
 *              linked to a floor) happens to be absent -- that is ordinary
 *              incomplete evidence, not a capability gap. PARTIAL/DEFERRED
 *              specifically mean at least one clause depends on a
 *              SemanticGraphSnapshot field that is *always*
 *              UnavailableReason for every current instance (e.g.
 *              WallRecord::quarantineReason, PassageRecord::endpointSlotReason,
 *              RoomRecord::creationProvenanceReason), or on before/after
 *              transition behaviour this static-snapshot evaluator does not
 *              implement in this slice.
 */
enum class CapabilityLevel : std::uint8_t
{
    /*! @brief Every clause of the axiom's contract is checkable from fields
     *  the schema genuinely, always populates. */
    FULL = 0U,

    /*! @brief Some clauses are checkable; at least one is permanently
     *  unprovable from the current schema. */
    PARTIAL = 1U,

    /*! @brief The axiom's substantive content depends on a field that is
     *  always unavailable, or on dynamic/transition logic not yet
     *  implemented; every finding for this code is UNKNOWN in this slice. */
    DEFERRED = 2U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_CAPABILITY_LEVEL_H
