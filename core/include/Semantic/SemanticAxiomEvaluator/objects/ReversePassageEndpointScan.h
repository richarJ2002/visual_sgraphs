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
 * @file            ReversePassageEndpointScan.h
 *
 * @brief           Declares the outcome of scanReversePassageEndpoints(): every
 *                  RoomRecord (live or retired) across the evaluated snapshot
 *                  whose own passageRefs names one specific passage, classified
 *                  by how trustworthy that reverse reference is.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_REVERSE_PASSAGE_ENDPOINT_SCAN_H
#define SEMANTIC_AXIOM_EVALUATOR_REVERSE_PASSAGE_ENDPOINT_SCAN_H

#include <vector>

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Every live RoomRecord found, by exhaustively scanning every
 *                  map's rooms (not only the two rooms a passage's own forward
 *                  knownSideRoomRef/prospectiveRoomRef name), whose independent
 *                  passageRefs collection names one evaluated passage -- see
 *                  scanReversePassageEndpoints.cc. Every key vector is sorted
 *                  ascending with no duplicates.
 */
struct ReversePassageEndpointScan
{
  public:
    /*!
     * @brief           Live, ROOM-variant, same-map rooms that cleanly and
     *                  trustworthily list the passage back. May include keys
     *                  the passage's own forward references already name -- the
     *                  caller reconciles that.
     */
    std::vector<EntityKey> confirmedReverseRoomKeys;

    /*!
     * @brief           Rooms whose passageRefs entry names the passage by key,
     *                  but whose own reference-level liveness for that entry
     *                  reports the passage as retired.
     */
    std::vector<EntityKey> badReverseRoomKeys;

    /*!
     * @brief           Rooms whose passageRefs entry names the passage by key,
     *                  but the room itself is declared in a different map than
     *                  the passage.
     */
    std::vector<EntityKey> crossMapReverseRoomKeys;

    /*!
     * @brief           Rooms whose key collides with another distinct
     *                  RoomRecord's key in the same map, discovered while
     *                  resolving a reverse passageRefs match: which RoomRecord
     *                  actually owns the relationship is ambiguous.
     */
    std::vector<EntityKey> duplicateIdentityRoomKeys;

    /*!
     * @brief           Live rooms with a passageRefs entry whose key shares
     *                  this passage's own map and entity id but a different
     *                  EntityKind: a wrong-kind key masquerading as a reference
     *                  to this passage.
     */
    std::vector<EntityKey> wrongKindReverseRoomKeys;

    /*!
     * @brief           Live, same-map rooms whose own passageRefs names this
     *                  passage more than once via otherwise-clean entries
     *                  (confirmed, prospective, or liveness-unavailable alike):
     *                  relationship multiplicity a single key-deduplication
     *                  pass must not silently erase, regardless of which of
     *                  those three clean categories the duplicated entries fall
     *                  into.
     */
    std::vector<EntityKey> duplicateReferenceRoomKeys;

    /*!
     * @brief           Live, same-map, non-ROOM-variant (prospective) rooms
     *                  that cleanly and trustworthily list the passage back:
     *                  represented here rather than silently discarded, even
     *                  though a prospective handle is never a "real"/confirmed
     *                  cardinality endpoint (see isRealPassageEndpoint()).
     */
    std::vector<EntityKey> prospectiveReverseRoomKeys;

    /*!
     * @brief           Live, same-map rooms whose passageRefs entry names the
     *                  passage by key with no independently-known
     *                  kind/map/identity contradiction, but whose own
     *                  EntityRef::isLive carries no value at all (genuinely
     *                  unproven liveness, distinct from an explicit isLive ==
     *                  false in badReverseRoomKeys): "missing liveness is
     *                  unavailable, not live" -- never silently counted as a
     *                  confirmed reciprocal endpoint.
     */
    std::vector<EntityKey> livenessUnavailableReverseRoomKeys;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_REVERSE_PASSAGE_ENDPOINT_SCAN_H
