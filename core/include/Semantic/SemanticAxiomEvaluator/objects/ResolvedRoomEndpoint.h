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
 * @file            ResolvedRoomEndpoint.h
 *
 * @brief           Declares the resolved state of one EntityRef that is
 *                  expected to name a Room, used by the AX-PASS-02/03/04 and
 *                  completeness evaluators (resolveRoomEndpoint()).
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_RESOLVED_ROOM_ENDPOINT_H
#define SEMANTIC_AXIOM_EVALUATOR_RESOLVED_ROOM_ENDPOINT_H

#include <optional>

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Resolved state of one EntityRef naming a Room, checked
 *              against an expected containing map id and the full
 *              SemanticGraphSnapshot (every live map, not only the
 *              expected one, so a genuine cross-map reference is still
 *              locatable and distinguishable from a reference this
 *              snapshot cannot locate at all).
 *
 *              "Real"/"confirmed" throughout this module's passage-endpoint
 *              evaluators means \c isLive && \c isConfirmedRoomVariant --
 *              see resolveRoomEndpoint.cc's Doxygen for why this documented
 *              proxy, not a not-yet-existing authoritative endpoint-slot
 *              field, is what this slice checks.
 */
struct ResolvedRoomEndpoint
{
  public:
    /*! @brief True when the source EntityRef itself named a key
     *  (EntityRef::reason == UnavailableReason::NONE). False for the
     *  ordinary "no such endpoint" case (no reference attempted) and for a
     *  non-null but unmapped target (see referenceUnresolvable). */
    bool referencePresent{false};

    /*! @brief True when a reference was attempted (the underlying pointer
     *  was non-null) but no key could be formed (EntityRef::reason ==
     *  UnavailableReason::ENTITY_HAS_NO_MAP). Always false when
     *  referencePresent is true. */
    bool referenceUnresolvable{false};

    /*! @brief The referenced room's key; present only when
     *  referencePresent. */
    std::optional<EntityKey> key;

    /*! @brief True when referencePresent and key->mapId differs from the
     *  expected map id passed to resolveRoomEndpoint(). Meaningful only
     *  when referencePresent. */
    bool isCrossMap{false};

    /*! @brief True when referencePresent and a RoomRecord with this exact
     *  key was found somewhere in the evaluated SemanticGraphSnapshot
     *  (any live map, not only the expected one). isConfirmedRoomVariant/
     *  floorKey are meaningful only when this is true; isLive is
     *  meaningful whenever referencePresent or referenceUnresolvable (see
     *  isLive below) regardless of this flag. */
    bool isFoundInSnapshot{false};

    /*! @brief The referenced room's own liveness, read directly from the
     *  source EntityRef::isLive (populated whenever the underlying pointer
     *  was non-null at capture time, independent of whether a key could be
     *  formed or whether this snapshot can enumerate the target in any
     *  captured map -- see EntityRef.h). Meaningful whenever
     *  referencePresent or referenceUnresolvable and isLiveAvailable.
     *  No longer filled or overridden from the
     *  found RoomRecord's own isLive when isFoundInSnapshot -- a genuinely
     *  unavailable reference-level liveness must never be manufactured from
     *  the target record; isFoundInSnapshot instead only ever populates
     *  isConfirmedRoomVariant/floorKey/isTargetDeclaredMapMismatch. */
    bool isLive{false};

    /*! @brief True when the found RoomRecord's variant ==
     *  Room::RoomVariant::ROOM (as opposed to UNDEFINED, a prospective
     *  handle not yet promoted); meaningful only when isFoundInSnapshot. */
    bool isConfirmedRoomVariant{false};

    /*! @brief The found RoomRecord's own floorRef.key, when that room has a
     *  floor link; present only when isFoundInSnapshot and that room's own
     *  floorRef resolves to a key. */
    std::optional<EntityKey> floorKey{};

    /*! @brief True when referencePresent and more than one distinct
     *  RoomRecord in the referenced key's own map shares that exact key:
     *  which room actually forms this endpoint is ambiguous, so it can
     *  never supply positive proof. Meaningful only when
     *  referencePresent. */
    bool isDuplicateIdentity{false};

    /*! @brief True when referencePresent and \c isLive is genuinely
     *  unproven (the source EntityRef::isLive carried no value at all --
     *  EntityRef::livenessUnavailableReason != UnavailableReason::NONE).
     *  \c isLive keeps its own default (false) in this case, so every
     *  caller must check this flag before treating \c isLive as a known
     *  fact: a keyed endpoint with missing liveness is unavailable proof,
     *  not a bad endpoint merely because \c isLive defaults to false. */
    bool isLiveAvailable{false};

    /*! @brief True when referencePresent and the referenced key's own
     *  EntityKind is not EntityKind::ROOM: a wrong-kind key masquerading as
     *  a room reference. Meaningful only when referencePresent. */
    bool isWrongKind{false};

    /*! @brief True when isFoundInSnapshot and the located RoomRecord's own
     *  declaredMapId disagrees with the map it was actually found in (its
     *  key's mapId): a record/declaration contradiction distinct from the
     *  reference itself being cross-map. Meaningful only when
     *  isFoundInSnapshot. */
    bool isTargetDeclaredMapMismatch{false};

    /*! @brief True when referencePresent (the source EntityRef::key had a
     *  value) but the source EntityRef::reason was not
     *  UnavailableReason::NONE -- a violation of EntityRef's own
     *  documented invariant (key.has_value() <=> reason == NONE). Snapshot
     *  records are adversarial value inputs and this invariant is not
     *  assumed to hold; a reference violating it is treated as a known
     *  contradiction, not an ordinary valid keyed reference. Meaningful
     *  only when referencePresent. */
    bool isReasonInconsistent{false};

    /*! @brief True when referencePresent and more than one MapSnapshot in
     *  the evaluated snapshot shares the referenced key's own mapId: which
     *  MapSnapshot is actually authoritative for that map id is itself
     *  ambiguous, so no first-match lookup may supply positive proof.
     *  Meaningful only when referencePresent. */
    bool isContainingMapAmbiguous{false};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_RESOLVED_ROOM_ENDPOINT_H
