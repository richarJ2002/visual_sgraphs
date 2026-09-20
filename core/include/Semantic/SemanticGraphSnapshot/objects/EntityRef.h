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
 * @file            EntityRef.h
 *
 * @brief           Declares a single optional reference to another snapshot
 *                  entity, paired with why it is absent when it is.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_ENTITY_REF_H
#define SEMANTIC_GRAPH_SNAPSHOT_ENTITY_REF_H

#include <optional>

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       A single optional reference to another snapshot entity, paired
 *              with why it is absent when it is.
 *
 *              Used for every one-to-one relationship field that could
 *              plausibly point either at nothing (an ordinary, expected state
 *              for most of these fields) or at an object this snapshot could
 *              not safely key (a non-null but unmapped referenced object).
 *              Also used as the element type of every one-to-many
 *              Room/Passage/Floor relationship collection
 * (RoomRecord::passageRefs, FloorRecord::roomRefs, WallRecord::ownerRoomRefs),
 * replacing a bare std::vector<EntityKey> so a keyed-but-bad or
 *              keyed-but-missing-from-enumeration collection member no
 *              longer loses its own liveness/local-identity evidence.
 *
 *              Invariant: key.has_value() is equivalent to
 *              reason == UnavailableReason::NONE. A default-constructed
 *              EntityRef therefore defaults reason to NULL_REFERENCE, never
 *              NONE -- an untouched field must never silently read as "a
 *              value is present."
 *
 *              Separately, localId.has_value() is true whenever the
 *              underlying model pointer was non-null at capture time,
 *              independent of key/reason -- so an unmapped non-null target
 *              (reason == ENTITY_HAS_NO_MAP) still retains its own local id,
 *              rather than degrading to the same evidence as a genuinely
 *              null pointer. localId is always populated before key,
 *              whether or not key ends up populated too.
 *
 *              Liveness is a tri-state, independent of key/localId:
 *              isLive.has_value() is equivalent to
 *              livenessUnavailableReason == UnavailableReason::NONE.
 *              - Null reference (localId absent): isLive is absent and
 *                livenessUnavailableReason is NULL_REFERENCE.
 *              - Non-null target whose type exposes isBad() (Room,
 *                Passage): isLive holds the actual !isBad() value and
 *                livenessUnavailableReason is NONE.
 *              - Non-null target whose type has no isBad() at all (Floor):
 *                isLive is explicitly absent and livenessUnavailableReason
 *                is NOT_TRACKED_BY_CURRENT_SCHEMA -- unknown liveness is
 *                never encoded as true.
 *              Liveness is captured directly from the referenced pointer,
 *              independent of whether that entity is present in this
 *              snapshot's own enumerated record collections, so a target
 *              missing from enumeration (e.g. erased from its map's
 *              collections elsewhere) still reports truthful liveness here
 *              rather than requiring an unreliable cross-reference lookup.
 */
struct EntityRef
{
  public:
    /*! @brief The referenced entity's key, present only when reason ==
     *  UnavailableReason::NONE. */
    std::optional<EntityKey> key;

    /*! @brief Why key is empty; UnavailableReason::NONE exactly when key
     *  has a value. */
    UnavailableReason reason{UnavailableReason::NULL_REFERENCE};

    /*! @brief The referenced entity's own local id (Room::getId(),
     *  Passage::getId(), or Floor::getId()), present whenever the
     *  underlying pointer was non-null at capture time -- including when it
     *  had no map and key could therefore not be formed. Absent exactly
     *  when the underlying pointer was nullptr. */
    std::optional<int> localId;

    /*! @brief Inverse of the referenced entity's own isBad() at capture
     *  time, present only when livenessUnavailableReason ==
     *  UnavailableReason::NONE. Never true merely because liveness is
     *  unknown -- see livenessUnavailableReason. */
    std::optional<bool> isLive;

    /*! @brief Why isLive is absent; UnavailableReason::NONE exactly when
     *  isLive has a value. NULL_REFERENCE when localId is also absent (no
     *  target at all); NOT_TRACKED_BY_CURRENT_SCHEMA when localId is
     *  present but the referenced type has no isBad() API (Floor). */
    UnavailableReason livenessUnavailableReason{
        UnavailableReason::NULL_REFERENCE};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_ENTITY_REF_H
