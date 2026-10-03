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
 * @file            computeLegacyMapCompleteness.cc
 *
 * @brief           Implements computeLegacyMapCompleteness(), declared in
 *                  private_functions.h.
 *
 *                  Reproduces core/src/SemanticsManager.cc's "MAP
 *                  COMPLETENESS" block field-for-field:
 *                  - confirmedRoomCount/completeRoomCount/incompleteRoomIds
 *                    iterate Map::GetAllRooms()'s own documented behaviour
 *                    (Map::GetAllDetectedMapRooms() concatenated with
 *                    Map::GetAllMarkerBasedMapRooms(), so a room present in
 *                    both is visited twice), skipping any entry that is
 *                    null, bad, or not RoomRecord::variant == ROOM -- here
 *                    reproduced as a per-room multiplicity
 *                    (isDetectedMember + isMarkerBasedMember) applied to
 *                    every count/list this room contributes to, since
 *                    RoomRecord already deduplicates each Room pointer to
 *                    one record;
 *                  - passageCount/fullyLinkedPassageCount/danglingPassageIds
 *                    iterate live passages, requiring both the known-side
 *                    and prospective room to be non-null, live, and
 *                    RoomRecord::variant == ROOM, with no map check at all
 *                    (reproduced via resolveRoomEndpoint() against the
 *                    whole snapshot, ignoring its isCrossMap output exactly
 *                    as the live code ignores map membership here); and
 *                  - isMapFullyModeled combines both exactly as the legacy
 *                    boolean expression does.
 *
 *                  Known residual gap in the "exact reproduction" claim:
 *                  the live legacy code dereferences a passage's known-
 *                  side/prospective Room pointer directly, so it needs no
 *                  enumeration at all to read isBad()/getRoomVariant().
 *                  resolveRoomEndpoint() cannot do the same for variant
 *                  (RoomRecord::variant has no EntityRef-level counterpart
 *                  -- see its own Doxygen), so a room that is live,
 *                  ROOM-variant, and genuinely referenced, but that this
 *                  snapshot cannot locate in any captured map's RoomRecord
 *                  collection (isFoundInSnapshot == false), is reproduced
 *                  here as not-real even though live legacy code would
 *                  count it. This requires the referenced room to exist
 *                  without being enumerated in any live map's Detected/
 *                  MarkerBased collection at all, which is not expected in
 *                  ordinary operation; closing it fully would require adding
 *                  variant to EntityRef, a schema change out of scope for this
 *                  function.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus computeLegacyMapCompleteness(
    const SemanticGraphSnapshot &snapshot_in,
    const MapSnapshot           &mapSnapshot_in,
    LegacyMapCompletenessResult &legacyMapCompleteness_out)
{
    LegacyMapCompletenessResult result;

    for (const RoomRecord &room : mapSnapshot_in.rooms)
    {
        const std::size_t multiplicity = (room.isDetectedMember ? 1U : 0U) +
                                         (room.isMarkerBasedMember ? 1U : 0U);
        if (multiplicity == 0U || !room.isLive ||
            room.variant != Room::RoomVariant::ROOM)
        {
            continue;
        }

        result.confirmedRoomCount += multiplicity;
        if (room.boundaryStatus == Room::BoundaryStatus::COMPLETE)
        {
            result.completeRoomCount += multiplicity;
        }
        else
        {
            for (std::size_t repeatIndex = 0U; repeatIndex < multiplicity;
                 ++repeatIndex)
            {
                result.incompleteRoomIds.push_back(room.key.entityId);
            }
        }
    }

    for (const PassageRecord &passage : mapSnapshot_in.passages)
    {
        if (!passage.isLive)
        {
            continue;
        }
        result.passageCount++;

        ResolvedRoomEndpoint knownSide{};
        if (resolveRoomEndpoint(passage.knownSideRoomRef,
                                mapSnapshot_in.mapId,
                                snapshot_in,
                                knownSide) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resolveRoomEndpoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        ResolvedRoomEndpoint prospective{};
        if (resolveRoomEndpoint(passage.prospectiveRoomRef,
                                mapSnapshot_in.mapId,
                                snapshot_in,
                                prospective) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resolveRoomEndpoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool hasRealKnownSideRoom = knownSide.isFoundInSnapshot &&
                                          knownSide.isLive &&
                                          knownSide.isConfirmedRoomVariant;
        const bool hasRealFarSideRoom = prospective.isFoundInSnapshot &&
                                        prospective.isLive &&
                                        prospective.isConfirmedRoomVariant;

        if (hasRealKnownSideRoom && hasRealFarSideRoom)
        {
            result.fullyLinkedPassageCount++;
        }
        else
        {
            result.danglingPassageIds.push_back(passage.key.entityId);
        }
    }

    result.isMapFullyModeled =
        result.confirmedRoomCount > 0U &&
        result.completeRoomCount == result.confirmedRoomCount &&
        result.fullyLinkedPassageCount == result.passageCount;

    legacyMapCompleteness_out = result;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
