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
 * @file            MapCompletenessResultStruct.h
 *
 * @brief           Declares one map's completeness result: the rooms and
 *                  passages still to explore, the conservative completeness
 *                  check, the legacy calculation and their divergence.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_MAPCOMPLETENESSRESULT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_MAPCOMPLETENESSRESULT_STRUCT_H

#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomResultEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/LegacyMapCompletenessResultStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/ReasonCodeEnum.h"

#include <cstddef>
#include <vector>

#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One map's completeness result, produced by
 *                  evaluateMapCompleteness() at the end of every semantic
 *                  cycle. incompleteRoomIds and frontierPassageIds come from
 *                  the scene-graph queries and drive the exploration output;
 *                  the conservative tri-state and the \c legacy reproduction
 *                  (LegacyMapCompletenessResult.h) are diagnostics that
 *                  change no map state.
 */
struct MapCompletenessResult
{
  public:
    /*!
     * @brief           Atlas::Map::GetId() of the map this result is for.
     */
    long unsigned int mapId{0U};

    /*!
     * @brief           Conservative semantic-completeness tri-state:
     *                  - FAIL: zero confirmed rooms, a live prospective room, a
     *                    confirmed room without COMPLETE boundary, a live
     *                    passage lacking two
     *                    distinct/live/confirmed/same-map/same-floor/reciprocal/COMPLETE
     *                    endpoints, or any other observable hard contradiction
     *                    reported by evaluateState() for an entity in this map.
     *                  - UNKNOWN: no contradiction found, but required proof
     *                    for at least one clause above is unavailable (see
     *                    \c reasons).
     *                  - PASS: every clause above is affirmatively satisfied.
     */
    AxiomResult conservativeResult{AxiomResult::UNKNOWN};

    /*!
     * @brief           True only when conservativeResult == AxiomResult::PASS;
     *                  never set from the legacy calculation.
     */
    bool isComplete{false};

    /*!
     * @brief           Every distinct reason contributing to
     *                  conservativeResult, sorted ascending by the underlying
     *                  ReasonCode value with no duplicates. Empty exactly when
     *                  conservativeResult == PASS (the sole contributing reason
     *                  is COMPLETENESS_ALL_CLEAR, reported via the AX-COMP-01
     *                  Finding this result feeds, not duplicated here).
     */
    std::vector<ReasonCode> reasons;

    /*!
     * @brief           Every entity key driving a non-PASS conservativeResult
     *                  (incomplete rooms, live prospectives, invalid passages,
     *                  or any entity named by another hard-violation Finding in
     *                  this map), sorted ascending with no duplicates. Empty
     *                  when conservativeResult == PASS.
     */
    std::vector<EntityKey> relevantEntityKeys;

    /*!
     * @brief           Count of live, RoomRecord::variant == ROOM rooms in this
     *                  map (each counted once, unlike the legacy multiplicity).
     */
    std::size_t confirmedRoomCount{0U};

    /*!
     * @brief           The confirmedRoomCount subset whose boundaryStatus ==
     *                  COMPLETE.
     */
    std::size_t completeRoomCount{0U};

    /*!
     * @brief           Count of live, RoomRecord::variant == UNDEFINED rooms in
     *                  this map (prospective handles, confirmed or not yet
     *                  promoted).
     */
    std::size_t prospectiveRoomCount{0U};

    /*!
     * @brief           Count of live passages in this map.
     */
    std::size_t livePassageCount{0U};

    /*!
     * @brief           The livePassageCount subset with two distinct, live,
     *                  confirmed, same-map, same-floor, reciprocal, COMPLETE
     *                  endpoints.
     */
    std::size_t fullyValidPassageCount{0U};

    /*!
     * @brief           Ids of the map's rooms whose boundary is not COMPLETE,
     *                  ascending (SemanticsManager::getIncompleteRooms(), via
     *                  MapSnapshot::incompleteRoomIds).
     */
    std::vector<int> incompleteRoomIds;

    /*!
     * @brief           Ids of the map's passages whose far-side room is
     *                  missing or not COMPLETE, ascending: where to explore
     *                  next (SemanticsManager::getFrontierPassages(), via
     *                  MapSnapshot::frontierPassageIds).
     */
    std::vector<int> frontierPassageIds;

    /*!
     * @brief           The exact legacy (SemanticsManager::Run()) calculation
     *                  for this same map, reproduced from this snapshot.
     */
    LegacyMapCompletenessResult legacy;

    /*!
     * @brief           True when legacy.isMapFullyModeled != isComplete: the
     *                  conservative calculation and the legacy calculation
     *                  disagree about this map.
     */
    bool doLegacyAndConservativeDiverge{false};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_MAPCOMPLETENESSRESULT_STRUCT_H \
        */
