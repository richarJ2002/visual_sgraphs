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
 * @file            LegacyMapCompletenessResult.h
 *
 * @brief           Declares an exact, snapshot-based reproduction of
 *                  SemanticsManager::Run()'s current ad-hoc, logging-only
 *                  per-map completeness calculation
 *                  (core/src/SemanticsManager.cc, the "MAP COMPLETENESS"
 *                  block around the "[SemMgrSummary] --- map completeness
 *                  ---" line).
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_LEGACY_MAP_COMPLETENESS_RESULT_H
#define SEMANTIC_AXIOM_EVALUATOR_LEGACY_MAP_COMPLETENESS_RESULT_H

#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Exact reproduction of the legacy per-map completeness
 *              projection, computed from a SemanticGraphSnapshot instead of
 *              live model pointers by computeLegacyMapCompleteness().
 *
 *              Deliberately preserves the legacy calculation's real
 *              multiplicity: it iterates the concatenation of
 *              Map::GetAllDetectedMapRooms() and
 *              Map::GetAllMarkerBasedMapRooms() (Map::GetAllRooms()'s own
 *              documented behaviour), so a Room pointer present in both
 *              collections is counted twice here as well, exactly as the
 *              live legacy code counts it twice -- see
 *              RoomRecord::isDetectedMember/isMarkerBasedMember, which the
 *              snapshot schema added specifically to make that
 *              reproduction possible without a live Atlas/Map pointer. This
 *              type is evidence for auditing/comparison only; it is not
 *              itself the legacy code path and does not replace it.
 */
struct LegacyMapCompletenessResult
{
  public:
    /*! @brief Legacy confirmedRoomCount: sum, over every GetAllRooms()
     *  entry (with the double-membership multiplicity above), of 1 for
     *  each entry whose room is live and RoomRecord::variant == ROOM. */
    std::size_t confirmedRoomCount{0U};

    /*! @brief Legacy completeRoomCount: the confirmedRoomCount subset whose
     *  boundaryStatus == COMPLETE. */
    std::size_t completeRoomCount{0U};

    /*! @brief Legacy incompleteRoomIds: local Room ids (Plane/Room-local,
     *  not EntityKey) of every confirmed-but-not-COMPLETE GetAllRooms()
     *  entry, in legacy iteration order including the same double-
     *  membership multiplicity (an id may repeat). */
    std::vector<int> incompleteRoomIds;

    /*! @brief Legacy passageCount: count of live passages in the map. */
    std::size_t passageCount{0U};

    /*! @brief Legacy fullyLinkedPassageCount: the passageCount subset whose
     *  known-side room and prospective room are both non-null, live, and
     *  RoomRecord::variant == ROOM. */
    std::size_t fullyLinkedPassageCount{0U};

    /*! @brief Legacy danglingPassageIds: local Passage ids of every live
     *  passage not counted in fullyLinkedPassageCount. */
    std::vector<int> danglingPassageIds;

    /*! @brief Legacy isMapFullyModeled: confirmedRoomCount > 0 &&
     *  completeRoomCount == confirmedRoomCount &&
     *  fullyLinkedPassageCount == passageCount. */
    bool isMapFullyModeled{false};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_LEGACY_MAP_COMPLETENESS_RESULT_H
