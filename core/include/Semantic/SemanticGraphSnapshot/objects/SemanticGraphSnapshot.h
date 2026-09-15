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
 * @file            SemanticGraphSnapshot.h
 *
 * @brief           Declares the immutable, value-only, atlas-wide semantic
 *                  graph snapshot type.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_OBJECT_H
#define SEMANTIC_GRAPH_SNAPSHOT_OBJECT_H

#include <optional>
#include <vector>

#include "AtlasCurrentMapStatus.h"

#include "Semantic/SemanticGraphSnapshot/objects/MapSnapshot.h"
#include "Semantic/SemanticGraphSnapshot/objects/OpenPassageHypothesisRecord.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnresolvedWallHypothesisRecord.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Immutable, value-only, atlas-wide semantic graph snapshot.
 *
 *              Every record and relationship collection reachable from this
 *              type is copied and sorted at capture time, so two captures of
 *              the same underlying state serialize byte-identically
 *              regardless of the model's live container iteration order (see
 *              P1.6 canonical serialization, a separate component that
 *              consumes this type).
 *
 *              Contains no raw/smart pointers to model objects, no PCL cloud
 *              data, no mutexes, no ROS types, no logger handles, no
 *              callbacks, and no wall-clock values.
 */
struct SemanticGraphSnapshot
{
  public:
    /*! @brief Absent only when currentMapStatus ==
     *  AtlasCurrentMapStatus::NO_CURRENT_MAP at capture time. Interpret
     *  together with currentMapStatus: a present value here is not itself
     *  proof that the named map appears in maps below -- see
     *  AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE. */
    std::optional<long unsigned int> currentMapId;

    /*! @brief Truthful status of currentMapId relative to maps below, read
     *  from Atlas::GetCoherentMapView() in the same critical section as
     *  currentMapId and maps. Never assume currentMapId names an entry in
     *  maps without checking this field first -- see
     *  AtlasCurrentMapStatus.h. */
    AtlasCurrentMapStatus currentMapStatus{
        AtlasCurrentMapStatus::NO_CURRENT_MAP};

    /*! @brief Sorted by mapId. Only Atlas::GetCoherentMapView()'s active
     *  map vector (live maps) is represented; see badRetiredMapVisibility. */
    std::vector<MapSnapshot> maps;

    /*! @brief Always NOT_EXPOSED_BY_CURRENT_API in this slice: Atlas's
     *  bad/retired map sets (mspBadMaps/mspRetiredMaps) are protected with
     *  no public enumeration API (confirmed by direct source read of
     *  Atlas.h/Atlas.cc). This snapshot never claims visibility into
     *  quarantined map state. */
    UnavailableReason badRetiredMapVisibility{
        UnavailableReason::NOT_EXPOSED_BY_CURRENT_API};

    /*! @brief Always NOT_CAPTURED_IN_FOUNDATION_SLICE in this slice:
     *  Atlas::copyRoomContextHistory()/copyRoomContextForMap() expose a
     *  per-map history of RoomContextSnapshot values captured before a map
     *  is abandoned (Atlas.h), but this foundation slice does not capture
     *  them into the graph snapshot. No current TODO in this plan names a
     *  phase that adds a value-only RoomContextSnapshot capture; resolving
     *  this to an actual value is an open scope decision, same as
     *  badRetiredMapVisibility above. */
    UnavailableReason roomContextHistoryReason{
        UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE};

    /*! @brief Value-only copy of SemanticsManager::openPassageEvidence_
     *  (SemanticsManager.h), converted and populated by SemanticsManager
     *  itself at the semantic transaction boundary (captureSemanticGraphSnapshot()
     *  cannot see this private member); empty and meaningless whenever
     *  managerPrivateOpenPassageHypothesesReason != NONE. */
    std::vector<OpenPassageHypothesisRecord> managerPrivateOpenPassageHypotheses;

    /*! @brief NOT_CAPTURED_IN_FOUNDATION_SLICE for a snapshot built only by
     *  captureSemanticGraphSnapshot() (e.g. a test fixture, or the legacy
     *  replay path); NONE once SemanticsManager::Run() has populated
     *  managerPrivateOpenPassageHypotheses above for this cycle. */
    UnavailableReason managerPrivateOpenPassageHypothesesReason{
        UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE};

    /*! @brief Value-only copy of SemanticsManager::undefendedWalls_
     *  (SemanticsManager.h), converted and populated by SemanticsManager
     *  itself at the semantic transaction boundary; empty and meaningless
     *  whenever managerPrivateUnresolvedWallHypothesesReason != NONE. */
    std::vector<UnresolvedWallHypothesisRecord>
        managerPrivateUnresolvedWallHypotheses;

    /*! @brief Same convention as managerPrivateOpenPassageHypothesesReason,
     *  for managerPrivateUnresolvedWallHypotheses. */
    UnavailableReason managerPrivateUnresolvedWallHypothesesReason{
        UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_OBJECT_H
