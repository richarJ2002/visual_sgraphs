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
 * @file            PassageRecord.h
 *
 * @brief           Declares a value-only copy of one passage.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_PASSAGE_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_PASSAGE_RECORD_H

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "Semantic/Passage.h"

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"
#include "Semantic/SemanticGraphSnapshot/objects/EntityRef.h"
#include "Semantic/SemanticGraphSnapshot/objects/RawPlaneRef.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Value-only copy of one passage (Passage::PassageVariant ==
 *              DOORWAY today; the enum leaves room for future variants).
 */
struct PassageRecord
{
  public:
    /*! @brief Map-qualified identity; mapId is the *containing* map used to
     *  enumerate this passage, which may differ from declaredMapId below. */
    EntityKey key;

    /*! @brief Inverse of Passage::isBad(). */
    bool isLive{true};

    /*! @brief The passage's own Passage::getMap() result at capture time,
     *  as a map id, or absent when that call returned nullptr. Captured
     *  independently of key.mapId (the containing map) so a stored-in-A/
     *  declares-B-or-null mismatch is directly diagnosable. */
    std::optional<long unsigned int> declaredMapId;

    /*! @brief Passage::getPassageType() at capture time. */
    Passage::PassageVariant passageType{Passage::PassageVariant::UNDEFINED};

    /*! @brief Passage::getGlobalEquation() at capture time. */
    Eigen::Vector4d planeEquation_world{Eigen::Vector4d::Zero()};

    /*! @brief Passage::getCentroid() at capture time. */
    Eigen::Vector3d passageCentroid_world_m{Eigen::Vector3d::Zero()};

    /*! @brief Passage::getWidth() at capture time. */
    double width_m{0.0};

    /*! @brief Passage::getHeight() at capture time. */
    double height_m{0.0};

    /*! @brief Passage::isPassable() at capture time. */
    bool isPassable{false};

    /*! @brief One RawPlaneRef per non-null Passage::getAssociateWalls()
     *  entry, sorted deterministically by (mapId, planeId, planeType). See
     *  RoomRecord::wallRefs for the identical retained-evidence rationale,
     *  including why a wrong-type target is never fabricated into a WALL
     *  identity. */
    std::vector<RawPlaneRef> associateWallRefs;

    /*! @brief Passage::getAssociateDoor(), a DOOR-typed Plane not captured
     *  as a WallRecord. reason == NULL_REFERENCE means no door plane is
     *  set, the ordinary case. */
    RawPlaneRef associateDoorRef;

    /*! @brief Passage::KnownSideProvenance::p_room. reason ==
     *  NULL_REFERENCE is the ordinary "no known-side room yet" case. */
    EntityRef knownSideRoomRef;

    /*! @brief Present only when Passage::KnownSideProvenance::
     *  hasDirection() was true at capture time (finite, near-unit norm);
     *  the struct's own check is trusted rather than re-derived here. */
    std::optional<Eigen::Vector3d> knownSideDirection_world;

    /*! @brief Passage::getProspectiveRoom(): the stable far-side room
     *  handle, present before AND after promotion to a confirmed Room
     *  (Passage.h documents it as the same object). Whether the
     *  referenced room is still UNDEFINED or has been promoted to ROOM is
     *  read from that room's own RoomRecord::variant in this same
     *  snapshot, not duplicated here. reason == NULL_REFERENCE is the
     *  ordinary "no far-side handle yet" case. */
    EntityRef prospectiveRoomRef;

    /*! @brief Passage::getTraversalKnownToFarCount() at capture time. */
    std::size_t traversalKnownToFarCount{0U};

    /*! @brief Passage::getTraversalFarToKnownCount() at capture time. */
    std::size_t traversalFarToKnownCount{0U};

    /*! @brief Passage::getTraversalUnknownCount() at capture time. */
    std::size_t traversalUnknownCount{0U};

    /*!
     * @brief        Always NOT_TRACKED_BY_CURRENT_SCHEMA: the current
     *               Passage model has no field distinguishing an
     *               authoritative DISCOVERY_SIDE/OPPOSITE_SIDE endpoint
     *               slot (confirmed by direct source read of
     *               Passage.h/Passage.cc -- KnownSideProvenance and
     *               getProspectiveRoom() record which room is
     *               known/prospective, not a named endpoint slot). An
     *               explicit schema field, not only prose in an
     *               evidence log, is required for "authoritative
     *               passage endpoints." A future extension that adds
     *               endpoint-slot tracking to Passage is the owner of
     *               resolving this to an actual value.
     */
    UnavailableReason endpointSlotReason{
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_PASSAGE_RECORD_H
