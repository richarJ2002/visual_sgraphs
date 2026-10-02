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
 * @file            WallRecord.h
 *
 * @brief           Declares a value-only copy of one wall face.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_WALL_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_WALL_RECORD_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "Geometric/Plane.h"

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
 * @brief       Value-only copy of one wall face: a Geometric/Plane.h object
 *              whose accepted Plane::PlaneVariant is WALL.
 */
struct WallRecord
{
  public:
    /*! @brief Map-qualified identity; mapId is the *containing* map used to
     *  enumerate this wall (Map::GetAllPlanes()), which may differ from
     *  declaredMapId below. */
    EntityKey key;

    /*! @brief Inverse of Plane::isBad(). */
    bool isLive{true};

    /*! @brief The plane's own Plane::GetMap() result at capture time, as a
     *  map id, or absent when that call returned nullptr. Captured
     *  independently of key.mapId (the containing map) so a stored-in-A/
     *  declares-B-or-null mismatch is directly diagnosable. */
    std::optional<long unsigned int> declaredMapId;

    /*! @brief Plane::getPlaneType() at capture time; always WALL for a
     *  record captured through this snapshot's wall-enumeration path,
     *  retained explicitly rather than assumed. */
    geometric::Plane::PlaneVariant planeType{
        geometric::Plane::PlaneVariant::WALL};

    /*! @brief PlaneGeometryMetadataSnapshot::planeEquation_world. */
    Eigen::Vector4d planeEquation_world{Eigen::Vector4d::Zero()};

    /*! @brief PlaneGeometryMetadataSnapshot::planeCentroid_world_m. */
    Eigen::Vector3d planeCentroid_world_m{Eigen::Vector3d::Zero()};

    /*! @brief PlaneGeometryMetadataSnapshot::minPlaneU_m. */
    double minPlaneU_m{0.0};

    /*! @brief PlaneGeometryMetadataSnapshot::maxPlaneU_m. */
    double maxPlaneU_m{0.0};

    /*! @brief PlaneGeometryMetadataSnapshot::minPlaneV_m. */
    double minPlaneV_m{0.0};

    /*! @brief PlaneGeometryMetadataSnapshot::maxPlaneV_m. */
    double maxPlaneV_m{0.0};

    /*! @brief PlaneGeometryMetadataSnapshot::finiteSupportCount. */
    std::size_t finiteSupportCount{0U};

    /*! @brief PlaneGeometryMetadataSnapshot::observationCount. */
    std::size_t observationCount{0U};

    /*! @brief PlaneGeometryMetadataSnapshot::cloudGeneration. */
    std::uint64_t cloudGeneration{0U};

    /*! @brief PlaneGeometryMetadataSnapshot::successfulRefitGeneration. */
    std::uint64_t successfulRefitGeneration{0U};

    /*! @brief World-frame camera position this face was first observed
     *  from, when Plane::setObservationOrigin_world() has been called
     *  (see Plane.h's documented rationale for why this, not a stored
     *  sign, distinguishes a physical wall's two faces). */
    std::optional<Eigen::Vector3d> observationOrigin_world_m;

    /*! @brief The derived, per-keyframe observation-side consensus
     *  (Plane::getObservationSideSnapshot()) is intentionally NOT captured
     *  in this foundation slice: it requires a caller-supplied normalized
     *  equation and iterates live KeyFrame pointers, which is a
     *  materially different, non-trivial capture step from the plain
     *  stored-field reads used everywhere else in this snapshot. A later
     *  phase captures it deliberately rather than this slice guessing
     *  which equation to pass it. */
    UnavailableReason observationSideConsensusReason{
        UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE};

    /*! @brief Non-owning link to the opposite-facing Plane hypothesis for
     *  the same physical wall. reason == NULL_REFERENCE is the ordinary
     *  "no twin identified yet" case. A RawPlaneRef, not an EntityKind::WALL
     *  EntityKey, because Plane::setTwinFace() has no type check: a wrong-
     *  type target (planeType != WALL) is retained truthfully here rather
     *  than fabricated into a nonexistent WallRecord identity. When
     *  wallKey has a value, the referenced wall is itself also a WallRecord
     *  in the same snapshot. */
    RawPlaneRef twinRef;

    /*! @brief One EntityRef per Room, in any live map, whose
     *  Room::getWalls() names this wall, computed at capture time by
     *  inverting the Room -> Wall direction (Plane stores no reverse
     *  pointer). Deliberately not restricted to this wall's own map, so a
     *  cross-map ownership error is visible rather than hidden.
     *  Cardinality other than exactly one is a question for the evaluator
     *  (AX-WALL-01), not this snapshot. Sorted; a wall registered as an
     *  owned wall of the same Room via two independent room-membership
     *  collections is not deduplicated away -- see the capture
     *  implementation's dedup-by-pointer inversion, which counts a room
     *  once per wall regardless of which collection(s) it was enumerated
     *  from, so this field reports distinct *rooms*, not distinct
     *  enumeration paths. Unlike the prior std::vector<EntityKey>
     *  representation, a bad owner retains its own liveness evidence here
     *  instead of losing it. */
    std::vector<EntityRef> ownerRoomRefs;

    /*!
     * @brief        Always NOT_TRACKED_BY_CURRENT_SCHEMA: no current
     *               Room/Wall/Passage field records semantic quarantine
     *               state (confirmed by direct source read --
     *               Plane.h/Room.h have no quarantine-flag member).
     *               SemanticsManager (holding
     *               Atlas::acquireSemanticUpdateLock()) would be the
     *               single writer of semantic quarantine state; a
     *               future extension that replaces
     *               suppressUndefendedWalls()'s semantic deletion with
     *               typed quarantine is the owner of resolving this to
     *               an actual value.
     */
    UnavailableReason quarantineReason{
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA};

    /*!
     * @brief        Always NOT_TRACKED_BY_CURRENT_SCHEMA: distinct from
     *               observationSideConsensusReason above (the derived
     *               per-keyframe side *consensus*), this covers the
     *               underlying individual observation-ray samples and
     *               their traversal order that would justify it.
     *               Plane::Observation (Plane.h) retains only an
     *               aggregated point-plane constraint matrix and
     *               per-generation counts, never an individual ray
     *               sample or its order (confirmed by direct source
     *               read). A future extension that adds bounded
     *               per-observation ray sampling and ray-parameter
     *               ordering owns resolving this to an actual value.
     */
    UnavailableReason observationRayEvidenceReason{
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_WALL_RECORD_H
