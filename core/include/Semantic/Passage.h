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
 * @file            Passage.h
 *
 * @brief           Declares Passage, a doorway or opening between rooms, with
 *                  the evidence of which rooms it joins.
 */

#ifndef PASSAGE_H
#define PASSAGE_H

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "Semantic/KnownSideProvenanceStatus.h"
#include "Semantic/PassageStatus.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace vs_graphs
{
namespace core
{
class Map;
namespace geometric
{
class Plane;
}
namespace semantic
{
class Marker;
class Room;

/*!
 * @brief           An opening between two spaces, such as a doorway, with its
 *                  size, the wall and door planes around it, the room beyond it
 *                  and the evidence of the UAV having crossed it.
 */
class Passage
{
  public:
    /*!
     * @brief           Which way a UAV crossing went, relative to the known
     *                  side (the side the passage was first observed from) and
     *                  the far side.
     */
    enum class TraversalDirection
    {
        UNKNOWN = 0,
        KNOWN_TO_FAR,
        FAR_TO_KNOWN
    };

    /*!
     * @brief           Atomic copy of the persistent camera/known-room side of
     *                  a passage.
     */
    struct KnownSideProvenance
    {
        /*!
         * @brief           Non-owning room known to occupy the observing side,
         *                  when available.
         */
        vs_graphs::core::semantic::Room *p_room{nullptr};

        /*!
         * Unit world-frame direction from the passage toward the observing
         * side. It is independent of the arbitrary sign of the plane equation.
         */
        Eigen::Vector3d knownSideDirection_world{Eigen::Vector3d::Zero()};

        /*!
         * @brief           Tells whether a usable known-side direction is
         *                  stored.
         *
         * @param[out]      hasDirection_out
         *                  True when the direction is finite and close to unit
         *                  length.
         *
         * @return          KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS always.
         */
        [[nodiscard]] KnownSideProvenanceStatus
            hasDirection(bool &hasDirection_out) const
        {
            hasDirection_out = knownSideDirection_world.allFinite() &&
                               knownSideDirection_world.squaredNorm() > 0.99;
            return KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS;
        }
    };

    /*!
     * @brief           Enumerator which defines the semantic variants a
     *                  passage can take.
     */
    enum class PassageVariant : std::int8_t
    {
        UNDEFINED = -1,
        DOORWAY   = 0
    };

  private:
    /*!
     * @brief           Stable identifier of this passage; two passages with the
     *                  same id are duplicates of one passage and get merged; -1
     *                  = not assigned yet.
     */
    int id;

    /*!
     * @brief           Identifier of this passage in the local optimizer;
     *                  -1 = not assigned yet.
     */
    int opId;

    /*!
     * @brief           Identifier of this passage in the global optimizer;
     *                  -1 = not assigned yet.
     */
    int opIdG;

    /*!
     * @brief           Width of the opening, in metres. Guarded by
     *                  geometryMutex.
     */
    double width;

    /*!
     * @brief           Height of the opening, in metres. Guarded by
     *                  geometryMutex.
     */
    double height;

    /*!
     * @brief           True when free space through the passage was observed,
     *                  so it can be walked through. Guarded by typeMutex.
     */
    bool isMarkedPassable;

    /*!
     * @brief           Centre of the opening in the world frame, metres.
     *                  Guarded by geometryMutex.
     */
    Eigen::Vector3d centroid;

    /*!
     * @brief           Kind of passage; UNDEFINED until set. Guarded by
     *                  typeMutex.
     */
    PassageVariant passageType;

    /*!
     * @brief           Plane of the opening in the world frame (unit normal and
     *                  offset). Guarded by geometryMutex.
     */
    g2o::Plane3D globalEquation;

    /*!
     * @brief           Door plane belonging to this passage; borrowed, null =
     *                  none. Guarded by geometryMutex.
     */
    vs_graphs::core::geometric::Plane *p_associatedDoor;

    /*!
     * @brief           Wall planes on either side of the opening; borrowed
     *                  pointers, no repeats. Guarded by geometryMutex.
     */
    std::vector<vs_graphs::core::geometric::Plane *> associateWalls;

    /*!
     * @brief           Stable handle of the room on the far side of the
     *                  passage; borrowed, null = none. Guarded by
     *                  geometryMutex.
     */
    vs_graphs::core::semantic::Room *p_prospectiveRoom;

    /*!
     * @brief           Room and direction of the side the passage was observed
     *                  from. Guarded by geometryMutex.
     */
    KnownSideProvenance knownSideProvenance;

    /*!
     * @brief           True once the passage was invalidated (see setBad).
     *                  Guarded by mapMutex.
     */
    bool isFlaggedBad{false};

    /*!
     * @brief           True when the passage only restores earlier topology and
     *                  has no supporting geometry in the active map. Guarded by
     *                  typeMutex.
     */
    bool isMarkedRecoveryProxy{false};

    /*!
     * @brief           Number of independent traversal observations.
     *
     *                  A traversal observation is recorded whenever the UAV
     *                  camera trajectory crosses the passage aperture. More
     *                  than zero marks the passage as SETTLED / TRAVERSED.
     */
    std::size_t traversalKnownToFarCount;

    /*!
     * @brief           Number of recorded crossings from the far side to the
     *                  known side. Guarded by typeMutex.
     */
    std::size_t traversalFarToKnownCount;

    /*!
     * @brief           Number of recorded crossings whose direction could not
     *                  be determined. Guarded by typeMutex.
     */
    std::size_t traversalUnknownCount;

    /*!
     * @brief           Trajectory segments already counted, as (frame id, key
     *                  frame id) pairs, so a replayed segment is not counted
     *                  twice. Holds the latest 128; guarded by typeMutex.
     */
    std::deque<std::pair<unsigned long, unsigned long>> traversalSegmentHistory;

  public:
    Passage() :
        id(-1),
        opId(-1),
        opIdG(-1),
        width(0.0),
        height(0.0),
        isMarkedPassable(false),
        centroid(Eigen::Vector3d::Zero()),
        passageType(Passage::PassageVariant::UNDEFINED),
        p_associatedDoor(nullptr),
        p_prospectiveRoom(nullptr),
        traversalKnownToFarCount(0U),
        traversalFarToKnownCount(0U),
        traversalUnknownCount(0U),
        p_map(nullptr)
    {}

    /*!
     * @brief           Apply a rigid/similarity transform to the plane
     *                  geometry.
     *
     *                  Updates the centroid, point cloud and plane equations so
     *                  that the plane remains consistent with the merged map
     *                  frame.
     *
     * @param[in]       alignmentTransform_oldWorldToNewWorld_in
     *                  Transform from the current plane frame to the new map
     *                  frame.
     */
    [[nodiscard]] PassageStatus applyTransform(
        const g2o::Sim3 &alignmentTransform_oldWorldToNewWorld_in);

    /*!
     * @brief           Returns the identifier of this passage.
     *
     * @param[out]      id_out
     *                  Passage identifier; -1 = not assigned yet.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getId(int &id_out) const;
    /*!
     * @brief           Sets the identifier of this passage.
     *
     * @param[in]       value_in
     *                  New passage identifier.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setId(int value_in);

    /*!
     * @brief           Returns the identifier of this passage in the local
     *                  optimizer.
     *
     * @param[out]      opId_out
     *                  Local optimizer identifier; -1 = not assigned yet.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getOpId(int &opId_out) const;
    /*!
     * @brief           Sets the identifier of this passage in the local
     *                  optimizer.
     *
     * @param[in]       value_in
     *                  New local optimizer identifier.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setOpId(int value_in);

    /*!
     * @brief           Returns the identifier of this passage in the global
     *                  optimizer.
     *
     * @param[out]      opIdG_out
     *                  Global optimizer identifier; -1 = not assigned yet.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getOpIdG(int &opIdG_out) const;
    /*!
     * @brief           Sets the identifier of this passage in the global
     *                  optimizer.
     *
     * @param[in]       value_in
     *                  New global optimizer identifier.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setOpIdG(int value_in);

    /*!
     * @brief           Tells whether free space through this passage was
     *                  observed.
     *
     * @param[out]      isPassable_out
     *                  True when the passage is marked passable.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus isPassable(bool &isPassable_out) const;
    /*!
     * @brief           Marks this passage as passable or not.
     *
     * @param[in]       value_in
     *                  True when the passage can be walked through.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setPassable(bool value_in);

    /*!
     * @brief           Marks this passage invalid (e.g. never resolved to any
     *                  associated room -- see associatePassagesToRooms()'s
     *                  0-room invalidation). Passage objects are never removed
     *                  from the Atlas; callers that iterate
     *                  Atlas::GetAllPassages() must skip bad ones themselves,
     *                  the same convention Plane/Room already use.
     */
    [[nodiscard]] PassageStatus isBad(bool &isBad_out);
    /*!
     * @brief           Marks this passage as invalid; it is never removed from
     *                  the atlas, so callers skip bad ones.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setBad();

    /*!
     * @brief           Marks topology restored without active-map supporting
     *                  geometry.
     */
    [[nodiscard]] PassageStatus setRecoveryProxy(bool isRecoveryProxy_in);

    /*!
     * @brief           Returns whether this passage is historical recovery
     *                  topology only.
     */
    [[nodiscard]] PassageStatus
        isRecoveryProxy(bool &isRecoveryProxy_out) const;

    /*!
     * @brief           Returns whether the UAV has crossed this passage
     *                  aperture.
     *
     *                  A traversed passage is considered settled: it must never
     *                  be dropped by stale-cleanup logic and must always retain
     *                  a routable far-side resolution.
     */
    [[nodiscard]] PassageStatus
        getTraversalEvidence(bool &traversalEvidence_out) const;

    /*!
     * @brief           Sets or clears the traversal evidence flag.
     *
     * @param[in]       value_in
     *                  Settled (true) or not yet crossed (false).
     */
    [[nodiscard]] PassageStatus setTraversalEvidence(bool value_in);

    /*!
     * @brief           Returns the number of traversal observations
     *                  accumulated.
     */
    [[nodiscard]] PassageStatus getTraversalObservationCount(
        std::size_t &traversalObservationCount_out) const;

    /*!
     * @brief           Records one traversal observation (UAV crossing
     *                  observed).
     */
    [[nodiscard]] PassageStatus addTraversalObservation();

    /*!
     * @brief           Records one crossing of the passage in the given
     *                  direction; the matching counter stops at its largest
     *                  value.
     *
     * @param[in]       direction_in
     *                  Crossing direction; UNKNOWN counts as an unknown
     *                  crossing.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        addTraversalObservation(TraversalDirection direction_in);

    /*!
     * @brief           Records one trajectory segment once, including during
     *                  history replay.
     */
    [[nodiscard]] PassageStatus
        addTraversalObservation(TraversalDirection direction_in,
                                unsigned long      frameId_in,
                                unsigned long      keyFrameId_in,
                                bool              &wasObservationAdded_out);

    /*!
     * @brief           Returns how many crossings went from the known side to
     *                  the far side.
     *
     * @param[out]      traversalKnownToFarCount_out
     *                  Number of known-to-far crossings.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getTraversalKnownToFarCount(
        std::size_t &traversalKnownToFarCount_out) const;
    /*!
     * @brief           Returns how many crossings went from the far side to the
     *                  known side.
     *
     * @param[out]      traversalFarToKnownCount_out
     *                  Number of far-to-known crossings.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getTraversalFarToKnownCount(
        std::size_t &traversalFarToKnownCount_out) const;
    /*!
     * @brief           Returns how many crossings had no determinable
     *                  direction.
     *
     * @param[out]      traversalUnknownCount_out
     *                  Number of crossings of unknown direction.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        getTraversalUnknownCount(std::size_t &traversalUnknownCount_out) const;
    /*!
     * @brief           Tells whether the passage was crossed both ways.
     *
     * @param[out]      hasBidirectionalTraversalEvidence_out
     *                  True when at least one known-to-far and one
     *                  far-to-known crossing were recorded.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus hasBidirectionalTraversalEvidence(
        bool &hasBidirectionalTraversalEvidence_out) const;

    /*!
     * @brief           Overrides the traversal observation counter.
     *
     * @param[in]       value_in
     *                  New traversal observation count.
     */
    [[nodiscard]] PassageStatus
        setTraversalObservationCount(std::size_t value_in);

    /*!
     * @brief           Returns the width of this passage.
     *
     * @param[out]      width_out
     *                  Opening width, metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getWidth(double &width_out) const;
    /*!
     * @brief           Sets the width of this passage.
     *
     * @param[in]       value_in
     *                  New opening width, metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setWidth(double value_in);

    /*!
     * @brief           Returns the height of this passage.
     *
     * @param[out]      height_out
     *                  Opening height, metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getHeight(double &height_out) const;
    /*!
     * @brief           Sets the height of this passage.
     *
     * @param[in]       value_in
     *                  New opening height, metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setHeight(double value_in);

    /*!
     * @brief           Returns the kind of this passage.
     *
     * @param[out]      passageType_out
     *                  Passage type; UNDEFINED until set.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        getPassageType(Passage::PassageVariant &passageType_out);
    /*!
     * @brief           Sets the kind of this passage.
     *
     * @param[in]       newType_in
     *                  New passage type.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setPassageType(PassageVariant newType_in);

    /*!
     * @brief           Returns the centre of this passage.
     *
     * @param[out]      centroid_out
     *                  Centre of the opening in the world frame, metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        getCentroid(Eigen::Vector3d &centroid_out) const;
    /*!
     * @brief           Sets the centre of this passage.
     *
     * @param[in]       value_in
     *                  New centre of the opening in the world frame,
     *                  metres.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setCentroid(const Eigen::Vector3d &value_in);

    /*!
     * @brief           Returns the plane of this passage.
     *
     * @param[out]      globalEquation_out
     *                  Plane of the opening in the world frame.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        getGlobalEquation(g2o::Plane3D &globalEquation_out) const;
    /*!
     * @brief           Sets the plane of this passage.
     *
     * @param[in]       value_in
     *                  New plane of the opening in the world frame.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setGlobalEquation(const g2o::Plane3D &value_in);

    /*!
     * @brief           Returns the door plane belonging to this passage.
     *
     * @param[out]      p_associateDoor_out
     *                  Borrowed door plane; null when none was set.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getAssociateDoor(
        vs_graphs::core::geometric::Plane *&p_associateDoor_out) const;
    /*!
     * @brief           Sets the door plane belonging to this passage.
     *
     * @param[in]       p_value_in
     *                  Borrowed door plane; the passage does not delete it;
     *                  null clears the link.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        setAssociateDoor(vs_graphs::core::geometric::Plane *p_value_in);

    /*!
     * @brief           Adds a wall plane next to this passage; a wall already
     *                  listed is not added twice.
     *
     * @param[in]       p_wall_in
     *                  Borrowed wall plane; a null pointer is ignored.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus
        addAssociateWall(vs_graphs::core::geometric::Plane *p_wall_in);

    /*!
     * @brief           Returns the stable room handle on the far side of this
     *                  passage. It starts as a prospective UNDEFINED room and
     *                  is retained when that same object is promoted to ROOM.
     */
    [[nodiscard]] PassageStatus getProspectiveRoom(
        vs_graphs::core::semantic::Room *&p_prospectiveRoom_out) const;

    /*!
     * @brief           Copies the far-side room ID while holding the passage
     *                  geometry lock.
     */
    [[nodiscard]] PassageStatus
        getProspectiveRoomId(std::optional<int> &prospectiveRoomId_out) const;

    /*!
     * @brief           Sets the stable far-side room resolution for this
     *                  passage.
     *                               Caller retains ownership; Passage stores a
     *                               non-owning
     *                  reference.
     */
    [[nodiscard]] PassageStatus
        setProspectiveRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*!
     * @brief           Checks whether this passage has a prospective room
     *                  assigned.
     */
    [[nodiscard]] PassageStatus
        hasProspectiveRoom(bool &hasProspectiveRoom_out) const;

    /*!
     * @brief           Replaces a retired prospective room with its retained
     *                  entity.
     *
     * @param[in]       p_retiredRoom_in
     *                  Duplicate room removed from the active map.
     *
     * @param[in]       p_retainedRoom_in
     *                  Room which retains the combined associations.
     *
     * @param[out]      wasRoomReplaced_out
     *                  True when this passage referenced the retired room.
     *
     * @return          PASSAGE_STATUS_SUCCESS, or
     *                  PASSAGE_STATUS_INVALID_ARGUMENT when an input is
     *                  rejected.
     */
    [[nodiscard]] PassageStatus replaceProspectiveRoom(
        vs_graphs::core::semantic::Room *p_retiredRoom_in,
        vs_graphs::core::semantic::Room *p_retainedRoom_in,
        bool                            &wasRoomReplaced_out);

    /*!
     * @brief           Returns the non-owning known-side room and sign-stable
     *                  direction.
     */
    [[nodiscard]] PassageStatus getKnownSideProvenance(
        Passage::KnownSideProvenance &knownSideProvenance_out) const;

    /*!
     * @brief           Stores a normalized, sign-stable observing-side
     *                  direction.
     */
    [[nodiscard]] PassageStatus setKnownSideDirection(
        const Eigen::Vector3d &knownSideDirection_world_in);

    /*!
     * @brief           Links the persisted known side to a room without
     *                  changing its direction.
     */
    [[nodiscard]] PassageStatus
        setKnownSideRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*!
     * @brief           Copies missing known-side fields from a duplicate
     *                  passage.
     */
    [[nodiscard]] PassageStatus
        mergeKnownSideProvenance(const KnownSideProvenance &provenance_in);

    /*!
     * @brief           Reconciles a same-identity duplicate into this canonical
     *                  passage.
     *
     *                  A real observed duplicate replaces recovery-proxy
     *                  geometry and current passability while preserving this
     *                  object's stable address and ID. Topology links and
     *                  traversal counters are merged without duplication. The
     *                  caller must serialize semantic graph mutation for both
     *                  passages.
     *
     * @param[in,out]   p_duplicate_inout
     *                  Duplicate passage with the same stable ID.
     *
     * @param[out]      wasGeometryReplaced_out
     *                  True when valid real geometry replaced recovery-proxy
     *                  geometry.
     *
     * @return          PASSAGE_STATUS_SUCCESS, or
     *                  PASSAGE_STATUS_INVALID_ARGUMENT when an input is
     *                  rejected.
     */
    [[nodiscard]] PassageStatus mergeFromDuplicate(
        vs_graphs::core::semantic::Passage *p_duplicate_inout,
        bool                               &wasGeometryReplaced_out);

    /*!
     * @brief           Replaces every reference to a retired plane hypothesis.
     *
     *                  Both the supporting-wall collection and the optional
     *                  door plane are updated. Duplicate retained-wall entries
     *                  are removed atomically.
     *
     * @param[in]       p_retiredPlane_in
     *                  Plane hypothesis which is being retired.
     *
     * @param[in]       p_retainedPlane_in
     *                  Plane hypothesis which owns the fused geometry.
     *
     * @param[out]      wasAssociationReplaced_out
     *                  True when at least one association was replaced.
     *
     * @return          PASSAGE_STATUS_SUCCESS, or
     *                  PASSAGE_STATUS_INVALID_ARGUMENT when an input is
     *                  rejected.
     */
    [[nodiscard]] PassageStatus replacePlaneAssociation(
        vs_graphs::core::geometric::Plane *p_retiredPlane_in,
        vs_graphs::core::geometric::Plane *p_retainedPlane_in,
        bool                              &wasAssociationReplaced_out);

    /*!
     * @brief           Returns a copy of the wall planes next to this passage.
     *
     * @param[out]      associateWalls_out
     *                  Borrowed wall planes.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getAssociateWalls(
        std::vector<vs_graphs::core::geometric::Plane *> &associateWalls_out)
        const;

    /*!
     * @brief           Returns the map this passage belongs to.
     *
     * @param[out]      p_map_out
     *                  Borrowed map pointer; null when no map was set.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus getMap(vs_graphs::core::Map *&p_map_out);
    /*!
     * @brief           Sets the map this passage belongs to.
     *
     * @param[in]       p_map_in
     *                  Borrowed map pointer; the passage does not delete it.
     *
     * @return          PASSAGE_STATUS_SUCCESS always.
     */
    [[nodiscard]] PassageStatus setMap(vs_graphs::core::Map *p_map_in);

  protected:
    /*!
     * @brief           Map this passage belongs to; borrowed, null = none.
     *                  Guarded by mapMutex.
     */
    vs_graphs::core::Map *p_map{nullptr};

    /*!
     * @brief           Guards p_map and the bad flag.
     */
    std::mutex mapMutex;

    /*!
     * @brief           Guards the passable flag, the passage type, the recovery
     *                  proxy flag and the traversal counters and history.
     */
    mutable std::mutex typeMutex;

    /*!
     * @brief           Guards the size, centroid, plane, the door and wall
     *                  planes, the far-side room and the known-side provenance.
     */
    mutable std::mutex geometryMutex;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif
