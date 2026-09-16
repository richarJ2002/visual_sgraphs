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
 * @file         Room.h
 *
 * @brief        Declares the semantic Room object.
 */

#ifndef ROOM_H
#define ROOM_H

#include "Geometric/Plane.h"
#include "Passage.h"
#include "Thirdparty/g2o/g2o/types/vertex_plane.h"

#include <cstdint>
#include <optional>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
struct RoomContextSnapshot;
class Floor;

class Room
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief        Enumerator which defines the types of rooms available as
     *               an object.
     */
    enum class RoomVariant : std::int8_t
    {
        /*!
         * @brief        Room is not defined, due to lack of semantic
         *               information.
         */
        UNDEFINED = -1,

        /*!
         * @brief        Room contains more than one wall. A corridor is not a
         *               distinct semantic type: it is simply an incomplete
         *               room whose boundary is still open.
         */
        ROOM = 0
    };

    /*!
     * @brief        Describes the geometric maturity of the observed
     *               room boundary.
     *
     *              Passage observations are deliberately independent
     *              of this state. An opening can be confirmed before
     *              enough finite wall extents have been observed to
     *              close the room boundary.
     */
    enum class BoundaryStatus
    {
        /*!
         * @brief        No wall evidence has been observed yet.
         */
        UNOBSERVED  = 0,
        /*!
         * @brief        Observed walls do not close the boundary
         *               yet.
         */
        INCOMPLETE  = 1,
        /*!
         * @brief        Observed walls close a valid boundary loop.
         */
        COMPLETE    = 2,
        /*!
         * @brief        Wall evidence contradicts a single closed
         *               boundary.
         */
        CONFLICTING = 3
    };

    /*!
     * @brief        One angular sector, measured counter-clockwise
     *               from the room's own centroid in the ground
     *               plane, with no observed wall inside it -- a
     *               candidate direction worth revisiting to
     *               complete this room's boundary.
     */
    struct ObservationGap
    {
        /*!
         * @brief        Sector start angle in the ground-plane (U,V)
         *               tangent frame, radians, atan2 convention.
         */
        double startAngle_rad{0.0};
        /*!
         * @brief        Sector span in radians; always positive. The
         *               sector runs [startAngle_rad,
         *               startAngle_rad + spanAngle_rad).
         */
        double spanAngle_rad{0.0};
    };

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    mutable std::mutex stateMutex;

    /*!
     * @brief        The room's identifier.
     */
    int id{-1};

    /*!
     * @brief        The room's identifier in the local optimizer.
     */
    int opId{-1};

    /*!
     * @brief        The room's identifier in the global optimizer.
     */
    int opIdG{-1};

    /*!
     * @brief        Marks the room as bad (if true, the room will not be used).
     */
    bool isBadFlag{false};

    /*!
     * @brief        The identifier of the room's meta-marker (containing
     *               information about the room).
     */
    int metaMarkerId{-1};

    /*!
     * @brief        The name devoted for each room (optional).
     */
    std::string name;

    /*!
     * @brief        Identity tag carried across map restarts
     *               (e.g. "room_5"). Empty when the room has not been
     *               matched to a prior-map context.
     */
    std::string roomTag;

    /*!
     * @brief        Non-owning pointer to the context snapshot from which
     *               the room identity was inherited.  nullptr when unset.
     */
    RoomContextSnapshot *p_matchedContext{nullptr};

    /*!
     * @brief        Checks if it is a candidate room (meta-marker detected) or
     *               not.
     */
    bool hasKnownLabel{false};

    /*!
     * @brief        True while identity and topology were restored
     *               without fresh map geometry.
     */
    bool recoveryProxy{false};

    /*!
     * @brief        True once the UAV has entered this room.
     *               Observed mission state only; never consulted by
     *               creation, promotion, retirement, or merge paths.
     */
    bool previouslyVisited{false};

    /*!
     * @brief        The meta-marker assigned for the room.
     */
    Marker *p_metaMarker{nullptr};

    /*!
     * @brief        The ground plane associated with the room.
     */
    geometric::Plane *p_groundPlane{nullptr};

    /*!
     * @brief        Non-owning floor node in the semantic hierarchy.
     */
    Floor *p_floor{nullptr};

    /*!
     * @brief        The room's semantic type (e.g., corridor, room, etc.).
     */
    RoomVariant variant{RoomVariant::UNDEFINED};

    /*!
     * @brief        The center of the room as a 3D vector in the global
     *               reference.
     */
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};

    /*!
     * @brief        The vector of detected walls of a room.
     */
    std::vector<geometric::Plane *> walls;

    /*!
     * @brief        The vector of detected doorways of a room.
     */
    std::vector<vs_graphs::core::semantic::Passage *> doorways;

    /*!
     * @brief        Current validation result for the finite
     *               horizontal wall loop.
     */
    BoundaryStatus boundaryStatus{BoundaryStatus::UNOBSERVED};

    /*!
     * @brief        Ordered, closed-loop corner points of the finite
     *               wall boundary, in world coordinates. Only
     *               meaningful while boundaryStatus == COMPLETE;
     *               empty otherwise.
     */
    std::vector<Eigen::Vector3d> boundaryCorners_World_m;

    /*!
     * @brief        Currently unobserved angular sectors around this
     *               room's own centroid. See getObservationGaps()
     *               for when this is populated.
     */
    std::vector<ObservationGap> observationGaps;

  public:
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief        Constructs an unassigned room with default semantic state.
     */
    Room();

    /*!
     * @brief        Destroys the room without deleting non-owning map elements.
     */
    ~Room();

    /*!
     * @brief        Apply a rigid/similarity transform to the plane
     *               geometry.
     *
     *              Updates the centroid, point cloud and plane
     *              equations so that the plane remains consistent
     *              with the merged map frame.
     *
     * @param[in]    transform_oldWorldToNewWorld_in
     *               Transform from the current plane frame to the
     *               new map frame.
     */
    void applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    /*!
     * @brief        Returns the room identifier assigned by the atlas.
     *
     * @return       Atlas room identifier.
     */
    int getId() const;

    /*!
     * @brief        Sets the room identifier assigned by the atlas.
     *
     * @param[in]    id_in
     *               New atlas room identifier.
     */
    void setId(int id_in);

    /*!
     * @brief        Returns the local optimizer vertex identifier.
     *
     * @return       Local optimizer identifier.
     */
    int getOpId() const;

    /*!
     * @brief        Sets the local optimizer vertex identifier.
     *
     * @param[in]    opId_in
     *               New local optimizer identifier.
     */
    void setOpId(int opId_in);

    /*!
     * @brief        Returns the global optimizer vertex identifier.
     *
     * @return       Global optimizer identifier.
     */
    int getOpIdG() const;

    /*!
     * @brief        Sets the global optimizer vertex identifier.
     *
     * @param[in]    opIdG_in
     *               New global optimizer identifier.
     */
    void setOpIdG(int opIdG_in);

    /*!
     * @brief        Reports whether the room has been invalidated.
     *
     * @return       True when the room is marked bad.
     */
    bool isBad();

    /*!
     * @brief        Marks the room as invalid for subsequent
     *               processing.
     */
    void setBad();

    /*!
     * @brief        Returns the room's current classification.
     *
     * @return       Active room variant.
     */
    RoomVariant getRoomVariant();

    /*!
     * @brief        Updates the room's classification.
     *
     * @param[in]    variant_in
     *               New room variant.
     */
    void setRoomVariant(RoomVariant variant_in);

    /*!
     * @brief        Returns the latest finite-wall boundary
     *               validation result.
     *
     * @return       Active boundary status.
     */
    BoundaryStatus getBoundaryStatus() const;

    /*!
     * @brief        Updates the finite-wall boundary validation
     *               result.
     *
     * @param[in]    boundaryStatus_in
     *               Result produced by the semantic boundary
     *               validator.
     */
    void setBoundaryStatus(BoundaryStatus boundaryStatus_in);

    /*!
     * @brief        Reports whether observed wall extents form a
     *               closed valid loop.
     *
     * @return       True while the boundary status is COMPLETE.
     */
    bool isBoundaryComplete() const;

    /*!
     * @brief        Returns the ordered, closed-loop corner points
     *               of the room's finite wall boundary, in world
     *               coordinates.
     *
     *              Populated by SemanticsManager::
     *              validateRoomBoundaries() only while
     *              getBoundaryStatus() == COMPLETE; empty otherwise.
     *              This is the same corner set the validator already
     *              computes to decide boundary completeness, exposed
     *              here for consumers (e.g. RViz visualization). A
     *              wall Plane's own finite bounds are never mutated
     *              to match these corners -- those bounds are owned
     *              by the measurement/refit pipeline.
     *
     * @return       Closed-loop corners in the world frame, or an
     *               empty vector when incomplete.
     */
    std::vector<Eigen::Vector3d> getBoundaryCorners_World_m() const;

    /*!
     * @brief        Sets the room's finite wall boundary corner
     *               points.
     *
     * @param[in]    corners_World_m_in
     *               Closed-loop corners in the world frame.
     */
    void setBoundaryCorners_World_m(
        std::vector<Eigen::Vector3d> corners_World_m_in);

    /*!
     * @brief        Returns the room's currently unobserved angular
     *               sectors -- directions around the room's own
     *               centroid with no wall evidence yet.
     *
     *              Populated by SemanticsManager::
     *              validateRoomBoundaries() every cycle this room
     *              has a finite centroid, regardless of
     *              BoundaryStatus (unlike
     *              getBoundaryCorners_World_m(), which is
     *              COMPLETE-only): this is precisely the "what's
     *              still missing" signal a COMPLETE room by
     *              definition no longer has. A room with zero walls
     *              reports one full-circle gap; a genuinely COMPLETE
     *              room reports none. Diagnostic and
     *              situational-awareness data only -- read by
     *              nothing else yet; a future observation planner is
     *              the intended consumer, not built here.
     *
     * @return       Unobserved sectors around the room centroid.
     */
    std::vector<ObservationGap> getObservationGaps() const;

    /*!
     * @brief        Sets the room's currently unobserved angular
     *               sectors.
     *
     * @param[in]    gaps_in
     *               New unobserved sectors.
     */
    void setObservationGaps(std::vector<ObservationGap> gaps_in);

    /*!
     * @brief        Reports whether the room has an externally known
     *               label.
     *
     * @return       True when a known label was assigned.
     */
    bool getHasKnownLabel() const;

    /*!
     * @brief        Sets whether the room has an externally known
     *               label.
     *
     * @param[in]    hasKnownLabel_in
     *               New known-label flag.
     */
    void setHasKnownLabel(bool hasKnownLabel_in);

    /*!
     * @brief        Returns the identifier of the room's metadata
     *               marker.
     *
     * @return       Metadata marker identifier.
     */
    int getMetaMarkerId() const;

    /*!
     * @brief        Sets the identifier of the room's metadata
     *               marker.
     *
     * @param[in]    metaMarkerId_in
     *               New metadata marker identifier.
     */
    void setMetaMarkerId(int metaMarkerId_in);

    /*!
     * @brief        Returns the non-owning metadata marker
     *               associated with the room.
     *
     * @return       Associated marker, or nullptr when unset.
     */
    Marker *getMetaMarker() const;

    /*!
     * @brief        Associates a non-owning metadata marker with the
     *               room.
     *
     * @param[in]    p_metaMarker_in
     *               Marker to associate; may be null.
     */
    void setMetaMarker(Marker *p_metaMarker_in);

    /*!
     * @brief        Returns the human-readable room label.
     *
     * @return       Room name, possibly empty.
     */
    std::string getName() const;

    /*!
     * @brief        Sets the human-readable room label.
     *
     * @param[in]    name_in
     *               New room name.
     */
    void setName(std::string name_in);

    /*!
     * @brief        Returns the persistent room identity tag assigned by
     *               Atlas room-context matching.
     *
     * @return       Tag string (e.g. "room_5") or an empty string when the
     *               room has not been matched to a prior-map context.
     */
    std::string getRoomTag() const;

    /*!
     * @brief        Assigns a persistent room identity tag.
     *
     * @param[in]    tag_in
     *               Tag string (e.g. "room_5") propagated from a prior map.
     */
    void setRoomTag(const std::string &tag_in);

    /*!
     * @brief        Reports whether the room carries a persistent
     *               identity tag.
     *
     * @return       True when a room tag was assigned.
     */
    bool hasRoomTag() const;

    /*!
     * @brief        Marks identity restored after a map reset but
     *               not yet re-observed.
     *
     * @param[in]    isRecoveryProxy_in
     *               New recovery-proxy flag.
     */
    void setRecoveryProxy(bool isRecoveryProxy_in);

    /*!
     * @brief        Returns whether this room still lacks fresh
     *               active-map observations.
     *
     * @return       True while the room is a recovery proxy.
     */
    bool isRecoveryProxy() const;

    /*!
     * @brief        Marks that the UAV has entered this room.
     *
     * @param[in]    visited_in
     *               New visited flag.
     */
    void setPreviouslyVisited(bool visited_in);

    /*!
     * @brief        Returns whether the UAV has entered this room.
     *
     * @return       True once the room was visited.
     */
    bool hasPreviouslyVisited() const;

    /*!
     * @brief        Stores a non-owning pointer to the context snapshot from
     *               which the room identity was inherited.
     *
     * @param[in]    p_matchedContext_in
     *               Pointer to a snapshot owned by the Atlas, or nullptr.
     */
    void setMatchedContext(RoomContextSnapshot *p_matchedContext_in);

    /*!
     * @brief        Returns the context snapshot associated with
     *               this room, or nullptr when no match exists.
     *
     * @return       Non-owning matched snapshot, or nullptr.
     */
    RoomContextSnapshot *getMatchedContext() const;

    /*!
     * @brief        Adds a non-owning passage association to the
     *               room.
     *
     * @param[in]    p_passage_in
     *               Passage to associate; null pointers are
     *               ignored.
     */
    void setDoorways(vs_graphs::core::semantic::Passage *p_passage_in);

    /*!
     * @brief        Returns the passages associated with the room.
     *
     * @return       Non-owning associated passages.
     */
    std::vector<vs_graphs::core::semantic::Passage *> getPassages() const;

    /*!
     * @brief        Replaces a retired passage with its retained fused entity.
     *
     * @param[in]    p_retiredPassage_in
     *               Duplicate passage removed from the active map.
     * @param[in]    p_retainedPassage_in
     *               Passage which retains the combined associations.
     *
     * @return       True when this room referenced the retired passage.
     */
    bool replacePassageAssociation(vs_graphs::core::semantic::Passage *p_retiredPassage_in,
                                   vs_graphs::core::semantic::Passage *p_retainedPassage_in);

    /*!
     * @brief        Removes every passage association without
     *               deleting passages.
     *
     *              The semantic manager uses this before rebuilding
     *              passage-room edges from the latest geometric
     *              evidence.
     */
    void clearPassages();

    /*!
     * @brief        Removes a single passage association from this room.
     *
     *               Used to enforce the invariant that a passage is associated
     *               with at most two rooms (one per side of its supporting
     * wall).
     *
     * @param[in]    p_removedPassage_in
     *               Passage whose association should be revoked.
     *
     * @return       True when the association was present and removed.
     */
    bool removePassageAssociation(vs_graphs::core::semantic::Passage *p_removedPassage_in);

    /*!
     * @brief        Adds a non-owning wall-plane association to this
     *               room.
     *
     *              Membership is local to the room and deduplicated
     *              by plane identity. The semantic manager enforces
     *              one owning room per mapped wall surface. Adjacent
     *              rooms therefore use distinct observations of the
     *              two physical wall faces.
     *
     * @param[in]    p_wall_in
     *               Wall plane to associate; null pointers are
     *               ignored.
     */
    void setWalls(vs_graphs::core::geometric::Plane *p_wall_in);

    /*!
     * @brief        Replaces a retired wall-plane association atomically.
     *
     *               This is used when two mapped plane hypotheses are fused.
     *               Existing membership of the retained plane is deduplicated.
     *
     * @param[in]    p_retiredWall_in
     *               Wall plane which is being retired.
     *
     * @param[in]    p_retainedWall_in
     *               Wall plane which owns the fused geometry.
     *
     * @return       True when this room referenced the retired wall.
     */
    bool replaceWall(vs_graphs::core::geometric::Plane *p_retiredWall_in,
                     vs_graphs::core::geometric::Plane *p_retainedWall_in);

    /*!
     * @brief        Removes one non-owning wall association.
     *
     * @param[in]    p_wall_in
     *               Wall whose ownership relationship is removed.
     *
     * @return       True when the wall was associated with this
     *               room.
     */
    bool removeWall(vs_graphs::core::geometric::Plane *p_wall_in);

    /*!
     * @brief        Returns the wall planes associated with the
     *               room.
     *
     * @return       Non-owning associated walls.
     */
    std::vector<vs_graphs::core::geometric::Plane *> getWalls() const;

    /*!
     * @brief        Returns a wall normal oriented toward this room's centroid.
     *
     *               A geometric plane has two equivalent coefficient
     *               representations, `(n, d)` and `(-n, -d)`. This method
     *               resolves that ambiguity for one room-wall relationship by
     *               evaluating the room centroid's signed distance to the wall.
     *               The stored plane equation is not modified.
     *
     * @param[in]    p_wall_in
     *               Non-owning pointer to the wall plane whose normal is
     *               required.
     *
     * @return       Unit wall normal expressed in the world frame and pointing
     *               toward the room centroid, or `std::nullopt` when the wall,
     *               plane equation, or room centroid is invalid. If the room
     *               centroid lies exactly on the wall, the normalized stored
     *               normal direction is returned.
     */
    std::optional<Eigen::Vector3d>
        getWallNormalTowardRoom_World(const geometric::Plane *p_wall_in) const;

    /*!
     * @brief        Removes all wall associations without deleting the planes.
     */
    void clearWalls();

    /*!
     * @brief        Removes null and invalid wall associations atomically.
     *
     * @return       Number of wall associations that remain valid.
     */
    std::size_t removeInvalidWalls();

    /*!
     * @brief        Returns the non-owning ground-plane association.
     *
     * @return       Associated ground plane, or nullptr when unset.
     */
    geometric::Plane *getGroundPlane() const;

    /*!
     * @brief        Associates a non-owning ground plane with the
     *               room.
     *
     * @param[in]    p_groundPlane_in
     *               Ground plane to associate; may be null.
     */
    void setGroundPlane(geometric::Plane *p_groundPlane_in);

    /*!
     * @brief        Replaces a retired ground-plane association.
     *
     * @param[in]    p_retiredGround_in
     *               Ground plane which is being retired.
     *
     * @param[in]    p_retainedGround_in
     *               Ground plane which owns the fused geometry.
     *
     * @return       True when this room referenced the retired ground plane.
     */
    bool replaceGroundPlane(geometric::Plane *p_retiredGround_in,
                            geometric::Plane *p_retainedGround_in);

    /*!
     * @brief        Returns the non-owning floor node associated with
     *               this room.
     *
     * @return       Associated floor, or nullptr when unset.
     */
    Floor *getFloor() const;

    /*!
     * @brief        Sets the non-owning floor node associated with
     *               this room.
     *
     * @param[in]    p_floor_in
     *               Floor to associate; may be null.
     */
    void setFloor(Floor *p_floor_in);

    /*!
     * @brief        Returns the room centroid in the active map
     *               frame.
     *
     * @return       Centroid in metres in the world frame.
     */
    Eigen::Vector3d getCentroid() const;

    /*!
     * @brief        Sets the room centroid in the active map frame.
     *
     * @param[in]    centroid_in
     *               New centroid in metres in the world frame.
     */
    void setCentroid(Eigen::Vector3d centroid_in);

    /*!
     * @brief        Returns the map that owns this room.
     *
     * @return       Non-owning owning map, or nullptr when unset.
     */
    Map *getMap();

    /*!
     * @brief        Assigns the room to a map.
     *
     * @param[in]    p_map_in
     *               Owning map; shall be non-null.
     */
    void setMap(Map *p_map_in);

  protected:
    /* ---------------------------------------------------------------------- *
     * PROTECTED MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief        Non-owning pointer to the map that owns this room.
     */
    Map *p_map{nullptr};

    /*!
     * @brief        Protects access to the owning-map association.
     */
    mutable std::mutex mapMutex;

    /*!
     * @brief        Protects the room's non-owning wall membership
     *               collection.
     */
    mutable std::mutex wallsMutex;

    /*!
     * @brief        Protects the non-owning room-to-floor hierarchy
     *               edge.
     */
    mutable std::mutex floorMutex;

    /*!
     * @brief        Protects the independently updated
     *               boundary-validation state.
     */
    mutable std::mutex boundaryStatusMutex;
};
} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif
