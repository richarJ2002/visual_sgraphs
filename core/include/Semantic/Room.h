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

#ifndef ROOM_H
#define ROOM_H

#include "Geometric/Plane.h"
#include "Passage.h"
#include "Thirdparty/g2o/g2o/types/vertex_plane.h"

#include <optional>

namespace ORB_SLAM3
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
     * @brief       Enumerator which defines the types of rooms available as
     *              an object.
     */
    enum roomVariant
    {
        /*!
         * @brief       Room is not defined, due to lack of semantic
         *              information.
         */
        UNDEFINED = -1,

        /*!
         * @brief       Room contains more than one wall. A corridor is not a
         *              distinct semantic type: it is simply an incomplete
         *              room whose boundary is still open.
         */
        ROOM = 0
    };

    /*!
     * @brief Describes the geometric maturity of the observed room boundary.
     *
     * Passage observations are deliberately independent of this state. An
     * opening can be confirmed before enough finite wall extents have been
     * observed to close the room boundary.
     */
    enum class BoundaryStatus
    {
        UNOBSERVED  = 0,
        INCOMPLETE  = 1,
        COMPLETE    = 2,
        CONFLICTING = 3
    };

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The room's identifier.
     */
    int id{-1};

    /*!
     * @brief       The room's identifier in the local optimizer.
     */
    int opId{-1};

    /*!
     * @brief       The room's identifier in the global optimizer.
     */
    int opIdG{-1};

    /*!
     * @brief       Marks the room as bad (if true, the room will not be used).
     */
    bool mbBad{false};

    /*!
     * @brief       The identifier of the room's meta-marker (containing
     *              information about the room).
     */
    int metaMarkerId{-1};

    /*!
     * @brief       The name devoted for each room (optional).
     */
    std::string name;

    /*!
     * @brief       Identity tag carried across map restarts
     *              (e.g. "room_5"). Empty when the room has not been
     *              matched to a prior-map context.
     */
    std::string mRoomTag;

    /*!
     * @brief       Non-owning pointer to the WP1 context snapshot from which
     *              the room identity was inherited.  nullptr when unset.
     */
    RoomContextSnapshot *mpMatchedContext{nullptr};

    /*!
     * @brief       Checks if it is a candidate room (meta-marker detected) or
     *              not.
     */
    bool hasKnownLabel{false};

    /*!
     * @brief       The meta-marker assigned for the room.
     */
    Marker *metaMarker{nullptr};

    /*!
     * @brief       The ground plane associated with the room.
     */
    Plane *groundPlane{nullptr};

    /*! @brief Non-owning floor node in the semantic hierarchy. */
    Floor *floor{nullptr};

    /*!
     * @brief       The room's semantic type (e.g., corridor, room, etc.).
     */
    roomVariant variant{roomVariant::UNDEFINED};

    /*!
     * @brief       The center of the room as a 3D vector in the global
     *              reference.
     */
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};

    /*!
     * @brief       The vector of detected walls of a room.
     */
    std::vector<Plane *> walls;

    /*!
     * @brief       The vector of detected doorways of a room.
     */
    std::vector<ORB_SLAM3::Passage *> doorways;

    /*!
     * @brief Current validation result for the finite horizontal wall loop.
     */
    BoundaryStatus boundaryStatus{BoundaryStatus::UNOBSERVED};

  public:
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Constructs an unassigned room with default semantic state.
     */
    Room();

    /*!
     * @brief       Destroys the room without deleting non-owning map elements.
     */
    ~Room();

    /*!
     * @brief       Apply a rigid/similarity transform to the plane geometry.
     *
     *              Updates the centroid, point cloud and plane equations so
     *              that the plane remains consistent with the merged map frame.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the current plane frame to the new map frame.
     */
    void applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    /*!
     * @brief       Returns the room identifier assigned by the atlas.
     */
    int getId() const;

    /*!
     * @brief       Sets the room identifier assigned by the atlas.
     */
    void setId(int value);

    /*!
     * @brief       Returns the local optimizer vertex identifier.
     */
    int getOpId() const;

    /*!
     * @brief       Sets the local optimizer vertex identifier.
     */
    void setOpId(int value);

    /*!
     * @brief       Returns the global optimizer vertex identifier.
     */
    int getOpIdG() const;

    /*!
     * @brief       Sets the global optimizer vertex identifier.
     */
    void setOpIdG(int value);

    /*!
     * @brief       Reports whether the room has been invalidated.
     */
    bool isBad();

    /*!
     * @brief       Marks the room as invalid for subsequent processing.
     */
    void setBad();

    /*!
     * @brief       Returns the room's current classification.
     */
    roomVariant getRoomVariant();

    /*!
     * @brief       Updates the room's classification.
     */
    void setRoomVariant(roomVariant value);

    /*!
     * @brief Returns the latest finite-wall boundary validation result.
     */
    BoundaryStatus getBoundaryStatus() const;

    /*!
     * @brief Updates the finite-wall boundary validation result.
     *
     * @param[in] boundaryStatus_in
     *            Result produced by the semantic boundary validator.
     */
    void setBoundaryStatus(BoundaryStatus boundaryStatus_in);

    /*!
     * @brief Reports whether observed wall extents form a closed valid loop.
     */
    bool isBoundaryComplete() const;

    /*!
     * @brief       Reports whether the room has an externally known label.
     */
    bool getHasKnownLabel() const;

    /*!
     * @brief       Sets whether the room has an externally known label.
     */
    void setHasKnownLabel(bool value);

    /*!
     * @brief       Returns the identifier of the room's metadata marker.
     */
    int getMetaMarkerId() const;

    /*!
     * @brief       Sets the identifier of the room's metadata marker.
     */
    void setMetaMarkerId(int value);

    /*!
     * @brief       Returns the non-owning metadata marker associated with the
     *              room.
     */
    Marker *getMetaMarker() const;

    /*!
     * @brief       Associates a non-owning metadata marker with the room.
     */
    void setMetaMarker(Marker *value);

    /*!
     * @brief       Returns the human-readable room label.
     */
    std::string getName() const;

    /*!
     * @brief       Sets the human-readable room label.
     */
    void setName(std::string value);

    /*!
     * @brief       Returns the persistent room identity tag assigned by
     *              Atlas room-context matching.
     *
     * @return      Tag string (e.g. "room_5") or an empty string when the
     *              room has not been matched to a prior-map context.
     */
    std::string getRoomTag() const;

    /*!
     * @brief       Assigns a persistent room identity tag.
     *
     * @param[in]   tag
     *              Tag string (e.g. "room_5") propagated from a prior map.
     */
    void setRoomTag(const std::string &tag);

    /*!
     * @brief       Reports whether the room carries a persistent identity tag.
     */
    bool hasRoomTag() const;

    /*!
     * @brief       Stores a non-owning pointer to the context snapshot from
     *              which the room identity was inherited.
     *
     * @param[in]   ctx
     *              Pointer to a snapshot owned by the Atlas, or nullptr.
     */
    void setMatchedContext(RoomContextSnapshot *ctx);

    /*!
     * @brief       Returns the context snapshot associated with this room,
     *              or nullptr when no match has been made.
     */
    RoomContextSnapshot *getMatchedContext() const;

    /*!
     * @brief       Adds a non-owning passage association to the room.
     */
    void setDoorways(ORB_SLAM3::Passage *value);

    /*!
     * @brief       Returns the passages associated with the room.
     */
    std::vector<ORB_SLAM3::Passage *> getPassages() const;

    /*!
     * @brief       Replaces a retired passage with its retained fused entity.
     *
     * @param[in]   p_retiredPassage_in
     *              Duplicate passage removed from the active map.
     * @param[in]   p_retainedPassage_in
     *              Passage which retains the combined associations.
     *
     * @return      True when this room referenced the retired passage.
     */
    bool replacePassageAssociation(ORB_SLAM3::Passage *p_retiredPassage_in,
                                   ORB_SLAM3::Passage *p_retainedPassage_in);

    /*!
     * @brief Removes every passage association without deleting passages.
     *
     *        The semantic manager uses this before rebuilding passage-room
     *        edges from the latest geometric evidence.
     */
    void clearPassages();

    /*!
     * @brief       Removes a single passage association from this room.
     *
     *              Used to enforce the invariant that a passage is associated
     *              with at most two rooms (one per side of its supporting wall).
     *
     * @param[in]   p_removedPassage_in
     *              Passage whose association should be revoked.
     *
     * @return      True when the association was present and removed.
     */
    bool removePassageAssociation(ORB_SLAM3::Passage *p_removedPassage_in);

    /*!
     * @brief       Adds a non-owning wall-plane association to this room.
     *
     *              Membership is local to the room and deduplicated by plane
     *              identity. The semantic manager enforces one owning room
     *              per mapped wall surface. Adjacent rooms therefore use
     *              distinct observations of the two physical wall faces.
     *
     * @param[in]   p_wall_in
     *              Wall plane to associate with this room.
     */
    void setWalls(ORB_SLAM3::Plane *p_wall_in);

    /*!
     * @brief       Replaces a retired wall-plane association atomically.
     *
     *              This is used when two mapped plane hypotheses are fused.
     *              Existing membership of the retained plane is deduplicated.
     *
     * @param[in]   p_retiredWall_in
     *              Wall plane which is being retired.
     *
     * @param[in]   p_retainedWall_in
     *              Wall plane which owns the fused geometry.
     *
     * @return      True when this room referenced the retired wall.
     */
    bool replaceWall(ORB_SLAM3::Plane *p_retiredWall_in,
                     ORB_SLAM3::Plane *p_retainedWall_in);

    /*!
     * @brief Removes one non-owning wall association.
     *
     * @param[in] p_wall_in
     *            Wall whose ownership relationship is removed.
     *
     * @return True when the wall was associated with this room.
     */
    bool removeWall(ORB_SLAM3::Plane *p_wall_in);

    /*!
     * @brief       Returns the wall planes associated with the room.
     */
    std::vector<ORB_SLAM3::Plane *> getWalls() const;

    /*!
     * @brief       Returns a wall normal oriented toward this room's centroid.
     *
     *              A geometric plane has two equivalent coefficient
     *              representations, `(n, d)` and `(-n, -d)`. This method
     *              resolves that ambiguity for one room-wall relationship by
     *              evaluating the room centroid's signed distance to the wall.
     *              The stored plane equation is not modified.
     *
     * @param[in]   p_wall_in
     *              Non-owning pointer to the wall plane whose normal is
     *              required.
     *
     * @return      Unit wall normal expressed in the world frame and pointing
     *              toward the room centroid, or `std::nullopt` when the wall,
     *              plane equation, or room centroid is invalid. If the room
     *              centroid lies exactly on the wall, the normalized stored
     *              normal direction is returned.
     */
    std::optional<Eigen::Vector3d>
        getWallNormalTowardRoom_World(const Plane *p_wall_in) const;

    /*!
     * @brief       Removes all wall associations without deleting the planes.
     */
    void clearWalls();

    /*!
     * @brief       Removes null and invalid wall associations atomically.
     *
     * @return      Number of wall associations that remain valid.
     */
    std::size_t removeInvalidWalls();

    /*!
     * @brief       Returns the non-owning ground-plane association.
     */
    Plane *getGroundPlane() const;

    /*!
     * @brief       Associates a non-owning ground plane with the room.
     */
    void setGroundPlane(Plane *p_groundPlane_in);

    /*!
     * @brief       Replaces a retired ground-plane association.
     *
     * @param[in]   p_retiredGround_in
     *              Ground plane which is being retired.
     *
     * @param[in]   p_retainedGround_in
     *              Ground plane which owns the fused geometry.
     *
     * @return      True when this room referenced the retired ground plane.
     */
    bool replaceGroundPlane(Plane *p_retiredGround_in,
                            Plane *p_retainedGround_in);

    /** Returns the non-owning floor node associated with this room. */
    Floor *getFloor() const;

    /** Sets the non-owning floor node associated with this room. */
    void setFloor(Floor *p_floor_in);

    /*!
     * @brief       Returns the room centroid in the active map frame.
     */
    Eigen::Vector3d getCentroid() const;

    /*!
     * @brief       Sets the room centroid in the active map frame.
     */
    void setCentroid(Eigen::Vector3d value);

    /*!
     * @brief       Returns the map that owns this room.
     */
    Map *getMap();

    /*!
     * @brief       Assigns the room to a map.
     */
    void setMap(Map *pMap);

  protected:
    /* ---------------------------------------------------------------------- *
     * PROTECTED MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Non-owning pointer to the map that owns this room.
     */
    Map *mpMap{nullptr};

    /*!
     * @brief       Protects access to the owning-map association.
     */
    mutable std::mutex mMutexMap;

    /*!
     * @brief Protects the room's non-owning wall membership collection.
     */
    mutable std::mutex mMutexWalls;

    /*! @brief Protects the non-owning room-to-floor hierarchy edge. */
    mutable std::mutex mMutexFloor;

    /*!
     * @brief Protects the independently updated boundary-validation state.
     */
    mutable std::mutex mMutexBoundaryStatus;
};
} // namespace ORB_SLAM3

#endif
