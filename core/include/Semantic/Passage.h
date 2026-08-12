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

#ifndef PASSAGE_H
#define PASSAGE_H

#include <deque>

#include "Map.h"
#include "Thirdparty/g2o/g2o/types/plane3d.h"

namespace ORB_SLAM3
{
class Map;
class Plane;
class Marker;
class Room;

class Passage
{
  public:
    enum class TraversalDirection
    {
        UNKNOWN = 0,
        KNOWN_TO_FAR,
        FAR_TO_KNOWN
    };

    /** Atomic copy of the persistent camera/known-room side of a passage. */
    struct KnownSideProvenance
    {
        /** Non-owning room known to occupy the observing side, when available.
         */
        ORB_SLAM3::Room *pRoom{nullptr};

        /**
         * Unit world-frame direction from the passage toward the observing
         * side. It is independent of the arbitrary sign of the plane equation.
         */
        Eigen::Vector3d direction_World{Eigen::Vector3d::Zero()};

        bool hasDirection() const
        {
            return direction_World.allFinite() &&
                   direction_World.squaredNorm() > 0.99;
        }
    };

    enum passageVariant
    {
        UNDEFINED = -1,
        DOORWAY   = 0
    };

  private:
    int                             id;
    int                             opId;
    int                             opIdG;
    double                          width;
    double                          height;
    bool                            passable;
    Eigen::Vector3d                 centroid;
    passageVariant                  passageType;
    g2o::Plane3D                    globalEquation;
    ORB_SLAM3::Plane               *associateDoor;
    std::vector<ORB_SLAM3::Plane *> associateWalls;
    ORB_SLAM3::Room    *prospectiveRoom; // Stable far-side room handle
    KnownSideProvenance knownSideProvenance;

    /*!
     * @brief       Number of independent traversal observations.
     *
     *              A traversal observation is recorded whenever the UAV camera
     *              trajectory crosses the passage aperture. More than zero
     *              marks the passage as SETTLED / TRAVERSED.
     */
    std::size_t traversalKnownToFarCount;
    std::size_t traversalFarToKnownCount;
    std::size_t traversalUnknownCount;
    std::deque<std::pair<unsigned long, unsigned long>> traversalSegmentHistory;

  public:
    Passage();
    ~Passage();

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

    int  getId() const;
    void setId(int value);

    int  getOpId() const;
    void setOpId(int value);

    int  getOpIdG() const;
    void setOpIdG(int value);

    bool isPassable() const;
    void setPassable(bool value);

    /*!
     * @brief       Returns whether the UAV has crossed this passage aperture.
     *
     *              A traversed passage is considered settled: it must never be
     *              dropped by stale-cleanup logic and must always retain a
     *              routable far-side resolution.
     */
    bool getTraversalEvidence() const;

    /*!
     * @brief       Sets or clears the traversal evidence flag.
     *
     * @param[in]   value
     *              Settled (true) or not yet crossed (false).
     */
    void setTraversalEvidence(bool value);

    /*!
     * @brief       Returns the number of traversal observations accumulated.
     */
    std::size_t getTraversalObservationCount() const;

    /*!
     * @brief       Records one traversal observation (UAV crossing observed).
     */
    void addTraversalObservation();

    void addTraversalObservation(TraversalDirection direction_in);

    /** Records one trajectory segment once, including during history replay. */
    bool addTraversalObservation(TraversalDirection direction_in,
                                 unsigned long      frameId_in,
                                 unsigned long      keyFrameId_in);

    std::size_t getTraversalKnownToFarCount() const;
    std::size_t getTraversalFarToKnownCount() const;
    std::size_t getTraversalUnknownCount() const;
    bool        hasBidirectionalTraversalEvidence() const;

    /*!
     * @brief       Overrides the traversal observation counter.
     *
     * @param[in]   value
     *              New traversal observation count.
     */
    void setTraversalObservationCount(std::size_t value);

    double getWidth() const;
    void   setWidth(double value);

    double getHeight() const;
    void   setHeight(double value);

    passageVariant getPassageType();
    void           setPassageType(passageVariant newType);

    Eigen::Vector3d getCentroid() const;
    void            setCentroid(const Eigen::Vector3d &value);

    g2o::Plane3D getGlobalEquation() const;
    void         setGlobalEquation(const g2o::Plane3D &value);

    ORB_SLAM3::Plane *getAssociateDoor() const;
    void              setAssociateDoor(ORB_SLAM3::Plane *value);

    void addAssociateWall(ORB_SLAM3::Plane *p_wall_in);

    /*!
     * @brief       Returns the stable room handle on the far side of this
     *              passage. It starts as a prospective UNDEFINED room and is
     *              retained when that same object is promoted to ROOM.
     */
    ORB_SLAM3::Room *getProspectiveRoom() const;

    /*!
     * @brief       Sets the stable far-side room resolution for this passage.
     *              Caller retains ownership; Passage stores a non-owning
     * reference.
     */
    void setProspectiveRoom(ORB_SLAM3::Room *p_room_in);

    /*!
     * @brief       Checks whether this passage has a prospective room assigned.
     */
    bool hasProspectiveRoom() const;

    /*!
     * @brief       Replaces a retired prospective room with its retained
     * entity.
     *
     * @param[in]   p_retiredRoom_in
     *              Duplicate room removed from the active map.
     * @param[in]   p_retainedRoom_in
     *              Room which retains the combined associations.
     *
     * @return      True when this passage referenced the retired room.
     */
    bool replaceProspectiveRoom(ORB_SLAM3::Room *p_retiredRoom_in,
                                ORB_SLAM3::Room *p_retainedRoom_in);

    /** Returns the non-owning known-side room and sign-stable direction. */
    KnownSideProvenance getKnownSideProvenance() const;

    /** Stores a normalized, sign-stable observing-side direction. */
    bool setKnownSideDirection(const Eigen::Vector3d &direction_World_in);

    /** Links the persisted known side to a room without changing its direction.
     */
    void setKnownSideRoom(ORB_SLAM3::Room *p_room_in);

    /** Copies missing known-side fields from a duplicate passage. */
    void mergeKnownSideProvenance(const KnownSideProvenance &provenance_in);

    /*!
     * @brief       Replaces every reference to a retired plane hypothesis.
     *
     *              Both the supporting-wall collection and the optional door
     *              plane are updated. Duplicate retained-wall entries are
     *              removed atomically.
     *
     * @param[in]   p_retiredPlane_in
     *              Plane hypothesis which is being retired.
     *
     * @param[in]   p_retainedPlane_in
     *              Plane hypothesis which owns the fused geometry.
     *
     * @return      True when at least one association was replaced.
     */
    bool replacePlaneAssociation(ORB_SLAM3::Plane *p_retiredPlane_in,
                                 ORB_SLAM3::Plane *p_retainedPlane_in);

    std::vector<ORB_SLAM3::Plane *> getAssociateWalls() const;

    ORB_SLAM3::Map *getMap();
    void            setMap(ORB_SLAM3::Map *pMap);

  protected:
    ORB_SLAM3::Map    *mpMap;
    std::mutex         mMutexMap;
    mutable std::mutex mMutexType, mMutexGeometry;
};

} // namespace ORB_SLAM3

#endif
