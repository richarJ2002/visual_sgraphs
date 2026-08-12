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

#ifndef FLOOR_H
#define FLOOR_H

#include "Map.h"
#include "Room.h"

#include <cstddef>
#include <optional>

namespace ORB_SLAM3
{
class Room;

class Floor
{
  public:
    /** Comparable, normalized ground-plane identity and its observation quality. */
    struct PlaneIdentity
    {
        Eigen::Vector4d equation_World{Eigen::Vector4d::Zero()};
        std::size_t     finiteSupportCount{0U};
        std::size_t     observationCount{0U};
    };

    static constexpr double kMergeMaxPlaneNormalAngle_deg = 10.0;
    static constexpr double kMergeMaxPlaneOffset_m        = 0.35;

  private:
    int             id;       // Floor's ID
    int             opId;     // Floor's ID in the local optimizer
    int             opIdG;    // Floor's ID in the global optimizer
    std::string     name;     // The name devoted for each room (optional)
    Eigen::Vector3d centroid; // Floor's centroid in the global reference
    std::vector<ORB_SLAM3::Room *> rooms; // Floor's rooms and corridors
    std::optional<PlaneIdentity> planeIdentity;

    void detachRoom(ORB_SLAM3::Room *p_room_in);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Floor();
    ~Floor();

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

    std::string getName() const;
    void        setName(std::string value);

    Eigen::Vector3d getCentroid() const;
    void            setCentroid(Eigen::Vector3d value);

    /** Returns true when a finite normalized ground-plane identity is stored. */
    bool hasPlaneIdentity() const;

    /** Returns one atomic snapshot of the equation and its evidence quality. */
    std::optional<PlaneIdentity> getPlaneIdentity() const;

    /**
     * Stores a normalized equation and evidence. Invalid equations are ignored
     * so a previously valid identity is never overwritten by missing data.
     */
    bool setPlaneIdentity(const Eigen::Vector4d &equation_World_in,
                          std::size_t finiteSupportCount_in,
                          std::size_t observationCount_in);

    /** Marks the current ground-plane identity unavailable. */
    void clearPlaneIdentity(void);

    /** Transforms an identity under the active old-world to new-world Sim3. */
    static std::optional<PlaneIdentity> transformPlaneIdentity(
        const PlaneIdentity &identity_OldWorld_in,
        const g2o::Sim3      &transform_oldWorldToNewWorld_in);

    /** Compares normalized planes with sign-invariant angle and offset tests. */
    static bool planeIdentitiesMatch(const PlaneIdentity &firstIdentity_in,
                                     const PlaneIdentity &secondIdentity_in,
                                     double maximumNormalAngle_deg_in,
                                     double maximumOffset_m_in,
                                     double &normalAngle_deg_out,
                                     double &offset_m_out);

    /** Selects the valid identity with most finite support, then observations. */
    static Floor *selectBestObservedFloor(const std::vector<Floor *> &floors_in);

    void                           addRoom(ORB_SLAM3::Room *value);
    std::vector<ORB_SLAM3::Room *> getRooms() const;
    void setRooms(const std::vector<ORB_SLAM3::Room *> &value);

    /*!
     * @brief       Replaces a retired room association after consolidation.
     *
     * @param[in]   p_retiredRoom_in
     *              Structural element which is being retired.
     *
     * @param[in]   p_retainedRoom_in
     *              Structural element which absorbs the relationship.
     *
     * @return      True when the retired room was present.
     */
    bool replaceRoom(ORB_SLAM3::Room *p_retiredRoom_in,
                     ORB_SLAM3::Room *p_retainedRoom_in);

    ORB_SLAM3::Map *getMap();
    void            setMap(ORB_SLAM3::Map *pMap);

  protected:
    ORB_SLAM3::Map    *mpMap;
    std::mutex         mMutexMap;
    mutable std::mutex mMutexRooms;
    mutable std::mutex mMutexGeometry;
};
} // namespace ORB_SLAM3

#endif
