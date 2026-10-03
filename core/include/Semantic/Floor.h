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
 * @file            Floor.h
 *
 * @brief           Declares Floor, one storey of the building and the rooms on
 *                  it.
 */

#ifndef FLOOR_H
#define FLOOR_H

#include "Semantic/FloorStatus.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

#include <Eigen/Core>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
class Map;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Room;

/*!
 * @brief           One storey of the building: the rooms and corridors found on
 *                  it, plus the ground plane used to tell floors apart and to
 *                  merge the same floor seen in different maps.
 */
class Floor
{
  public:
    /*!
     * @brief           Comparable, normalized ground-plane identity and its
     *                  observation quality.
     */
    struct PlaneIdentity
    {
        /*!
         * @brief           Ground-plane equation (normal x, y, z and offset)
         *                  in the world frame, scaled so the normal has unit
         *                  length.
         */
        Eigen::Vector4d planeEquation_world{Eigen::Vector4d::Zero()};

        /*!
         * @brief           How many observations of the plane had finite
         *                  support; the first criterion for picking the best
         *                  observed floor.
         */
        std::size_t finiteSupportCount{0U};

        /*!
         * @brief           Total number of observations of the plane; breaks
         *                  ties between equal finite support counts.
         */
        std::size_t observationCount{0U};
    };

    /*!
     * @brief           Largest angle, in degrees, between two ground-plane
     *                  normals for the planes to count as the same floor.
     */
    static constexpr double kMergeMaxPlaneNormalAngle_deg = 10.0;

    /*!
     * @brief           Largest difference, in metres, between two ground-plane
     *                  offsets for the planes to count as the same floor.
     */
    static constexpr double kMergeMaxPlaneOffset_m = 0.35;

  private:
    /*!
     * @brief           Identifier of this floor, reserved by the atlas;
     *                  -1 = not assigned yet.
     */
    int id;

    /*!
     * @brief           Identifier of this floor in the local optimizer;
     *                  -1 = not assigned yet.
     */
    int opId;

    /*!
     * @brief           Identifier of this floor in the global optimizer;
     *                  -1 = not assigned yet.
     */
    int opIdG;

    /*!
     * @brief           Optional display name of this floor (empty = none).
     */
    std::string name;

    /*!
     * @brief           Centroid of this floor in the world frame, metres.
     *                  Guarded by geometryMutex.
     */
    Eigen::Vector3d centroid;

    /*!
     * @brief           Rooms and corridors on this floor; borrowed pointers,
     *                  the floor does not delete them. Guarded by roomsMutex.
     */
    std::vector<vs_graphs::core::semantic::Room *> rooms;

    /*!
     * @brief           Ground-plane identity of this floor; empty until one is
     *                  stored with setPlaneIdentity. Guarded by geometryMutex.
     */
    std::optional<PlaneIdentity> planeIdentity;

    /*!
     * @brief           Removes a room from this floor and clears the room's
     *                  floor link when it still points here.
     *
     * @param[in,out]   p_room_inout
     *                  Room to detach; a null pointer is ignored.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus
        detachRoom(vs_graphs::core::semantic::Room *p_room_inout);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Floor() :
        id(-1),
        opId(-1),
        opIdG(-1),
        name(""),
        centroid(Eigen::Vector3d::Zero()),
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
    [[nodiscard]] FloorStatus applyTransform(
        const g2o::Sim3 &alignmentTransform_oldWorldToNewWorld_in);

    /*!
     * @brief           Returns the identifier of this floor.
     *
     * @param[out]      id_out
     *                  Floor identifier; -1 = not assigned yet.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getId(int &id_out) const;

    /*!
     * @brief           Sets the identifier of this floor.
     *
     * @param[in]       value_in
     *                  New floor identifier.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setId(int value_in);

    /*!
     * @brief           Returns the identifier of this floor in the local
     *                  optimizer.
     *
     * @param[out]      opId_out
     *                  Local optimizer identifier; -1 = not assigned yet.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getOpId(int &opId_out) const;

    /*!
     * @brief           Sets the identifier of this floor in the local
     *                  optimizer.
     *
     * @param[in]       value_in
     *                  New local optimizer identifier.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setOpId(int value_in);

    /*!
     * @brief           Returns the identifier of this floor in the global
     *                  optimizer.
     *
     * @param[out]      opIdG_out
     *                  Global optimizer identifier; -1 = not assigned yet.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getOpIdG(int &opIdG_out) const;

    /*!
     * @brief           Sets the identifier of this floor in the global
     *                  optimizer.
     *
     * @param[in]       value_in
     *                  New global optimizer identifier.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setOpIdG(int value_in);

    /*!
     * @brief           Returns the display name of this floor.
     *
     * @param[out]      name_out
     *                  Floor name; empty when none was set.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getName(std::string &name_out) const;

    /*!
     * @brief           Sets the display name of this floor.
     *
     * @param[in]       value_in
     *                  New floor name.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setName(std::string value_in);

    /*!
     * @brief           Returns the centroid of this floor.
     *
     * @param[out]      centroid_out
     *                  Floor centroid in the world frame, metres.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getCentroid(Eigen::Vector3d &centroid_out) const;

    /*!
     * @brief           Sets the centroid of this floor.
     *
     * @param[in]       value_in
     *                  New floor centroid in the world frame, metres.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setCentroid(Eigen::Vector3d value_in);

    /*!
     * @brief           Returns true when a finite normalized ground-plane
     *                  identity is stored.
     */
    [[nodiscard]] FloorStatus
        hasPlaneIdentity(bool &hasPlaneIdentity_out) const;

    /*!
     * @brief           Returns one atomic snapshot of the equation and its
     *                  evidence quality.
     */
    [[nodiscard]] FloorStatus getPlaneIdentity(
        std::optional<Floor::PlaneIdentity> &planeIdentity_out) const;

    /*!
     * Stores a normalized equation and evidence. Invalid equations are ignored
     * so a previously valid identity is never overwritten by missing data.
     */
    [[nodiscard]] FloorStatus
        setPlaneIdentity(const Eigen::Vector4d &planeEquation_world_in,
                         std::size_t            finiteSupportCount_in,
                         std::size_t            observationCount_in);

    /*!
     * @brief           Marks the current ground-plane identity unavailable.
     */
    [[nodiscard]] FloorStatus clearPlaneIdentity(void);

    /*!
     * @brief           Transforms an identity under the active old-world to
     *                  new-world Sim3.
     */
    [[nodiscard]] static FloorStatus transformPlaneIdentity(
        const PlaneIdentity &planeIdentity_oldWorld_in,
        const g2o::Sim3     &alignmentTransform_oldWorldToNewWorld_in,
        std::optional<Floor::PlaneIdentity> &transformedIdentity_out);

    /*!
     * @brief           Compares normalized planes with sign-invariant angle and
     *                  offset tests.
     */
    [[nodiscard]] static FloorStatus
        planeIdentitiesMatch(const PlaneIdentity &firstIdentity_in,
                             const PlaneIdentity &secondIdentity_in,
                             double               maximumNormalAngle_deg_in,
                             double               maximumOffset_m_in,
                             double              &normalAngle_deg_out,
                             double              &offset_m_out,
                             bool                &isMatch_out);

    /*!
     * @brief           Selects the valid identity with most finite support,
     *                  then observations.
     */
    [[nodiscard]] static FloorStatus
        selectBestObservedFloor(const std::vector<Floor *> &floors_in,
                                Floor                     *&p_bestFloor_out);

    /*!
     * @brief           Adds a room to this floor, taking it away from any other
     *                  floor it was on, and points the room's floor link here.
     *
     * @param[in,out]   p_value_inout
     *                  Room to add; a null pointer is ignored. A room already
     *                  on this floor is not added twice.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus
        addRoom(vs_graphs::core::semantic::Room *p_value_inout);

    /*!
     * @brief           Returns a copy of the list of rooms on this floor.
     *
     * @param[out]      rooms_out
     *                  Borrowed room pointers (a copy of the list).
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getRooms(
        std::vector<vs_graphs::core::semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Replaces the rooms on this floor with the given list.
     *                  Null and repeated entries are skipped, rooms taken from
     *                  other floors are detached from them, and rooms dropped
     *                  from this floor lose their floor link.
     *
     * @param[in]       value_in
     *                  New list of rooms (borrowed pointers).
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setRooms(
        const std::vector<vs_graphs::core::semantic::Room *> &value_in);

    /*!
     * @brief           Replaces a retired room association after consolidation.
     *
     * @param[in,out]   p_retiredRoom_inout
     *                  Structural element which is being retired.
     *
     * @param[in,out]   p_retainedRoom_inout
     *                  Structural element which absorbs the relationship.
     *
     * @param[out]      wasRoomReplaced_out
     *                  True when the retired room was present.
     *
     * @return          FLOOR_STATUS_SUCCESS, or FLOOR_STATUS_INVALID_ARGUMENT
     *                  when an input is rejected.
     */
    [[nodiscard]] FloorStatus
        replaceRoom(vs_graphs::core::semantic::Room *p_retiredRoom_inout,
                    vs_graphs::core::semantic::Room *p_retainedRoom_inout,
                    bool                            &wasRoomReplaced_out);

    /*!
     * @brief           Returns the map this floor belongs to.
     *
     * @param[out]      p_map_out
     *                  Borrowed map pointer; null when no map was set.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus getMap(vs_graphs::core::Map *&p_map_out);

    /*!
     * @brief           Sets the map this floor belongs to.
     *
     * @param[in]       p_map_in
     *                  Borrowed map pointer; the floor does not delete it.
     *
     * @return          FLOOR_STATUS_SUCCESS always.
     */
    [[nodiscard]] FloorStatus setMap(vs_graphs::core::Map *p_map_in);

  protected:
    /*!
     * @brief           Map this floor belongs to; borrowed, null = none.
     *                  Guarded by mapMutex.
     */
    vs_graphs::core::Map *p_map{nullptr};

    /*!
     * @brief           Guards p_map.
     */
    std::mutex mapMutex;

    /*!
     * @brief           Guards the list of rooms.
     */
    mutable std::mutex roomsMutex;

    /*!
     * @brief           Guards the centroid and the ground-plane identity.
     */
    mutable std::mutex geometryMutex;
};
} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif
