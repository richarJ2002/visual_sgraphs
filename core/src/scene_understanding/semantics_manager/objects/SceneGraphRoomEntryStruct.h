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
 * @file            SceneGraphRoomEntryStruct.h
 *
 * @brief           Declares SceneGraphRoomEntry, the plain-data copy of a room
 *                  inside a SceneGraphSnapshot.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_ROOM_ENTRY_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_ROOM_ENTRY_STRUCT_H

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "scene_understanding/scene_graph/room/objects/RoomClass.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the publisher, the archive and the health report need
 *                  to know about one room, copied out under the graph lock.
 *                  Holds no pointer into the graph: links to other nodes and to
 *                  planes are ids, resolved through the snapshot and the
 *                  map that owns the planes.
 */
struct SceneGraphRoomEntry
{
  public:
    /*!
     * @brief           Room id, unique within its map.
     */
    int id{-1};

    /*!
     * @brief           Display name of the room.
     */
    std::string name;

    /*!
     * @brief           True when the room is flagged bad.
     */
    bool isBad{false};

    /*!
     * @brief           Confirmed room, or undefined (a candidate or a passage
     *                  hypothesis).
     */
    semantic::Room::RoomVariant variant{semantic::Room::RoomVariant::UNDEFINED};

    /*!
     * @brief           Maturity of the room's wall boundary.
     */
    semantic::Room::BoundaryStatus boundaryStatus{
        semantic::Room::BoundaryStatus::UNOBSERVED};

    /*!
     * @brief           True when the wall boundary is closed.
     */
    bool isBoundaryComplete{false};

    /*!
     * @brief           True when the camera has been inside the room before.
     */
    bool hasBeenVisited{false};

    /*!
     * @brief           Room centroid in the map's world frame, metres.
     */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Ordered corners of the closed wall boundary in the map's
     *                  world frame, metres; empty when not closed.
     */
    std::vector<Eigen::Vector3d> boundaryCorners_world_m;

    /*!
     * @brief           Ids of the room's wall planes, in the room's own order.
     *                  Resolved by the map that owns the planes.
     */
    std::vector<int> wallPlaneIds;

    /*!
     * @brief           Ids of the passages associated with the room.
     */
    std::vector<int> passageIds;

    /*!
     * @brief           Id of the storey holding the room; empty when none.
     */
    std::optional<int> floorId;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_ROOM_ENTRY_STRUCT_H \
        */
