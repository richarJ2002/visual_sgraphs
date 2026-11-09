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
 * @file            RoomSnapshotStruct.h
 *
 * @brief           Declares RoomSnapshot, the plain-data copy of a room that
 *                  code outside the semantics manager works with.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_SNAPSHOT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_SNAPSHOT_STRUCT_H

#include <Eigen/Core>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What optimisation and relocalization need to know about a
 *                  room, copied out under the graph lock. Holds no pointer into
 *                  the graph, so it stays valid after the lock is released.
 */
struct RoomSnapshot
{
  public:
    /*!
     * @brief           Room id, unique within its map.
     */
    int id{-1};

    /*!
     * @brief           True for a detected room, false for a candidate room.
     */
    bool isDetected{false};

    /*!
     * @brief           True when the room's wall boundary is closed.
     */
    bool isBoundaryComplete{false};

    /*!
     * @brief           Room centroid in the map's world frame, metres.
     */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Ids of the room's wall planes, in the room's own order.
     *                  Plane ids are resolved by the map that owns the planes.
     */
    std::vector<int> wallPlaneIds;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ROOM_SNAPSHOT_STRUCT_H  \
        */
