/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            RoomContextSnapshotStruct.h
 *
 * @brief           Declares RoomContextSnapshot, a saved description of a room
 * used to recognise it again.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_ROOM_CONTEXT_SNAPSHOT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_ROOM_CONTEXT_SNAPSHOT_STRUCT_H

#include "semantic/room/objects/PassageContextStruct.h"
#include "semantic/room/objects/WallBoundsStruct.h"

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{





/*!
 * @brief           Saved description of one room (walls, passages, tag) taken
 *                  from a map, used to recognise the same room again later.
 *                  The wall vectors share one index per wall and the passage
 *                  vectors one index per passage.
 */
struct RoomContextSnapshot
{
    /*!
     * @brief           Id of the room this snapshot was taken from.
     */
    int                          roomId{0};
    /*!
     * @brief           The room's floor at snapshot time, -1 when the room had
     *                  no floor identity yet. A room is always floor-scoped
     *                  (Room::getFloor()), so this is captured alongside roomId
     *                  rather than re-derived later -- by the time this
     *                  snapshot is consumed (e.g. reacquisition after a
     *                  tracking loss), the original Room object may no longer
     *                  be the cheapest way to answer "what floor was the UAV
     *                  on". A future building-node level would extend this the
     *                  same way, once that concept exists.
     */
    int                          floorId{-1};
    /*!
     * @brief           Room centroid in the world frame, metres.
     */
    Eigen::Vector3d              centroid{Eigen::Vector3d::Zero()};
    /*!
     * @brief           Per wall, unit normal in the world frame pointing toward
     *                  the room; NaN when the wall was bad or had no normal.
     */
    std::vector<Eigen::Vector3d> wallNormals;
    /*!
     * @brief           Per wall, wall centroid in the world frame, metres; NaN
     *                  when the wall was bad.
     */
    std::vector<Eigen::Vector3d> wallCentroids;
    /*!
     * @brief           Per wall, distance of the wall plane from the world
     *                  origin, metres; NaN when the wall was bad.
     */
    std::vector<double>          wallDistances;
    /*!
     * @brief           Per wall, in-plane extent of the wall; default (invalid)
     *                  when the wall was bad.
     */
    std::vector<WallBounds>      wallBounds;
    /*!
     * @brief           Per passage, passage centre in the world frame, metres;
     *                  NaN when the passage pointer was null.
     */
    std::vector<Eigen::Vector3d> passageCentroids;
    /*!
     * @brief           Per passage, saved passage description, in the same
     *                  order as passageCentroids.
     */
    std::vector<PassageContext>  passageContexts;
    /*!
     * @brief           Time the snapshot was taken, seconds since the
     *                  std::chrono::steady_clock epoch.
     */
    double                       timestamp{0.0};
    /*!
     * @brief           Persistent room tag, "room_<roomId>", used to match
     *                  rooms across maps.
     */
    std::string                  roomTag;
    /*!
     * @brief           True when the room had been promoted to a confirmed
     *                  room at snapshot time, false while it was still a
     *                  candidate.
     */
    bool                         wasConfirmedRoom{false};
    /*!
     * @brief           Whether the UAV had entered the room at snapshot time.
     *                  Mission truth restored alongside identity; never
     *                  consulted by creation, promotion, retirement, or merge
     *                  paths.
     */
    bool                         wasPreviouslyVisited{false};
    /*!
     * @brief           Room::BoundaryStatus at snapshot time, stored as its
     *                  integer value (0 unobserved, 1 incomplete, 2 complete,
     *                  3 conflicting).
     */
    int                          boundaryStatus{0};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_ROOM_CONTEXT_SNAPSHOT_STRUCT_H \
        */
