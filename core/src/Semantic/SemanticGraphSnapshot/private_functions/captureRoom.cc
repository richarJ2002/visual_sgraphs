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
 * @file            captureRoom.cc
 *
 * @brief           Implements captureRoom(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

#include "Map.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus captureRoom(Room             *p_room_in,
                                        long unsigned int mapId_in,
                                        bool              isDetectedMember_in,
                                        bool        isMarkerBasedMember_in,
                                        RoomRecord &roomRecord_out)
{
    RoomRecord record;
    int        room_inId{};
    if (p_room_in->getId(room_inId) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    EntityKey key2{};
    if (makeKey(EntityKind::ROOM, mapId_in, room_inId, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeKey returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.key = key2;
    bool room_inIsBad{};
    if (p_room_in->isBad(room_inIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.isLive              = !room_inIsBad;
    record.isDetectedMember    = isDetectedMember_in;
    record.isMarkerBasedMember = isMarkerBasedMember_in;

    core::Map *p_declaredMap = nullptr;
    if (p_room_in->getMap(p_declaredMap) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->getId();
    }

    Room::RoomVariant room_inRoomVariant{};
    if (p_room_in->getRoomVariant(room_inRoomVariant) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomVariant returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.variant = room_inRoomVariant;
    Eigen::Vector3d room_inCentroid{};
    if (p_room_in->getCentroid(room_inCentroid) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.centroid_World_m = room_inCentroid;
    Room::BoundaryStatus room_inBoundaryStatus{};
    if (p_room_in->getBoundaryStatus(room_inBoundaryStatus) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBoundaryStatus returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    record.boundaryStatus = room_inBoundaryStatus;
    std::vector<Eigen::Vector3d> room_inBoundaryCorners_World_m{};
    if (p_room_in->getBoundaryCorners_World_m(room_inBoundaryCorners_World_m) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBoundaryCorners_World_m returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    record.boundaryCorners_World_m = room_inBoundaryCorners_World_m;
    std::vector<Room::ObservationGap> room_inObservationGaps{};
    if (p_room_in->getObservationGaps(room_inObservationGaps) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationGaps returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    record.observationGaps = room_inObservationGaps;

    std::vector<geometric::Plane *> room_inWalls{};
    if (p_room_in->getWalls(room_inWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_wall : room_inWalls)
    {
        if (appendWallRef(p_wall, record.wallRefs) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendWallRef returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
    /* Full value-based total order (see isRawPlaneRefLess()), not merely a
     * stable pass-through of pre-sort order. */
    std::sort(record.wallRefs.begin(),
              record.wallRefs.end(),
              &isRawPlaneRefLess);

    std::vector<vs_graphs::core::semantic::Passage *> room_inPassages{};
    if (p_room_in->getPassages(room_inPassages) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Passage *p_passage : room_inPassages)
    {
        if (appendPassageRef(p_passage, record.passageRefs) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendPassageRef returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    std::sort(record.passageRefs.begin(),
              record.passageRefs.end(),
              &isEntityRefLess);

    Floor *p_room_inFloor = nullptr;
    if (p_room_in->getFloor(p_room_inFloor) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    EntityRef entityRef{};
    if (entityRefForFloor(p_room_inFloor, entityRef) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: entityRefForFloor returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    record.floorRef                        = entityRef;
    geometric::Plane *p_room_inGroundPlane = nullptr;
    if (p_room_in->getGroundPlane(p_room_inGroundPlane) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGroundPlane returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    RawPlaneRef rawPlaneRef2{};
    if (rawPlaneRef(p_room_inGroundPlane, rawPlaneRef2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rawPlaneRef returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.groundPlaneRef = rawPlaneRef2;

    roomRecord_out = record;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
