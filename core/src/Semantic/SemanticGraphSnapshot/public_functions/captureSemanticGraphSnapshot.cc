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
 * @file            captureSemanticGraphSnapshot.cc
 *
 * @brief           Implements captureSemanticGraphSnapshot(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/public_functions.h"

#include <algorithm>
#include <map>
#include <rclcpp/logging.hpp>
#include <set>

#include "Atlas.h"
#include "Map.h"

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus
    captureSemanticGraphSnapshot(core::Atlas           *p_atlas_in,
                                 SemanticGraphSnapshot &snapshot_out)
{
    SemanticGraphSnapshot snapshot;
    if (p_atlas_in == nullptr)
    {
        snapshot_out = snapshot;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }

    /* GetCoherentMapView(), unlike GetCurrentMap(), never creates a map as a
     * read side effect and reads currentMapId, currentMapStatus, and maps
     * below under one Atlas-lock critical section, so all three always
     * describe one coherent instant. currentMapStatus must be checked
     * before trusting that currentMapId names an entry in maps -- see
     * SemanticGraphSnapshot::currentMapStatus's Doxygen and
     * AtlasCurrentMapStatus.h. */
    std::vector<core::Map *> maps{};
    if (p_atlas_in->getCoherentMapView(snapshot.currentMapId,
                                       snapshot.currentMapStatus,
                                       maps) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCoherentMapView returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Pass 1: invert Room -> Wall ownership across every live map so each
     * WallRecord can carry its owning rooms without Plane storing a
     * reverse pointer. Deliberately not scoped to one map at a time, so a
     * cross-map ownership error stays visible to the evaluator instead of
     * being hidden by only checking within the wall's own map. Rooms are
     * deduplicated by pointer per map (rather than using
     * Map::GetAllRooms(), whose union of mspDetectedRooms and
     * mspMarkerBasedRooms lists a room registered in both collections
     * twice) so a doubly-registered room does not manufacture a duplicate
     * ownership entry that never actually occurred. Each owner is captured
     * as a full EntityRef (not a bare EntityKey) so a bad owning room
     * retains its own liveness evidence in WallRecord::ownerRoomRefs. The
     * owner's key is deliberately built from this loop's own containing
     * mapId (matching every RoomRecord's own key convention), not from
     * entityRefForRoom()'s target-declared-map semantics: a room enumerated
     * from this map while declaring a different (or no) map must still be
     * keyed here exactly as its own RoomRecord is keyed, so the two records
     * are the same lookup key -- entityRefForRoom() is reserved for
     * one-to-one fields naming a relationship target by its own identity
     * (floorRef, knownSideRoomRef, prospectiveRoomRef), which have no such
     * containing-enumeration context. */
    std::map<geometric::Plane *, std::vector<EntityRef>> wallOwnersByPointer;
    for (core::Map *p_map : maps)
    {
        if (p_map == nullptr)
        {
            continue;
        }
        unsigned long mapIdValue{};
        if (p_map->getId(mapIdValue) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const long unsigned int mapId =
            static_cast<long unsigned int>(mapIdValue);
        std::set<Room *>              roomsInMap;
        std::vector<semantic::Room *> mapAllDetectedMapRooms{};
        if (p_map->getAllDetectedMapRooms(mapAllDetectedMapRooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllDetectedMapRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (Room *p_room : mapAllDetectedMapRooms)
        {
            if (p_room != nullptr)
            {
                roomsInMap.insert(p_room);
            }
        }
        std::vector<semantic::Room *> mapAllMarkerBasedMapRooms{};
        if (p_map->getAllMarkerBasedMapRooms(mapAllMarkerBasedMapRooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllMarkerBasedMapRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        for (Room *p_room : mapAllMarkerBasedMapRooms)
        {
            if (p_room != nullptr)
            {
                roomsInMap.insert(p_room);
            }
        }
        for (Room *p_room : roomsInMap)
        {
            EntityRef ownerReference;
            int       roomId{};
            if (p_room->getId(roomId) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            EntityKey key2{};
            if (makeKey(EntityKind::ROOM, mapId, roomId, key2) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: makeKey returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            ownerReference.key    = key2;
            ownerReference.reason = UnavailableReason::NONE;
            int roomId2{};
            if (p_room->getId(roomId2) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            ownerReference.localId = roomId2;
            bool roomIsBad{};
            if (p_room->isBad(roomIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            ownerReference.isLive                    = !roomIsBad;
            ownerReference.livenessUnavailableReason = UnavailableReason::NONE;
            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                if (p_wall == nullptr)
                {
                    continue;
                }
                wallOwnersByPointer[p_wall].push_back(ownerReference);
            }
        }
    }
    for (auto &[p_wall, ownerRefs] : wallOwnersByPointer)
    {
        (void)p_wall;
        std::sort(ownerRefs.begin(), ownerRefs.end(), &isEntityRefLess);
    }

    for (core::Map *p_map : maps)
    {
        if (p_map == nullptr)
        {
            continue;
        }

        MapSnapshot   mapSnapshot;
        unsigned long mapId2{};
        if (p_map->getId(mapId2) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mapSnapshot.mapId = mapId2;
        mapSnapshot.isCurrentMap =
            snapshot.currentMapId.has_value() &&
            (*snapshot.currentMapId == mapSnapshot.mapId);

        /* Detected and marker-based/candidate membership are captured
         * separately per RoomRecord (isDetectedMember/isMarkerBasedMember)
         * so a room present in both collections stays diagnosable rather
         * than being collapsed by a single union enumeration. */
        std::set<Room *>              detectedRooms;
        std::vector<semantic::Room *> mapAllDetectedMapRooms2{};
        if (p_map->getAllDetectedMapRooms(mapAllDetectedMapRooms2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllDetectedMapRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (Room *p_room : mapAllDetectedMapRooms2)
        {
            if (p_room != nullptr)
            {
                detectedRooms.insert(p_room);
            }
        }
        std::set<Room *>              markerBasedRooms;
        std::vector<semantic::Room *> mapAllMarkerBasedMapRooms2{};
        if (p_map->getAllMarkerBasedMapRooms(mapAllMarkerBasedMapRooms2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllMarkerBasedMapRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        for (Room *p_room : mapAllMarkerBasedMapRooms2)
        {
            if (p_room != nullptr)
            {
                markerBasedRooms.insert(p_room);
            }
        }
        std::set<Room *> allRoomsInMap = detectedRooms;
        allRoomsInMap.insert(markerBasedRooms.begin(), markerBasedRooms.end());

        for (Room *p_room : allRoomsInMap)
        {
            RoomRecord roomRecord{};
            if (captureRoom(p_room,
                            mapSnapshot.mapId,
                            detectedRooms.count(p_room) > 0,
                            markerBasedRooms.count(p_room) > 0,
                            roomRecord) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: captureRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapSnapshot.rooms.push_back(roomRecord);
        }
        if (sortByKey(mapSnapshot.rooms) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sortByKey returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<geometric::Plane *> mapAllPlanes{};
        if (p_map->getAllPlanes(mapAllPlanes) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_plane : mapAllPlanes)
        {
            geometric::Plane::PlaneVariant planeType{};
            if (!(p_plane == nullptr) &&
                p_plane->getPlaneType(planeType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane == nullptr ||
                planeType != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }
            WallRecord wallRecord{};
            if (captureWall(p_plane,
                            mapSnapshot.mapId,
                            wallOwnersByPointer,
                            wallRecord) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: captureWall returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapSnapshot.walls.push_back(wallRecord);
        }
        if (sortByKey(mapSnapshot.walls) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sortByKey returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<vs_graphs::core::semantic::Passage *> mapAllPassages{};
        if (p_map->getAllPassages(mapAllPassages) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (Passage *p_passage : mapAllPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }
            PassageRecord passageRecord{};
            if (capturePassage(p_passage, mapSnapshot.mapId, passageRecord) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: capturePassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapSnapshot.passages.push_back(passageRecord);
        }
        if (sortByKey(mapSnapshot.passages) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sortByKey returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<semantic::Floor *> mapAllFloors{};
        if (p_map->getAllFloors(mapAllFloors) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (Floor *p_floor : mapAllFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }
            FloorRecord floorRecord{};
            if (captureFloor(p_floor, mapSnapshot.mapId, floorRecord) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: captureFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            mapSnapshot.floors.push_back(floorRecord);
        }
        if (sortByKey(mapSnapshot.floors) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sortByKey returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        snapshot.maps.push_back(std::move(mapSnapshot));
    }
    /* std::stable_sort for the same collision-safety reason as
     * sortByKey.tpp: GetCoherentMapView() already returns maps sorted by
     * id, so this is a no-op reassertion in the ordinary unique-id case,
     * but never leaves an id collision (which should not occur for
     * distinct Atlas maps, but is not fabricated away here either) with an
     * unspecified relative order. */
    std::stable_sort(snapshot.maps.begin(),
                     snapshot.maps.end(),
                     [](const MapSnapshot &lhs_in, const MapSnapshot &rhs_in)
                     { return lhs_in.mapId < rhs_in.mapId; });

    snapshot_out = snapshot;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
