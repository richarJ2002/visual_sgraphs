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

SemanticGraphSnapshot captureSemanticGraphSnapshot(core::Atlas *p_atlas_in)
{
    SemanticGraphSnapshot snapshot;
    if (p_atlas_in == nullptr)
    {
        return snapshot;
    }

    /* GetCoherentMapView(), unlike GetCurrentMap(), never creates a map as a
     * read side effect and reads currentMapId, currentMapStatus, and maps
     * below under one Atlas-lock critical section, so all three always
     * describe one coherent instant. currentMapStatus must be checked
     * before trusting that currentMapId names an entry in maps -- see
     * SemanticGraphSnapshot::currentMapStatus's Doxygen and
     * AtlasCurrentMapStatus.h. */
    const std::vector<core::Map *> maps =
        p_atlas_in->getCoherentMapView(snapshot.currentMapId,
                                       snapshot.currentMapStatus);

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
        const long unsigned int mapId = p_map->getId();
        std::set<Room *>        roomsInMap;
        for (Room *p_room : p_map->getAllDetectedMapRooms())
        {
            if (p_room != nullptr)
            {
                roomsInMap.insert(p_room);
            }
        }
        for (Room *p_room : p_map->getAllMarkerBasedMapRooms())
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
                // getId cannot fail; continue as before.
            }
            ownerReference.key    = makeKey(EntityKind::ROOM, mapId, roomId);
            ownerReference.reason = UnavailableReason::NONE;
            int roomId2{};
            if (p_room->getId(roomId2) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            ownerReference.localId = roomId2;
            bool roomIsBad{};
            if (p_room->isBad(roomIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            ownerReference.isLive                    = !roomIsBad;
            ownerReference.livenessUnavailableReason = UnavailableReason::NONE;
            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
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

        MapSnapshot mapSnapshot;
        mapSnapshot.mapId = p_map->getId();
        mapSnapshot.isCurrentMap =
            snapshot.currentMapId.has_value() &&
            (*snapshot.currentMapId == mapSnapshot.mapId);

        /* Detected and marker-based/candidate membership are captured
         * separately per RoomRecord (isDetectedMember/isMarkerBasedMember)
         * so a room present in both collections stays diagnosable rather
         * than being collapsed by a single union enumeration. */
        std::set<Room *> detectedRooms;
        for (Room *p_room : p_map->getAllDetectedMapRooms())
        {
            if (p_room != nullptr)
            {
                detectedRooms.insert(p_room);
            }
        }
        std::set<Room *> markerBasedRooms;
        for (Room *p_room : p_map->getAllMarkerBasedMapRooms())
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
            mapSnapshot.rooms.push_back(
                captureRoom(p_room,
                            mapSnapshot.mapId,
                            detectedRooms.count(p_room) > 0,
                            markerBasedRooms.count(p_room) > 0));
        }
        sortByKey(mapSnapshot.rooms);

        for (geometric::Plane *p_plane : p_map->getAllPlanes())
        {
            if (p_plane == nullptr ||
                p_plane->getPlaneType() != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }
            mapSnapshot.walls.push_back(
                captureWall(p_plane, mapSnapshot.mapId, wallOwnersByPointer));
        }
        sortByKey(mapSnapshot.walls);

        for (Passage *p_passage : p_map->getAllPassages())
        {
            if (p_passage == nullptr)
            {
                continue;
            }
            mapSnapshot.passages.push_back(
                capturePassage(p_passage, mapSnapshot.mapId));
        }
        sortByKey(mapSnapshot.passages);

        for (Floor *p_floor : p_map->getAllFloors())
        {
            if (p_floor == nullptr)
            {
                continue;
            }
            mapSnapshot.floors.push_back(
                captureFloor(p_floor, mapSnapshot.mapId));
        }
        sortByKey(mapSnapshot.floors);

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

    return snapshot;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
