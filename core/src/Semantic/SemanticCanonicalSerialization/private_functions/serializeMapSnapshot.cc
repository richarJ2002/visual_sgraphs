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

/*!
 * @file            serializeMapSnapshot.cc
 *
 * @brief           Implements serializeMapSnapshot(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeMapSnapshot(const MapSnapshot &value_in,
                                    bool               includeGeometry_in)
{
    nlohmann::json json;
    json["mapId"]        = value_in.mapId;
    json["isCurrentMap"] = value_in.isCurrentMap;

    /* Two record-collision comparator sets, not SemanticGraphSnapshot's own
     * sortByKey<>()/isValueLessForCollisionTiebreak(): this module owns its
     * own comparators (rather than including a sibling module's private
     * header -- CPP_CODING_STANDARD.md Section 5.4) and additionally needs
     * a topology-only order that never inspects a geometric field, so a
     * geometry-only perturbation of otherwise key-colliding records cannot
     * reorder or alter the topology-only projection's bytes. */
    std::vector<RoomRecord> rooms = value_in.rooms;
    std::sort(rooms.begin(),
              rooms.end(),
              includeGeometry_in ? &isRoomRecordLessFullGeometry
                                 : &isRoomRecordLessTopologyOnly);
    nlohmann::json roomsJson = nlohmann::json::array();
    for (const RoomRecord &room : rooms)
    {
        roomsJson.push_back(serializeRoomRecord(room, includeGeometry_in));
    }
    json["rooms"] = std::move(roomsJson);

    std::vector<WallRecord> walls = value_in.walls;
    std::sort(walls.begin(),
              walls.end(),
              includeGeometry_in ? &isWallRecordLessFullGeometry
                                 : &isWallRecordLessTopologyOnly);
    nlohmann::json wallsJson = nlohmann::json::array();
    for (const WallRecord &wall : walls)
    {
        wallsJson.push_back(serializeWallRecord(wall, includeGeometry_in));
    }
    json["walls"] = std::move(wallsJson);

    std::vector<PassageRecord> passages = value_in.passages;
    std::sort(passages.begin(),
              passages.end(),
              includeGeometry_in ? &isPassageRecordLessFullGeometry
                                 : &isPassageRecordLessTopologyOnly);
    nlohmann::json passagesJson = nlohmann::json::array();
    for (const PassageRecord &passage : passages)
    {
        passagesJson.push_back(
            serializePassageRecord(passage, includeGeometry_in));
    }
    json["passages"] = std::move(passagesJson);

    std::vector<FloorRecord> floors = value_in.floors;
    std::sort(floors.begin(),
              floors.end(),
              includeGeometry_in ? &isFloorRecordLessFullGeometry
                                 : &isFloorRecordLessTopologyOnly);
    nlohmann::json floorsJson = nlohmann::json::array();
    for (const FloorRecord &floor : floors)
    {
        floorsJson.push_back(serializeFloorRecord(floor, includeGeometry_in));
    }
    json["floors"] = std::move(floorsJson);

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
