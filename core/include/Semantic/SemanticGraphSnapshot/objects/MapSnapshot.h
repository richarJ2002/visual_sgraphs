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
 * @file            MapSnapshot.h
 *
 * @brief           Declares every entity captured from one Atlas map.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_MAP_SNAPSHOT_H
#define SEMANTIC_GRAPH_SNAPSHOT_MAP_SNAPSHOT_H

#include <vector>

#include "Semantic/SemanticGraphSnapshot/objects/FloorRecord.h"
#include "Semantic/SemanticGraphSnapshot/objects/PassageRecord.h"
#include "Semantic/SemanticGraphSnapshot/objects/RoomRecord.h"
#include "Semantic/SemanticGraphSnapshot/objects/WallRecord.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Every entity captured from one Atlas map, live and bad
 *              alike -- see each record type's own isLive field. Neither
 *              this type nor any record vector it holds filters on
 *              liveness; a retired room, wall, or passage remains present
 *              here for the evaluator to judge.
 */
struct MapSnapshot
{
  public:
    /*! @brief Atlas::Map::GetId() of the captured map. */
    long unsigned int mapId{0U};

    /*! @brief True for the single map SemanticGraphSnapshot::currentMapId
     *  names at capture time -- only meaningful when currentMapStatus ==
     *  AtlasCurrentMapStatus::CURRENT_MAP_ACTIVE; see its Doxygen. */
    bool isCurrentMap{false};

    /*! @brief Sorted by RoomRecord::key. */
    std::vector<RoomRecord> rooms;

    /*! @brief Sorted by WallRecord::key. */
    std::vector<WallRecord> walls;

    /*! @brief Sorted by PassageRecord::key. */
    std::vector<PassageRecord> passages;

    /*! @brief Sorted by FloorRecord::key. */
    std::vector<FloorRecord> floors;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_MAP_SNAPSHOT_H
