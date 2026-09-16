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
 * @file            countFloorRecordsWithKey.cc
 *
 * @brief           Implements countFloorRecordsWithKey(), declared in
 *                  private_functions.h.
 *
 *                  Snapshot-wide (like countRoomRecordsWithKey() and
 *                  countWallRecordsWithKey()), summing across every
 *                  MapSnapshot whose own mapId equals key_in.mapId, rather
 *                  than being scoped to one caller-chosen MapSnapshot -- a
 *                  duplicate MapSnapshot::mapId must not let a same-key
 *                  floor duplicated across the two map snapshots escape
 *                  detection.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::size_t countFloorRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                     const EntityKey             &key_in)
{
    std::size_t matchCount = 0U;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != key_in.mapId)
        {
            continue;
        }
        for (const FloorRecord &floor : mapSnapshot.floors)
        {
            if (floor.key == key_in)
            {
                ++matchCount;
            }
        }
    }
    return matchCount;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
