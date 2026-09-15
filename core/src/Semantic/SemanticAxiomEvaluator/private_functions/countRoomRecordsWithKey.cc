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
 * @file            countRoomRecordsWithKey.cc
 *
 * @brief           Implements countRoomRecordsWithKey(), declared in
 *                  private_functions.h.
 *
 *                  2026-09-07 second proof-closure repair: sums matches
 *                  across every MapSnapshot whose own mapId equals
 *                  key_in.mapId, rather than returning after the first such
 *                  map -- a duplicate MapSnapshot::mapId (adversarial-only;
 *                  unreachable through production capture, see
 *                  captureSemanticGraphSnapshot.cc) must not let a
 *                  same-key room in the second map escape detection.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::size_t countRoomRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                    const EntityKey             &key_in)
{
    std::size_t matchCount = 0U;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != key_in.mapId)
        {
            continue;
        }
        for (const RoomRecord &room : mapSnapshot.rooms)
        {
            if (room.key == key_in)
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
