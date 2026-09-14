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
 * @file            evaluateAxBound01.cc
 *
 * @brief           Implements evaluateAxBound01(), declared in
 *                  private_functions.h. The per-room logic lives in
 *                  evaluateOneRoomBoundary.cc (CPP_CODING_STANDARD.md
 *                  Section 5.4: one ordinary function per .cc).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

void evaluateAxBound01(const SemanticGraphSnapshot &snapshot_in,
                       std::vector<Finding>        &findings_inout)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        for (const RoomRecord &room : mapSnapshot.rooms)
        {
            if (!room.isLive || room.variant != Room::roomVariant::ROOM)
            {
                continue;
            }
            evaluateOneRoomBoundary(room, snapshot_in, findings_inout);
        }
    }
}

} // namespace semantic
} // namespace ORB_SLAM3
