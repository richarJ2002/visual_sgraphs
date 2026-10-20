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
 * @file            SemanticMergeRoomEvidenceStruct.h
 *
 * @brief           Declares SemanticMergeRoomEvidence, copied evidence for one
 * room.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_ROOM_EVIDENCE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_ROOM_EVIDENCE_STRUCT_H

#include "scene_understanding/semantic/room/objects/RoomContextSnapshotStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/VerifyWallObservationStruct.h"

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Copied evidence for one room. No live map pointer escapes
 *                  the merge lock.
 */
struct SemanticMergeRoomEvidence
{
    /*!
     * @brief           Snapshot of the room's identity, floor, centroid and
     *                  passages.
     */
    RoomContextSnapshot                context;
    /*!
     * @brief           Wall observations of the room, in its own map's world
     *                  frame.
     */
    std::vector<VerifyWallObservation> walls;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_ROOM_EVIDENCE_STRUCT_H \
        */
