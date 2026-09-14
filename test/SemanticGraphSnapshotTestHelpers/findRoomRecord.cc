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
 * @file            findRoomRecord.cc
 *
 * @brief           Implements findRoomRecord(), declared in
 *                  SemanticGraphSnapshotTestHelpers.h.
 */

#include "SemanticGraphSnapshotTestHelpers.h"

namespace ORB_SLAM3
{
namespace semantic
{

const RoomRecord *findRoomRecord(const MapSnapshot &mapSnapshot_in,
                                 int                entityId_in)
{
    for (const RoomRecord &record : mapSnapshot_in.rooms)
    {
        if (record.key.entityId == entityId_in)
        {
            return &record;
        }
    }
    return nullptr;
}

} // namespace semantic
} // namespace ORB_SLAM3
