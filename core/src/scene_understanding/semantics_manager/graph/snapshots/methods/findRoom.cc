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
 * @file            findRoom.cc
 *
 * @brief           Implements SceneGraphSnapshot::findRoom(), declared in
 *                  semantics_manager/objects/SceneGraphSnapshotStruct.h.
 */

#include "scene_understanding/semantics_manager/objects/SceneGraphSnapshotStruct.h"

namespace vs_graphs
{
namespace core
{

const SceneGraphRoomEntry *SceneGraphSnapshot::findRoom(int roomId_in) const
{
    for (const SceneGraphRoomEntry &roomEntry : rooms)
    {
        if (roomEntry.id == roomId_in)
        {
            return &roomEntry;
        }
    }
    return nullptr;
}

} /* namespace core */
} /* namespace vs_graphs */
