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
 * @file            retireSceneMapContents.cc
 *
 * @brief           Implements retireSceneMapContents(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <utility>

namespace vs_graphs
{
namespace core
{

void retireSceneMapContents(scene_graph::SceneMap &sceneMap_inout,
                            RetiredNodes          &retiredNodes_inout)
{
    for (std::pair<const int, std::unique_ptr<semantic::Room>> &roomEntry :
         sceneMap_inout.detectedRooms)
    {
        retiredNodes_inout.rooms.push_back(std::move(roomEntry.second));
    }
    for (std::pair<const int, std::unique_ptr<semantic::Room>> &roomEntry :
         sceneMap_inout.candidateRooms)
    {
        retiredNodes_inout.rooms.push_back(std::move(roomEntry.second));
    }
    for (std::pair<const int, std::unique_ptr<semantic::Passage>>
             &passageEntry : sceneMap_inout.passages)
    {
        retiredNodes_inout.passages.push_back(std::move(passageEntry.second));
    }
    sceneMap_inout.detectedRooms.clear();
    sceneMap_inout.candidateRooms.clear();
    sceneMap_inout.passages.clear();

    /* Buildings stay: building 0 is part of every scene map. */
    for (std::pair<const int, std::unique_ptr<scene_graph::Building>>
             &buildingEntry : sceneMap_inout.buildings)
    {
        for (std::pair<const int, std::unique_ptr<semantic::Floor>>
                 &floorEntry : buildingEntry.second->floors)
        {
            retiredNodes_inout.floors.push_back(std::move(floorEntry.second));
        }
        buildingEntry.second->floors.clear();
        buildingEntry.second->passages.clear();
    }
}

} /* namespace core */
} /* namespace vs_graphs */
