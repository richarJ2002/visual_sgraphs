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
 * @file            retireBadNodes.cc
 *
 * @brief           Implements retireBadNodes(), declared in
 *                  scene_graph/SceneGraph/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

#include <utility>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

void retireBadNodes(SceneMap &sceneMap_inout, RetiredNodes &retiredNodes_inout)
{
    retireDeadEntries(sceneMap_inout.detectedRooms,
                      &isRoomLive,
                      retiredNodes_inout.rooms);
    retireDeadEntries(sceneMap_inout.candidateRooms,
                      &isRoomLive,
                      retiredNodes_inout.rooms);

    /* The staircase index borrows its passages: a retired passage leaves it
     * first, so the index never points at a retired object. */
    for (const std::pair<const int, std::unique_ptr<semantic::Passage>>
             &passageEntry : sceneMap_inout.passages)
    {
        if (!isPassageLive(passageEntry.second.get()))
        {
            for (std::pair<const int, std::unique_ptr<Building>>
                     &buildingEntry : sceneMap_inout.buildings)
            {
                eraseEntriesByPointer(buildingEntry.second->passages,
                                      passageEntry.second.get());
            }
        }
    }
    retireDeadEntries(sceneMap_inout.passages,
                      &isPassageLive,
                      retiredNodes_inout.passages);

    for (std::pair<const int, std::unique_ptr<Building>> &buildingEntry :
         sceneMap_inout.buildings)
    {
        retireDeadEntries(buildingEntry.second->floors,
                          &isFloorLive,
                          retiredNodes_inout.floors);
    }
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
