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
 * @file            unlinkRetiredNodes.cc
 *
 * @brief           Implements unlinkRetiredNodes(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <memory>
#include <vector>

namespace vs_graphs
{
namespace core
{

void unlinkRetiredNodes(
    const std::map<unsigned long, scene_graph::SceneGraph *> &sceneGraphs_in,
    const RetiredNodes                                       &retiredNodes_in)
{
    /* Everything still in the tree is live: retireBadNodes() ran first. */
    std::vector<semantic::Room *>    liveRooms{};
    std::vector<semantic::Passage *> livePassages{};
    std::vector<semantic::Floor *>   liveFloors{};
    for (const std::pair<const unsigned long, scene_graph::SceneGraph *>
             &sceneGraphEntry : sceneGraphs_in)
    {
        for (const std::pair<const int, std::unique_ptr<semantic::Room>>
                 &roomEntry : sceneGraphEntry.second->detectedRooms)
        {
            liveRooms.push_back(roomEntry.second.get());
        }
        for (const std::pair<const int, std::unique_ptr<semantic::Room>>
                 &roomEntry : sceneGraphEntry.second->candidateRooms)
        {
            liveRooms.push_back(roomEntry.second.get());
        }
        for (const std::pair<const int, std::unique_ptr<semantic::Passage>>
                 &passageEntry : sceneGraphEntry.second->passages)
        {
            livePassages.push_back(passageEntry.second.get());
        }
        for (const std::pair<const int, std::unique_ptr<scene_graph::Building>>
                 &buildingEntry : sceneGraphEntry.second->buildings)
        {
            for (const std::pair<const int, std::unique_ptr<semantic::Floor>>
                     &floorEntry : buildingEntry.second->floors)
            {
                liveFloors.push_back(floorEntry.second.get());
            }
        }
    }

    unlinkRetiredRooms(retiredNodes_in.rooms, liveFloors, livePassages);
    unlinkRetiredPassages(retiredNodes_in.passages, liveRooms);
    unlinkRetiredFloors(retiredNodes_in.floors, liveRooms);
}

} /* namespace core */
} /* namespace vs_graphs */
