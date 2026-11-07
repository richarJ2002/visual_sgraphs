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
 * @file            addFloor.cc
 *
 * @brief           Implements SceneGraph::addFloor(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

#include "scene_understanding/scene_graph/Floor/Floor.h"
#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::addFloor(unsigned long    mapId_in,
                                      semantic::Floor *p_floor_inout)
{
    if (p_floor_inout == nullptr)
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, std::unique_ptr<SceneMap>>::iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    SceneMap &sceneMap = *sceneMapIterator->second;
    int       floorId{};
    if (p_floor_inout->getId(floorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* An entry for this storey under an older id is dropped first. */
    for (std::pair<const int, std::unique_ptr<Building>> &buildingEntry :
         sceneMap.buildings)
    {
        std::map<int, semantic::Floor *> &floorsById =
            buildingEntry.second->floors;
        for (std::map<int, semantic::Floor *>::iterator floorIterator =
                 floorsById.begin();
             floorIterator != floorsById.end();)
        {
            floorIterator = floorIterator->second == p_floor_inout &&
                                    floorIterator->first != floorId
                                ? floorsById.erase(floorIterator)
                                : std::next(floorIterator);
        }
    }

    /* A storey whose id is negative or held by another storey moves to the
     * next free id; ids handed out are never reused. */
    semantic::Floor *p_existingFloor = findFloorInBuildings(sceneMap, floorId);
    if (floorId < 0 ||
        (p_existingFloor != nullptr && p_existingFloor != p_floor_inout))
    {
        while (findFloorInBuildings(sceneMap, sceneMap.nextAvailableFloorId) !=
               nullptr)
        {
            sceneMap.nextAvailableFloorId++;
        }
        const int replacementFloorId = sceneMap.nextAvailableFloorId++;
        std::cerr << "[Map] Floor ID collision for " << floorId
                  << "; reassigned to " << replacementFloorId << "."
                  << std::endl;
        if (p_floor_inout->setId(replacementFloorId) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        floorId = replacementFloorId;
    }
    else
    {
        sceneMap.nextAvailableFloorId =
            std::max(sceneMap.nextAvailableFloorId, floorId + 1);
    }

    std::unique_ptr<Building> &p_building = sceneMap.buildings[0];
    if (p_building == nullptr)
    {
        p_building = std::make_unique<Building>();
    }
    p_building->floors.insert_or_assign(floorId, p_floor_inout);
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
