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
 * @file            verifyAgainstAtlas.cc
 *
 * @brief           Implements SceneGraph::verifyAgainstAtlas(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <string>
#include <vector>

#include "map_state/atlas/atlas.h"
#include "map_state/map/map.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus
    SceneGraph::verifyAgainstAtlas(atlas::Atlas &atlas_inout,
                                   std::string  &mismatchReport_out) const
{
    mismatchReport_out.clear();

    /* The atlas is read before the graph lock: lock order is atlasMutex,
     * then the graph lock. */
    std::vector<map::Map *> allMaps{};
    if (atlas_inout.getAllMaps(allMaps) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::shared_lock<std::shared_mutex> graphLock(graphMutex);
    if (allMaps.size() != sceneMaps.size())
    {
        mismatchReport_out += "maps " + std::to_string(sceneMaps.size()) +
                              " in tree, " + std::to_string(allMaps.size()) +
                              " in atlas; ";
    }

    for (map::Map *p_map : allMaps)
    {
        unsigned long mapId{};
        if (p_map == nullptr ||
            p_map->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
        {
            continue;
        }
        const std::string mapLabel = "map " + std::to_string(mapId) + ": ";
        std::map<unsigned long, std::unique_ptr<SceneMap>>::const_iterator
            sceneMapIterator = sceneMaps.find(mapId);
        if (sceneMapIterator == sceneMaps.end())
        {
            mismatchReport_out += mapLabel + "missing from tree; ";
            continue;
        }
        const SceneMap &sceneMap = *sceneMapIterator->second;

        std::vector<semantic::Floor *>   mapFloors{};
        std::vector<semantic::Room *>    detectedRooms{};
        std::vector<semantic::Room *>    candidateRooms{};
        std::vector<semantic::Passage *> mapPassages{};
        if ((p_map->getAllFloors(mapFloors) !=
             map::MapStatus::MAP_STATUS_SUCCESS) ||
            (p_map->getAllDetectedMapRooms(detectedRooms) !=
             map::MapStatus::MAP_STATUS_SUCCESS) ||
            (p_map->getAllCandidateMapRooms(candidateRooms) !=
             map::MapStatus::MAP_STATUS_SUCCESS) ||
            (p_map->getAllPassages(mapPassages) !=
             map::MapStatus::MAP_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: "
                "getAllFloors/getAllDetectedMapRooms/getAllCandidateMapRooms/"
                "getAllPassages returned a failure status although they "
                "cannot fail; continuing as before.",
                __func__);
        }

        std::size_t treeFloorCount = 0U;
        for (const std::pair<const int, std::unique_ptr<Building>>
                 &buildingEntry : sceneMap.buildings)
        {
            treeFloorCount += buildingEntry.second->floors.size();
        }

        if (treeFloorCount != mapFloors.size())
        {
            mismatchReport_out +=
                mapLabel + "floors " + std::to_string(treeFloorCount) +
                " in tree, " + std::to_string(mapFloors.size()) + " in lists; ";
        }
        if (sceneMap.detectedRooms.size() != detectedRooms.size())
        {
            mismatchReport_out +=
                mapLabel + "detected rooms " +
                std::to_string(sceneMap.detectedRooms.size()) + " in tree, " +
                std::to_string(detectedRooms.size()) + " in lists; ";
        }
        if (sceneMap.candidateRooms.size() != candidateRooms.size())
        {
            mismatchReport_out +=
                mapLabel + "candidate rooms " +
                std::to_string(sceneMap.candidateRooms.size()) + " in tree, " +
                std::to_string(candidateRooms.size()) + " in lists; ";
        }
        if (sceneMap.passages.size() != mapPassages.size())
        {
            mismatchReport_out +=
                mapLabel + "passages " +
                std::to_string(sceneMap.passages.size()) + " in tree, " +
                std::to_string(mapPassages.size()) + " in lists; ";
        }
    }

    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
