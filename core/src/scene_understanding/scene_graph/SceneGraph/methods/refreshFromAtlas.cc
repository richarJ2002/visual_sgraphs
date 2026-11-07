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
 * @file            refreshFromAtlas.cc
 *
 * @brief           Implements SceneGraph::refreshFromAtlas(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "map_state/atlas/atlas.h"
#include "map_state/map/map.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::refreshFromAtlas(atlas::Atlas &atlas_inout)
{
    /* The tree is rebuilt whole each time, so nothing stale survives a map
     * merge or an erased room. */
    sceneMaps.clear();
    activeMapId.reset();

    std::vector<map::Map *> allMaps{};
    map::Map               *p_currentMap = nullptr;
    if ((atlas_inout.getAllMaps(allMaps) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (atlas_inout.getCurrentMap(p_currentMap) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps/getCurrentMap returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    for (map::Map *p_map : allMaps)
    {
        unsigned long mapId{};
        if (p_map == nullptr ||
            p_map->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
        {
            continue;
        }

        std::unique_ptr<SceneMap> p_sceneMap = std::make_unique<SceneMap>();
        p_sceneMap->p_map                    = p_map;
        std::unique_ptr<Building> p_building = std::make_unique<Building>();
        if (mirrorMapIndexes(*p_map, *p_sceneMap) !=
                SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS ||
            mirrorMapBuilding(*p_map, *p_building) !=
                SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mirrorMapIndexes/mirrorMapBuilding returned a "
                         "failure status although they cannot fail; "
                         "continuing as before.",
                         __func__);
        }
        /* One building per map until buildings are detected. */
        p_sceneMap->buildings[0] = std::move(p_building);

        if (p_map == p_currentMap)
        {
            activeMapId = mapId;
        }
        sceneMaps[mapId] = std::move(p_sceneMap);
    }

    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
