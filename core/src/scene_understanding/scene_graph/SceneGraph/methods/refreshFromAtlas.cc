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

#include <rclcpp/logging.hpp>
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
    /* The atlas is read before any graph lock: lock order is atlasMutex,
     * then the graph lock. Each map is rebuilt under its own exclusive
     * lock by resyncMap(); no lock is held across the calls. */
    std::vector<map::Map *> allMaps{};
    if (atlas_inout.getAllMaps(allMaps) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (map::Map *p_map : allMaps)
    {
        if (p_map != nullptr &&
            resyncMap(*p_map) != SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resyncMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
