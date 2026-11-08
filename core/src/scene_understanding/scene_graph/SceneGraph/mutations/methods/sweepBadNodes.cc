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
 * @file            sweepBadNodes.cc
 *
 * @brief           Implements SceneGraph::sweepBadNodes(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <utility>
#include <vector>

#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::sweepBadNodes()
{
    /* Step 1, graph lock exclusive: flagged nodes leave the tree and the
     * whole retired batch is taken out of the shared list, so a node retired
     * by another thread while the later steps run waits for the next sweep
     * and is never freed unscanned. */
    RetiredNodes                  retiredBatch{};
    std::vector<map::Map *>       slamMaps{};
    std::vector<semantic::Room *> retiredRoomPointers{};
    {
        std::unique_lock<std::shared_mutex> graphLock(graphMutex);
        for (std::pair<const unsigned long, std::unique_ptr<SceneMap>>
                 &sceneMapEntry : sceneMaps)
        {
            retireBadNodes(*sceneMapEntry.second, retiredNodes);
            if (sceneMapEntry.second->p_map != nullptr)
            {
                slamMaps.push_back(sceneMapEntry.second->p_map);
            }
        }
        retiredBatch = std::move(retiredNodes);
        retiredNodes = RetiredNodes{};
    }
    for (const std::unique_ptr<semantic::Room> &p_retiredRoom :
         retiredBatch.rooms)
    {
        retiredRoomPointers.push_back(p_retiredRoom.get());
    }

    /* Step 2, no graph lock (lock order: a SLAM map is never locked under the
     * graph mutex): maps drop their starting and final room links. */
    for (map::Map *p_slamMap : slamMaps)
    {
        for (semantic::Room *p_retiredRoom : retiredRoomPointers)
        {
            if (p_slamMap->replaceLinkedRoom(p_retiredRoom, nullptr) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: replaceLinkedRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Step 3, graph lock exclusive: every other holder lets go, then the batch
     * is freed unless a reader holds the reclaim guard. The try-lock never
     * waits, so a reader can never stall the cycle. */
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    unlinkRetiredNodes(sceneMaps, retiredBatch);
    std::unique_lock<std::shared_mutex> reclaimLock(reclaimMutex,
                                                    std::try_to_lock);
    if (reclaimLock.owns_lock())
    {
        retiredBatch = RetiredNodes{};
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    for (std::unique_ptr<semantic::Room> &p_room : retiredBatch.rooms)
    {
        retiredNodes.rooms.push_back(std::move(p_room));
    }
    for (std::unique_ptr<semantic::Passage> &p_passage : retiredBatch.passages)
    {
        retiredNodes.passages.push_back(std::move(p_passage));
    }
    for (std::unique_ptr<semantic::Floor> &p_floor : retiredBatch.floors)
    {
        retiredNodes.floors.push_back(std::move(p_floor));
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
