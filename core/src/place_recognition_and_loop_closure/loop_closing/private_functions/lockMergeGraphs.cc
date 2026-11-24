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
 * @file            lockMergeGraphs.cc
 *
 * @brief           Implements lockMergeGraphs(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

#include "map_state/atlas/atlas.h"
#include "map_state/map/map.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{

void lockMergeGraphs(atlas::Atlas   *p_atlas_in,
                     map::Map       *p_currentMap_in,
                     map::Map       *p_mergeMap_in,
                     GraphLockMode   mode_in,
                     GraphLockScope &scope_out)
{
    SemanticsManager *p_semanticsManager = nullptr;
    unsigned long     currentMapId{};
    unsigned long     mergeMapId{};
    if ((p_atlas_in->getSemanticsManager(p_semanticsManager) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_currentMap_in->getId(currentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_mergeMap_in->getId(mergeMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_semanticsManager->lockGraphs({currentMapId, mergeMapId},
                                        mode_in,
                                        scope_out) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSemanticsManager/getId/lockGraphs returned a "
                     "failure status although they cannot fail; continuing as "
                     "before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
