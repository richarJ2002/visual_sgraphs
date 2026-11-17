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

/*!
 * @file            ensureBootstrapFloor.cc
 *
 * @brief           Implements SemanticsManager::ensureBootstrapFloor(),
 *                  declared in
semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>
#include <vector>

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"

namespace vs_graphs
{
namespace core
{

semantic::Floor *SemanticsManager::ensureBootstrapFloor(unsigned long mapId_in)
{
    std::vector<semantic::Floor *> floors{};
    semantic::Floor               *p_canonicalFloor = nullptr;
    if (getMapFloorsInIdOrder(mapId_in, floors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        semantic::Floor::selectBestObservedFloor(floors, p_canonicalFloor) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the floor query returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    if (p_canonicalFloor != nullptr)
    {
        return p_canonicalFloor;
    }

    /* The map has no storey yet. The tree is built top-down: the building
     * exists before its storey. */
    if (addBuilding(mapId_in, 0) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return nullptr;
    }
    if (utils::geo_sem_helpers::GeoSemHelpers::createMapFloor(p_atlas) !=
            utils::geo_sem_helpers::GeoSemHelpersStatus::
                GEO_SEM_HELPERS_STATUS_SUCCESS ||
        getMapFloorsInIdOrder(mapId_in, floors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        semantic::Floor::selectBestObservedFloor(floors, p_canonicalFloor) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createMapFloor or the floor query returned a "
                     "failure status; continuing as before.",
                     __func__);
    }
    return p_canonicalFloor;
}

} /* namespace core */
} /* namespace vs_graphs */
