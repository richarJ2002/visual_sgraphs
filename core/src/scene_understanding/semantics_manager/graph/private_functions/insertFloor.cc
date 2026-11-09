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
 * @file            insertFloor.cc
 *
 * @brief           Implements insertFloor(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void insertFloor(scene_graph::SceneMap           &sceneMap_inout,
                 std::unique_ptr<semantic::Floor> p_floor_in)
{
    if (p_floor_in == nullptr)
    {
        return;
    }

    int floorId{};
    if (p_floor_in->getId(floorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* A storey whose id is negative or held by another storey moves to the
     * next free id; ids handed out are never reused. */
    if (floorId < 0 || findFloorInBuildings(sceneMap_inout, floorId) != nullptr)
    {
        while (findFloorInBuildings(sceneMap_inout,
                                    sceneMap_inout.nextAvailableFloorId) !=
               nullptr)
        {
            sceneMap_inout.nextAvailableFloorId++;
        }
        const int replacementFloorId = sceneMap_inout.nextAvailableFloorId++;
        std::cerr << "[Map] Floor ID collision for " << floorId
                  << "; reassigned to " << replacementFloorId << "."
                  << std::endl;
        if (p_floor_in->setId(replacementFloorId) !=
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
        sceneMap_inout.nextAvailableFloorId =
            std::max(sceneMap_inout.nextAvailableFloorId, floorId + 1);
    }

    std::unique_ptr<scene_graph::Building> &p_building =
        sceneMap_inout.buildings[0];
    if (p_building == nullptr)
    {
        p_building = std::make_unique<scene_graph::Building>();
    }
    p_building->floors.insert_or_assign(floorId, std::move(p_floor_in));
}

} /* namespace core */
} /* namespace vs_graphs */
