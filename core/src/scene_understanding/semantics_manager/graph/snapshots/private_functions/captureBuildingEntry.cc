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
 * @file            captureBuildingEntry.cc
 *
 * @brief           Implements captureBuildingEntry(), declared in
 *                  graph/snapshots/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/snapshots/private_functions/private_functions.h"

#include <map>
#include <memory>
#include <utility>

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

void captureBuildingEntry(int                          buildingId_in,
                          const scene_graph::Building &building_in,
                          SceneGraphBuildingEntry     &buildingEntry_out)
{
    buildingEntry_out    = SceneGraphBuildingEntry{};
    buildingEntry_out.id = buildingId_in;

    /* The floors are keyed by id, so iteration is already in id order. */
    for (const std::pair<const int, std::unique_ptr<semantic::Floor>>
             &floorEntry : building_in.floors)
    {
        if (isFloorLive(floorEntry.second.get()))
        {
            buildingEntry_out.floorIds.push_back(floorEntry.first);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
