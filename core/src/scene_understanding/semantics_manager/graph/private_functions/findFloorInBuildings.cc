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
 * @file            findFloorInBuildings.cc
 *
 * @brief           Implements findFloorInBuildings(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <map>
#include <memory>

namespace vs_graphs
{
namespace core
{

semantic::Floor *findFloorInBuildings(const scene_graph::SceneMap &sceneMap_in,
                                      int                          floorId_in)
{
    for (const std::pair<const int, std::unique_ptr<scene_graph::Building>>
             &buildingEntry : sceneMap_in.buildings)
    {
        std::map<int, std::unique_ptr<semantic::Floor>>::const_iterator
            floorIterator = buildingEntry.second->floors.find(floorId_in);
        if (floorIterator != buildingEntry.second->floors.end())
        {
            return floorIterator->second.get();
        }
    }
    return nullptr;
}

} /* namespace core */
} /* namespace vs_graphs */
