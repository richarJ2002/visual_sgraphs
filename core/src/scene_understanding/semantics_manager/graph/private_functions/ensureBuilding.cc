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
 * @file            ensureBuilding.cc
 *
 * @brief           Implements ensureBuilding(), declared in
 *                  graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <memory>

namespace vs_graphs
{
namespace core
{

scene_graph::Building &ensureBuilding(scene_graph::SceneGraph &sceneGraph_inout,
                                      int                      buildingId_in)
{
    std::unique_ptr<scene_graph::Building> &p_building =
        sceneGraph_inout.buildings[buildingId_in];
    if (p_building == nullptr)
    {
        p_building = std::make_unique<scene_graph::Building>();
    }
    return *p_building;
}

} /* namespace core */
} /* namespace vs_graphs */
