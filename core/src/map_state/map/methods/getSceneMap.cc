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
 * @file            getSceneMap.cc
 *
 * @brief           Implements Map::getSceneMap(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::getSceneMap(scene_graph::SceneMap *&p_sceneMap_out)
{
    p_sceneMap_out = &sceneMap;

    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
