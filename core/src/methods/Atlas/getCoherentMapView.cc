/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            getCoherentMapView.cc
 *
 * @brief           Implements Atlas::GetCoherentMapView(), declared in
 *                  Atlas.h.
 */

/* Matching Declaration Include */
#include "Atlas.h"

/* C++ Standard Library Includes */
#include <algorithm>
#include <mutex>
#include <optional>
#include <vector>

/* External Library Includes */
/* None */

/* Module Includes */
/* None */

/* Object Includes */
#include "AtlasCurrentMapStatus.h"
#include "Map.h"

namespace vs_graphs
{
namespace core
{

std::vector<Map *> Atlas::getCoherentMapView(
    std::optional<long unsigned int> &currentMapId_out,
    AtlasCurrentMapStatus            &currentMapStatus_out)
{
    std::unique_lock<std::mutex> atlasLock(mMutexAtlas);

    std::vector<Map *> activeMaps(maps.begin(), maps.end());
    std::sort(activeMaps.begin(),
              activeMaps.end(),
              [](Map *p_lhs_in, Map *p_rhs_in)
              { return p_lhs_in->getId() < p_rhs_in->getId(); });

    currentMapId_out.reset();
    currentMapStatus_out = AtlasCurrentMapStatus::NO_CURRENT_MAP;
    if (p_activeMap != nullptr)
    {
        currentMapId_out = p_activeMap->getId();
        const bool isCurrentMapActive =
            std::find(activeMaps.begin(), activeMaps.end(), p_activeMap) !=
            activeMaps.end();
        /* SetMapBad(mpCurrentMap) erases the map from mspMaps and marks it
         * bad without clearing mpCurrentMap; a later ChangeMap() call is
         * what eventually installs a replacement. Report that reachable
         * state truthfully instead of assuming the current map is always
         * active. */
        currentMapStatus_out =
            isCurrentMapActive ? AtlasCurrentMapStatus::CURRENT_MAP_ACTIVE
                               : AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE;
    }

    return activeMaps;
}

} // namespace core
} // namespace vs_graphs
