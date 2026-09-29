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
#include <rclcpp/logging.hpp>
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

AtlasStatus Atlas::getCoherentMapView(
    std::optional<long unsigned int> &currentMapId_inout,
    AtlasCurrentMapStatus            &currentMapStatus_out,
    std::vector<Map *>               &coherentMapView_out)
{
    std::unique_lock<std::mutex> atlasLock(atlasMutex);

    std::vector<Map *> activeMaps(maps.begin(), maps.end());
    std::sort(
        activeMaps.begin(),
        activeMaps.end(),
        [](Map *p_lhs_in, Map *p_rhs_in)
        {
            unsigned long lhsId{};
            if (p_lhs_in->getId(lhsId) != MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long rhsId{};
            if (p_rhs_in->getId(rhsId) != MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            return lhsId < rhsId;
        });

    currentMapId_inout.reset();
    currentMapStatus_out = AtlasCurrentMapStatus::NO_CURRENT_MAP;
    if (p_activeMap != nullptr)
    {
        unsigned long activeMapId{};
        if (p_activeMap->getId(activeMapId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        currentMapId_inout = activeMapId;
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

    coherentMapView_out = activeMaps;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
