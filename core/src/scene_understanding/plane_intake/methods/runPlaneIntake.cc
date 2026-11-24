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
 * @file            runPlaneIntake.cc
 *
 * @brief           Implements SemanticsManager::runPlaneIntake(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runPlaneIntake(
    std::vector<geometric::Plane *> &intakePlanes_out)
{
    /* Camera centre of the newest key frame, read by the later stages */
    if (refreshCurrentCameraCenter() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: refreshCurrentCameraCenter returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * Planes have no change stamp and bundle adjustment may move any of them
     * between two cycles, so every plane of the active map counts as created
     * or updated since the last cycle. Id order makes every visit
     * independent of addresses.
     */
    intakePlanes_out.clear();
    if (p_atlas->getAllPlanes(intakePlanes_out) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(intakePlanes_out.begin(),
              intakePlanes_out.end(),
              semantic::isEntityIdLess<geometric::Plane>);

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
