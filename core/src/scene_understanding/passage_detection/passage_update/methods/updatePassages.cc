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
 * @file            updatePassages.cc
 *
 * @brief           Implements SemanticsManager::updatePassages(), declared in
 *                  SemanticsManagerClass.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::updatePassages(atlas::Atlas *p_atlas_in)
{
    PROFILE_FUNCTION();
    /* Get the ground plane */
    vs_graphs::core::geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas_in->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane == nullptr)
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;

    /*!
     * Get all passages and update their global pose to be consistent with the
     * ground plane
     */
    std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }

    for (semantic::Passage *const &passage : allPassages)
    {
        bool passageIsBad{};
        if (!(passage == nullptr) &&
            passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (passage == nullptr || passageIsBad)
        {
            continue;
        }

        /*!
         * Updating the dimensions of the passage based on the associated door
         * plane
         */
        vs_graphs::core::geometric::Plane *p_doorPlane = nullptr;
        if (passage->getAssociateDoor(p_doorPlane) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Blocked passages (closed doors) should be aligned with the ground
         * plane normal
         */
        bool passageIsPassable{};
        if (passage->isPassable(passageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!passageIsPassable)
        {
            if (refreshClosedPassageFromDoor(passage, p_doorPlane) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: refreshClosedPassageFromDoor returned a "
                             "failure status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
        }
        else
        {
            if (reanchorOpenPassageToWalls(passage, p_groundPlane) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: reanchorOpenPassageToWalls returned a "
                             "failure status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
