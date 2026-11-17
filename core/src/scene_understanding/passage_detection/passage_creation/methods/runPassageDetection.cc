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
 * @file            runPassageDetection.cc
 *
 * @brief           Implements SemanticsManager::runPassageDetection(), declared
 * in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runPassageDetection(void)
{
    /*!
     * Passage geometry is detected and updated before any wall-to-room
     * admission of the cycle, so admitWallToRoom()'s far-side-passage
     * backstop and the free-space clustering of the room stage can route a
     * wall to the correct side of a passage the moment the passage becomes
     * observable, instead of only on the next cycle. These steps only read
     * Plane/Passage data (no Room dependency). The passage steps that need
     * current room membership (traversal evidence, room association, wall
     * detachment, prospective-room cleanup) run later in run(), after the
     * wall admission.
     */
    if (!p_sysParams->semSeg.enablePassageDetection)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (detectPassages(p_atlas) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: detectPassages returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (updatePassages(p_atlas) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updatePassages returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (mergeOverlappingPassages() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mergeOverlappingPassages returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
