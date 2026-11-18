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
 * @file            runRoomValidation.cc
 *
 * @brief           Implements SemanticsManager::runRoomValidation(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runRoomValidation(void)
{
    /* Validate room geometry without delaying independent passage data. */
    if (validateRoomBoundaries() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: validateRoomBoundaries returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Retire only stale, weak wall hypotheses left unused by the graph. */
    if (suppressUndefendedWalls() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: suppressUndefendedWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Recompute room centroids as mean of associated wall centroids.
     * In FREE_SPACE mode the skeleton-cluster centroid set during room
     * detection is the authoritative room centre; overwriting it with the
     * wall-centroid mean drifts rooms away from the cluster each cycle,
     * which breaks the centroid-keyed room matching in associateRooms()
     * and spawns a duplicate room every run. */
    if (p_sysParams->roomSeg.method !=
        types::SystemParams::RoomSeg::Method::FREE_SPACE)
    {
        if (recomputeRoomCentroidsFromWalls() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: recomputeRoomCentroidsFromWalls returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Associate every valid room/SE with the floor */
    if (getUpdatedFloors() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpdatedFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Re-point any room whose own ground plane disagrees with the
     * just-refreshed canonical Floor identity. */
    if (reconcileRoomGroundPlanes() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reconcileRoomGroundPlanes returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
