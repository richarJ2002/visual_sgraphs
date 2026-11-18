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
 * @file            runRoomAssociation.cc
 *
 * @brief           Implements SemanticsManager::runRoomAssociation(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runRoomAssociation(void)
{
    /*!
     * One association pass per cycle. The passage-room steps run first, on
     * the room walls of the previous cycle and this cycle's passages (stage
     * 4): they link passages to their side rooms, create or retire far-side
     * prospective rooms and detach walls beyond confirmed passages. The wall
     * association then runs once on that state, so a detached wall or a new
     * prospective room is settled in the same cycle, and every unowned wall
     * of the map is offered to the current room. Unique wall ownership is
     * enforced last.
     */
    if (p_sysParams->semSeg.enablePassageDetection)
    {
        if (utils::utils::Utils::reAssociatePassages(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociatePassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (associatePassagesToRooms() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: associatePassagesToRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (detachWallsBeyondConfirmedPassages() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detachWallsBeyondConfirmedPassages returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }

        /* Continuous rule-invariant sweep (not just at admission time): see
         * enforcePassageSideInvariant()'s own comment. Room<->passage
         * association is current as of the two calls just above. */
        if (enforcePassageSideInvariant() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: enforcePassageSideInvariant returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Topology rules of every passage, after its side invariant. */
        if (enforcePassageTopologyInvariant() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: enforcePassageTopologyInvariant returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* A passage reference is the stable far-side handle; only orphaned or
         * invalid prospective rooms are retired. */
        if (retireOrphanedProspectiveRooms() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: retireOrphanedProspectiveRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Walls to rooms from free-space clusters split at passages; the
     * preferred association method. */
    if (p_sysParams->roomSeg.method ==
        types::SystemParams::RoomSeg::Method::FREE_SPACE)
    {
        if (associateWallsToRoomsFromFreeSpaceClusters() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: associateWallsToRoomsFromFreeSpaceClusters returned a "
                "failure status although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Every wall the free-space pass did not capture receives an existing
     * room or a new provisional structural element. */
    if (associateAllWallsToRooms() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: associateAllWallsToRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Consolidate only redundant single-wall provisional structures. */
    if (utils::utils::Utils::reAssociateRooms(p_atlas) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reAssociateRooms returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* A wall surface is owned by exactly one room in every configuration. */
    if (enforceUniqueWallOwnership() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: enforceUniqueWallOwnership returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
