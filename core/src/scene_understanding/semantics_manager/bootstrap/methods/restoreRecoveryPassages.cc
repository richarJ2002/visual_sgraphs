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
 * @file            restoreRecoveryPassages.cc
 *
 * @brief           Implements SemanticsManager::restoreRecoveryPassages(),
 *                  declared in
semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

std::size_t SemanticsManager::restoreRecoveryPassages(
    map::Map                            *p_activeMap_inout,
    unsigned long                        mapId_in,
    const semantic::RoomContextSnapshot &recoveryContext_in,
    semantic::Room                      *p_room_inout)
{
    std::size_t restoredPassageCount = 0U;
    for (const semantic::PassageContext &passageContext :
         recoveryContext_in.passageContexts)
    {
        /* A passage that already exists in the new map is reused. */
        semantic::Passage *p_passage = nullptr;
        if (getMapPassageById(mapId_in, passageContext.id, p_passage) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPassageById returned a failure status; "
                         "continuing as before.",
                         __func__);
        }
        if (p_passage == nullptr)
        {
            std::unique_ptr<semantic::Passage> p_newPassage =
                createRecoveryPassage(p_atlas,
                                      p_activeMap_inout,
                                      passageContext);
            p_passage = p_newPassage.get();
            if (addPassage(mapId_in, std::move(p_newPassage)) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addPassage returned a failure status; "
                             "continuing as before.",
                             __func__);
            }
        }
        linkRecoveryPassage(passageContext,
                            recoveryContext_in.roomId,
                            p_passage,
                            p_room_inout);
        restoredPassageCount++;
    }
    return restoredPassageCount;
}

} /* namespace core */
} /* namespace vs_graphs */
