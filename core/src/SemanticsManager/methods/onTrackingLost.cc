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

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::onTrackingLost(void)
{
    std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
    if (!isTrackingLossEpisodeActive)
    {
        int atlasGetCurrentSemanticRoomIdentity{};
        if (!(currentRoomId >= 0 || p_atlas == nullptr) &&
            p_atlas->getCurrentSemanticRoomIdentity(
                atlasGetCurrentSemanticRoomIdentity) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getCurrentSemanticRoomIdentity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        lastKnownRoomId = currentRoomId >= 0 || p_atlas == nullptr
                              ? currentRoomId
                              : atlasGetCurrentSemanticRoomIdentity;
        if (p_atlas != nullptr && lastKnownRoomId >= 0)
        {
            if (p_atlas->setCurrentSemanticRoomIdentity(lastKnownRoomId) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setCurrentSemanticRoomIdentity returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        isTrackingLostPending       = true;
        isTrackingLossEpisodeActive = true;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
