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

#include "Semantic/Passage.h"
#include "Semantic/PassageStatus.h"
#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::setDoorways(vs_graphs::core::semantic::Passage *p_passage_in)
{
    if (p_passage_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    const bool isAlreadyPresent = std::any_of(
        doorways.begin(),
        doorways.end(),
        [p_passage_in](vs_graphs::core::semantic::Passage *p_existingPassage)
        {
            int existingPassageId{};
            if ((p_existingPassage != nullptr) &&
                p_existingPassage->getId(existingPassageId) !=
                    PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int passage_inId{};
            if ((p_existingPassage != nullptr) &&
                p_passage_in->getId(passage_inId) !=
                    PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            return p_existingPassage != nullptr &&
                   existingPassageId == passage_inId;
        });

    if (!isAlreadyPresent)
    {
        doorways.push_back(p_passage_in);
    }

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
