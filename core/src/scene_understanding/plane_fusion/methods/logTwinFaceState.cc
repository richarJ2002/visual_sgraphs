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
 * @file            logTwinFaceState.cc
 *
 * @brief           Implements SemanticsManager::logTwinFaceState(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/plane_fusion/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::logTwinFaceState(
    long long                   recordMapId_in,
    geometric::Plane           *p_firstWall_in,
    geometric::Plane           *p_secondWall_in,
    const char                 *pairState_in,
    utils::utils::TwinFaceCheck decidingCheck_in,
    double                      decidingValue_in,
    double                      faceSeparation_m_in)
{
    int firstWallId{};
    if (p_firstWall_in->getId(firstWallId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int secondWallId{};
    if (p_secondWall_in->getId(secondWallId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const std::pair<int, int> pairKey{std::min(firstWallId, secondWallId),
                                      std::max(firstWallId, secondWallId)};
    const std::string         stateKey = std::string(pairState_in) + ":" +
                                 getTwinFaceCheckName(decidingCheck_in);
    std::string &loggedState = loggedTwinFaceStates[pairKey];
    if (loggedState == stateKey)
    {
        return;
    }
    loggedState = stateKey;
    std::cout << "SG_PIPELINE {\"event\":\"twin_face\",\"map_id\":"
              << recordMapId_in
              << ",\"semantic_cycle\":" << pipelineSemanticCycle
              << ",\"first_wall_id\":" << pairKey.first
              << ",\"second_wall_id\":" << pairKey.second << ",\"state\":\""
              << pairState_in << "\",\"check\":\""
              << getTwinFaceCheckName(decidingCheck_in)
              << "\",\"value\":" << decidingValue_in
              << ",\"thickness_m\":" << faceSeparation_m_in << "}" << std::endl;
}

} /* namespace core */
} /* namespace vs_graphs */
