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
 * @file            isSeedFusableIntoRoom.cc
 *
 * @brief           Implements SemanticsManager::isSeedFusableIntoRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::isSeedFusableIntoRoom(
    semantic::Room        *p_retainedRoom_in,
    semantic::Room        *p_seedRoom_in,
    semantic::Passage     *p_seedPassage_in,
    map::Map              *p_currentMap_in,
    const Eigen::Vector3d &retainedCentroid_world_m_in,
    bool                  &isFusable_out)
{
    isFusable_out = false;

    std::vector<geometric::Plane *> duplicateSeedWalls{};
    if (p_seedRoom_in->getWalls(duplicateSeedWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!duplicateSeedWalls.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<geometric::Plane *> retainedWalls{};
    if (p_retainedRoom_in->getWalls(retainedWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d upDirection_world = Eigen::Vector3d::Zero();
    bool            isUpFromImuGravity{};
    if (p_currentMap_in->getUpDirection(upDirection_world,
                                        isUpFromImuGravity) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpDirection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    RoomWallFootprint retainedFootprint{};
    bool              hasRetainedFootprint{};
    if (computeRoomWallFootprint(retainedWalls,
                                 upDirection_world,
                                 retainedFootprint,
                                 hasRetainedFootprint) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeRoomWallFootprint returned a failure "
                     "status although it cannot fail; continuing as "
                     "before.",
                     __func__);
    }
    Eigen::Vector3d seedCentroid_world_m{};
    if (p_seedRoom_in->getCentroid(seedCentroid_world_m) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const double footprintTolerance_m =
        0.5 * static_cast<double>(p_sysParams->optimization.minimumThickness_m);
    const double seedAlongFirst_m =
        seedCentroid_world_m.dot(retainedFootprint.firstAxis_world);
    const double seedAlongSecond_m =
        seedCentroid_world_m.dot(retainedFootprint.secondAxis_world);
    const bool isSeedInsideFootprint =
        hasRetainedFootprint && seedCentroid_world_m.allFinite() &&
        seedAlongFirst_m >=
            retainedFootprint.minimumFirst_m - footprintTolerance_m &&
        seedAlongFirst_m <=
            retainedFootprint.maximumFirst_m + footprintTolerance_m &&
        seedAlongSecond_m >=
            retainedFootprint.minimumSecond_m - footprintTolerance_m &&
        seedAlongSecond_m <=
            retainedFootprint.maximumSecond_m + footprintTolerance_m;
    if (!isSeedInsideFootprint)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D seedPassageGlobalEquation{};
    if (p_seedPassage_in->getGlobalEquation(seedPassageGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector4d seedPassageEquation_world =
        seedPassageGlobalEquation.coeffs();
    const double seedSide_m =
        seedPassageEquation_world.head<3>().dot(seedCentroid_world_m) +
        seedPassageEquation_world(3);
    const double retainedSide_m =
        seedPassageEquation_world.head<3>().dot(retainedCentroid_world_m_in) +
        seedPassageEquation_world(3);
    /* Only a strictly same-side pair fuses; an unusable plane or
     * centre (NaN or zero product) proves no side, so the seed keeps
     * waiting. */
    if (!(seedSide_m * retainedSide_m > 0.0))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    isFusable_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
