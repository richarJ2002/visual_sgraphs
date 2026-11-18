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
 * @file            fuseImportedRoomAttributes.cc
 *
 * @brief           Implements fuseImportedRoomAttributes(), declared in
 *                  graph/merging/private_functions.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
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
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void fuseImportedRoomAttributes(
    SemanticsManager                      *p_semanticsManager_inout,
    unsigned long                          mapId_in,
    semantic::Room                        *p_importedRoom_inout,
    semantic::Room                        *p_retainedRoom_inout,
    const Eigen::Vector3d                 &importedCentroid_world_m_in,
    const std::vector<geometric::Plane *> &importedWalls_in,
    const std::vector<geometric::Plane *> &retainedWalls_in)
{
    std::vector<vs_graphs::core::semantic::Passage *> importedRoomPassages{};
    if (p_importedRoom_inout->getPassages(importedRoomPassages) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Passage *p_importedPassage :
         importedRoomPassages)
    {
        if (p_retainedRoom_inout->addPassageAssociation(p_importedPassage) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addPassageAssociation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::vector<vs_graphs::core::semantic::Passage *> mapAllPassages{};
    if (p_semanticsManager_inout->getMapPassagesInIdOrder(mapId_in,
                                                          mapAllPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its passages.",
            __func__,
            mapId_in);
    }
    for (semantic::Passage *p_passage : mapAllPassages)
    {
        if (p_passage != nullptr)
        {
            bool passageWasRoomReplaced{};
            if (p_passage->replaceProspectiveRoom(p_importedRoom_inout,
                                                  p_retainedRoom_inout,
                                                  passageWasRoomReplaced) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                passageWasRoomReplaced = false;
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: replaceProspectiveRoom rejected its "
                            "input; continuing as before.",
                            __func__);
            }
        }
    }

    const double retainedWeight =
        static_cast<double>(std::max<std::size_t>(retainedWalls_in.size(), 1U));

    const double importedWeight =
        static_cast<double>(std::max<std::size_t>(importedWalls_in.size(), 1U));

    Eigen::Vector3d bestRetainedRoomCentroid{};
    if (p_retainedRoom_inout->getCentroid(bestRetainedRoomCentroid) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d fusedCentroid_world_m =
        (retainedWeight * bestRetainedRoomCentroid +
         importedWeight * importedCentroid_world_m_in) /
        (retainedWeight + importedWeight);

    if (p_retainedRoom_inout->setCentroid(fusedCentroid_world_m) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    geometric::Plane *p_bestRetainedRoomGroundPlane = nullptr;
    if (p_retainedRoom_inout->getGroundPlane(p_bestRetainedRoomGroundPlane) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_bestRetainedRoomGroundPlane == nullptr)
    {
        geometric::Plane *p_importedRoomGroundPlane = nullptr;
        if (p_importedRoom_inout->getGroundPlane(p_importedRoomGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_retainedRoom_inout->setGroundPlane(p_importedRoomGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    bool bestRetainedRoomHasKnownLabel{};
    if (p_retainedRoom_inout->getHasKnownLabel(bestRetainedRoomHasKnownLabel) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHasKnownLabel returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool importedRoomHasKnownLabel2{};
    if ((!bestRetainedRoomHasKnownLabel) &&
        p_importedRoom_inout->getHasKnownLabel(importedRoomHasKnownLabel2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHasKnownLabel returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!bestRetainedRoomHasKnownLabel && importedRoomHasKnownLabel2)
    {
        if (p_retainedRoom_inout->setHasKnownLabel(true) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string importedRoomName{};
        if (p_importedRoom_inout->getName(importedRoomName) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_retainedRoom_inout->setName(importedRoomName) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Presence on either side proves the UAV has been there: a room fused
     * from a visited duplicate stays visited. */
    bool importedRoomHasPreviouslyVisited{};
    if (p_importedRoom_inout->hasPreviouslyVisited(
            importedRoomHasPreviouslyVisited) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasPreviouslyVisited returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (importedRoomHasPreviouslyVisited)
    {
        if (p_retainedRoom_inout->setPreviouslyVisited(true) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPreviouslyVisited returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
