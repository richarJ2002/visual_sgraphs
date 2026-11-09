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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            createMapFloor.cc
 *
 * @brief           Implements GeoSemHelpers::createMapFloor(), declared in
 *                  GeoSemHelpers.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs::core::utils::geo_sem_helpers
{

GeoSemHelpersStatus GeoSemHelpers::createMapFloor(atlas::Atlas *p_atlas_inout)
{
    map::Map *p_currentMap = nullptr;
    if (p_atlas_inout->getCurrentMap(p_currentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_currentMap == nullptr)
    {
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Create a new floor object */
    Eigen::Vector3d                  centroid = Eigen::Vector3d::Zero();
    std::unique_ptr<semantic::Floor> p_newFloorOwner =
        std::make_unique<semantic::Floor>();
    semantic::Floor *p_newMapFloor = p_newFloorOwner.get();

    /* Variables */
    int floorId{};
    if (p_atlas_inout->reserveFloorIdentity(floorId) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reserveFloorIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Fill the floor entity */
    if (p_newMapFloor->setOpId(-1) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setOpId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapFloor->setOpIdG(-1) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setOpIdG returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapFloor->setId(floorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapFloor->setCentroid(centroid) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapFloor->setMap(p_currentMap) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapFloor->setName("Floor#" + std::to_string(floorId)) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setName returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Move the floor identity allocator past the id, then add the floor to
     * the map's scene graph, which may give it a free id. */
    if (p_atlas_inout->observeFloorIdentity(floorId) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: observeFloorIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    SemanticsManager *p_semanticsManager = nullptr;
    unsigned long     currentMapId{};
    if ((p_atlas_inout->getSemanticsManager(p_semanticsManager) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_currentMap->getId(currentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_semanticsManager->addFloor(currentMapId,
                                      std::move(p_newFloorOwner)) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getSemanticsManager/getId/addFloor found no scene graph for "
            "the floor's map; the floor was not added.",
            __func__);
    }

    int newMapFloorId{};
    if (p_newMapFloor->getId(newMapFloorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[GeoSemHelper] Creating Floor#" << newMapFloorId << " ..."
              << std::endl;

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} /* namespace vs_graphs::core::utils::geo_sem_helpers */
