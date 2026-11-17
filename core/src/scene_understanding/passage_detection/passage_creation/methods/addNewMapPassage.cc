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
 * @file            addNewMapPassage.cc
 *
 * @brief           Implements SemanticsManager::addNewMapPassage(), declared in
 *                  SemanticsManagerClass.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::addNewMapPassage(
    vs_graphs::core::geometric::Plane *p_doorPlane_in,
    vs_graphs::core::geometric::Plane *p_wallPlane_in,
    bool                               isOpenPassage_in,
    const Eigen::Vector3d             &passageCentroid_world_m_in,
    const g2o::Plane3D                &passageEquation_in,
    double                             passageWidth_m_in,
    double                             passageHeight_m_in,
    semantic::Passage                *&p_createdPassage_out)
{
    /* Passage identities belong to the mission, not to an active SLAM map. */
    int passageId{};
    if (p_atlas->reservePassageIdentity(passageId) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reservePassageIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Initialize passage object */
    std::unique_ptr<semantic::Passage> p_newPassageOwner =
        std::make_unique<semantic::Passage>();
    semantic::Passage *p_newMapPassage = p_newPassageOwner.get();

    /* Fill passage object */
    if (p_newMapPassage->setId(passageId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    map::Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPassage->setMap(p_atlasCurrentMap) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    if (p_newMapPassage->setCentroid(passageCentroid_world_m_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPassage->setGlobalEquation(passageEquation_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (p_newMapPassage->setWidth(passageWidth_m_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPassage->setHeight(passageHeight_m_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapPassage->setPassable(isOpenPassage_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_newMapPassage->addAssociateWall(p_wallPlane_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addAssociateWall returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * Both a detected closed door and a trajectory-detected open
     * passage represent a doorway.
     */
    if (p_newMapPassage->setPassageType(
            vs_graphs::core::semantic::Passage::PassageVariant::DOORWAY) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPassageType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_doorPlane_in != nullptr)
    {
        if (p_newMapPassage->setAssociateDoor(p_doorPlane_in) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* -------------------------------------------------------------- *
     * Insert into the Atlas
     * -------------------------------------------------------------- */

    std::ostringstream informationStream;

    informationStream << (isOpenPassage_in ? "open" : "blocked") << ", "
                      << std::fixed << std::setprecision(2) << passageWidth_m_in
                      << "x" << passageHeight_m_in << "m";

    int wallPlaneGetId4{};
    if (p_wallPlane_in->getId(wallPlaneGetId4) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[GeoSemHelper] Creating Passage#" << passageId
              << " associated with wall " << wallPlaneGetId4;

    if (p_doorPlane_in != nullptr)
    {
        int doorPlaneGetId2{};
        if (p_doorPlane_in->getId(doorPlaneGetId2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << " and door plane " << doorPlaneGetId2;
    }

    std::cout << " (" << informationStream.str()
              << "), centroid=" << passageCentroid_world_m_in.transpose() << "."
              << std::endl;

    /* Move the passage identity allocator past the id, then add the
     * passage to the map's scene graph. */
    if (p_atlas->observePassageIdentity(passageId) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: observePassageIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long atlasCurrentMapId{};
    if ((p_atlasCurrentMap->getId(atlasCurrentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (addPassage(atlasCurrentMapId, std::move(p_newPassageOwner)) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/addPassage found no scene graph for the "
                     "passage's map; the passage was not added.",
                     __func__);
    }

    p_createdPassage_out = p_newMapPassage;

    std::vector<vs_graphs::core::semantic::Passage *> atlasAllPassages{};
    if (getAllPassagesInIdOrder(atlasAllPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for the active map; "
            "continuing without its passages.",
            __func__);
    }
    std::cout << "[GeoSemHelper] Atlas now contains " << atlasAllPassages.size()
              << " passages." << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
