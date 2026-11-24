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
 * @file            createMapPassage.cc
 *
 * @brief           Implements SemanticsManager::createMapPassage(), declared in
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

#include <iostream>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::createMapPassage(
    vs_graphs::core::geometric::Plane *p_doorPlane_in,
    vs_graphs::core::geometric::Plane *p_wallPlane_in,
    semantic::Passage                *&p_createdPassage_out,
    bool                               isOpenPassage_in,
    const Eigen::Vector3d             &passageCentroid_world_m_in)
{
    p_createdPassage_out = nullptr;

    /* ---------------------------------------------------------------------- *
     * VALIDATE REQUIRED INPUTS
     * ---------------------------------------------------------------------- */

    if (p_atlas == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: Atlas is null."
                  << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Every passage must be associated with a wall.
     *
     * Closed door:
     *     p_doorPlane_in != nullptr
     *     p_wallPlane_in != nullptr
     *
     * Open passage:
     *     p_doorPlane_in == nullptr
     *     p_wallPlane_in != nullptr
     */
    if (p_wallPlane_in == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane is null"
                  << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    bool wallPlaneIsBad{};
    if (p_wallPlane_in->isBad(wallPlaneIsBad) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wallPlaneIsBad)
    {
        int wallPlaneGetId{};
        if (p_wallPlane_in->getId(wallPlaneGetId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane"
                  << wallPlaneGetId << " is bad." << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Passage creation requires the supporting wall to have sufficient
     * observations. This prevents spurious passages on isolated wall segments
     * that have no evidence.
     */
    bool                 wallHasConfirmedRoom = false;
    types::SystemParams *p_params             = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const size_t minimumObservation =
        p_params->roomSeg.minimumWallObservationCount;
    std::size_t wallPlaneGetObservationCount{};
    if (p_wallPlane_in->getObservationCount(wallPlaneGetObservationCount) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (wallPlaneGetObservationCount >= minimumObservation)
    {
        wallHasConfirmedRoom = true;
    }

    if (!wallHasConfirmedRoom)
    {
        int wallPlaneGetId2{};
        if (p_wallPlane_in->getId(wallPlaneGetId2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t wallPlaneGetObservationCount2{};
        if (p_wallPlane_in->getObservationCount(
                wallPlaneGetObservationCount2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservationCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane "
                  << wallPlaneGetId2 << " has insufficient observations ("
                  << wallPlaneGetObservationCount2 << " < "
                  << minimumObservation << ")." << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract all passages */
    std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for the active map; "
            "continuing without its passages.",
            __func__);
    }

    double          passageWidth_m{};
    double          passageHeight_m{};
    Eigen::Vector3d passageCentroid_world_m;
    g2o::Plane3D    passageEquation_world;
    bool            isGeometryValid = false;
    if (computeNewPassageGeometry(p_doorPlane_in,
                                  p_wallPlane_in,
                                  passageCentroid_world_m_in,
                                  passageWidth_m,
                                  passageHeight_m,
                                  passageCentroid_world_m,
                                  passageEquation_world,
                                  isGeometryValid) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeNewPassageGeometry returned a failure status "
                     "although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!isGeometryValid)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    bool isCreationNeeded = false;
    if (refreshDuplicatePassage(allPassages,
                                p_doorPlane_in,
                                p_wallPlane_in,
                                isOpenPassage_in,
                                passageCentroid_world_m,
                                passageEquation_world,
                                isCreationNeeded) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: refreshDuplicatePassage returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    if (!isCreationNeeded)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (addNewMapPassage(p_doorPlane_in,
                         p_wallPlane_in,
                         isOpenPassage_in,
                         passageCentroid_world_m,
                         passageEquation_world,
                         passageWidth_m,
                         passageHeight_m,
                         p_createdPassage_out) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: addNewMapPassage returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
