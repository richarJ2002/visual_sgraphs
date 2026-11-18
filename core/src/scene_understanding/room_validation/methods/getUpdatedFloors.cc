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
 * @file            getUpdatedFloors.cc
 *
 * @brief           Implements SemanticsManager::getUpdatedFloors(), declared in
 *                  SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getUpdatedFloors(void)
{
    PROFILE_FUNCTION();
    map::Map *p_currentMap = nullptr;
    if (p_atlas->getCurrentMap(p_currentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The current implementation supports one floor */
    std::vector<semantic::Floor *> currentMapAllFloors{};
    unsigned long                  currentMapId{};
    if (p_currentMap->getId(currentMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (getMapFloorsInIdOrder(currentMapId, currentMapAllFloors) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapFloorsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    if (currentMapAllFloors.empty())
    {
        /* An outdoor start has no storey until a room exists to put in it. */
        std::vector<semantic::Room *> detectedRooms{};
        if (p_sysParams->semanticsManager.startEnvironment ==
            scene_graph::StartEnvironment::START_ENVIRONMENT_OUTDOOR)
        {
            if (getMapDetectedRoomsInIdOrder(currentMapId, detectedRooms) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapDetectedRoomsInIdOrder returned a "
                             "failure status; continuing as before.",
                             __func__);
            }
            if (detectedRooms.empty())
            {
                return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
            }
        }

        /* A reset can reach this update before the new map has a usable
         * camera pose. The new floor takes a fresh identity and carries no
         * prior-map geometry. */
        if (utils::geo_sem_helpers::GeoSemHelpers::createMapFloor(p_atlas) !=
            utils::geo_sem_helpers::GeoSemHelpersStatus::
                GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Collapse legacy/merge duplicates before writing any hierarchy edge. */
    std::vector<vs_graphs::core::semantic::Floor *> floors{};
    if (getMapFloorsInIdOrder(currentMapId, floors) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapFloorsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    semantic::Floor *p_keeperFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(floors, p_keeperFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keeperFloor == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    for (semantic::Floor *p_duplicateFloor : floors)
    {
        if (p_duplicateFloor == nullptr || p_duplicateFloor == p_keeperFloor)
        {
            continue;
        }
        if (p_duplicateFloor->setRooms({}) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (eraseFloor(currentMapId, p_duplicateFloor) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseFloor returned a failure status; continuing "
                         "as before.",
                         __func__);
        }
    }

    /* Refresh the comparable identity from the strongest observed ground. */
    geometric::Plane *p_groundPlane = nullptr;
    if (p_currentMap->getBiggestGroundPlane(p_groundPlane) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundIdentityUpdated = false;
    if (p_groundPlane != nullptr)
    {
        geometric::PlaneGeometryMetadataSnapshot groundGeometry{};
        if (p_groundPlane->getGeometryMetadataSnapshot(groundGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getGeometryMetadataSnapshot returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
        const double groundNormalNorm =
            groundGeometry.planeEquation_world.head<3>().norm();
        if (groundGeometry.cloudGeneration ==
                groundGeometry.successfulRefitGeneration &&
            groundGeometry.finiteSupportCount > 0U &&
            groundGeometry.planeEquation_world.allFinite() &&
            std::isfinite(groundNormalNorm) &&
            std::abs(groundNormalNorm - 1.0) <= 1e-3)
        {
            groundIdentityUpdated =
                (p_keeperFloor->setPlaneIdentity(
                     groundGeometry.planeEquation_world,
                     groundGeometry.finiteSupportCount,
                     groundGeometry.observationCount) ==
                 semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
        }
    }
    if (!groundIdentityUpdated)
    {
        if (p_keeperFloor->clearPlaneIdentity() !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Extract only CONFIRMED rooms (detected map rooms) for floor centroid.
     * Exclude candidate/prospective rooms from mspMarkerBasedRooms. */
    std::vector<vs_graphs::core::semantic::Room *> confirmedRooms{};
    if (getMapDetectedRoomsInIdOrder(currentMapId, confirmedRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapDetectedRoomsInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }

    /* Remove all invalid rooms */
    confirmedRooms.erase(
        std::remove_if(
            confirmedRooms.begin(),
            confirmedRooms.end(),
            [](vs_graphs::core::semantic::Room *p_room)
            {
                bool roomIsBad{};
                if (!(p_room == nullptr) &&
                    p_room->isBad(roomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return p_room == nullptr || roomIsBad;
            }),
        confirmedRooms.end());

    /* Keep hierarchy backlinks current even when no valid rooms remain. */
    if (confirmedRooms.empty())
    {
        if (p_keeperFloor->setRooms({}) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Create list of centroids for each confirmed room */
    std::vector<Eigen::Vector3d> roomCentroids;
    roomCentroids.reserve(confirmedRooms.size());

    /* Extract centroids from each confirmed room */
    for (vs_graphs::core::semantic::Room *p_room : confirmedRooms)
    {
        Eigen::Vector3d roomCentroid{};
        if (p_room->getCentroid(roomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomCentroids.push_back(roomCentroid);
    }

    /* Find the floor centroid from the confirmed room centroids */
    Eigen::Vector3d floorCentroid{};
    if (utils::utils::Utils::computeCentroidFromPoints(roomCentroids,
                                                       floorCentroid) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeCentroidFromPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (p_keeperFloor->setRooms(confirmedRooms) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keeperFloor->setCentroid(floorCentroid) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
