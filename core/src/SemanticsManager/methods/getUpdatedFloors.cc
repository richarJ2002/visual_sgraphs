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

#include "GeoSemHelpers.h"
#include "GeoSemHelpersStatus.h"
#include "SemanticsManager.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::getUpdatedFloors(void)
{
    Map *p_currentMap = p_atlas->getCurrentMap();
    if (p_currentMap == nullptr)
    {
        return;
    }

    /* The current implementation supports one floor */
    std::vector<semantic::Floor *> currentMapAllFloors{};
    if (p_currentMap->getAllFloors(currentMapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentMapAllFloors.empty())
    {
        /* A reset can reach this update before the new map has a usable
         * camera pose. Recover the semantic floor identity from the last
         * current-room hierarchy instead of consuming a new mission identity.
         * The newly allocated Floor object deliberately carries no prior-map
         * geometry; only its semantic ID crosses the reset boundary. */
        std::optional<int> recoveredFloorId;
        const int          currentSemanticRoomId =
            p_atlas->getCurrentSemanticRoomIdentity();
        if (currentSemanticRoomId >= 0)
        {
            const std::optional<semantic::RoomContextSnapshot> recoveryContext =
                p_atlas->copyLatestRoomContext(currentSemanticRoomId);
            if (recoveryContext.has_value() && recoveryContext->floorId >= 0)
            {
                recoveredFloorId = recoveryContext->floorId;
            }
        }
        if (GeoSemHelpers::createMapFloor(p_atlas, recoveredFloorId) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Collapse legacy/merge duplicates before writing any hierarchy edge. */
    std::vector<vs_graphs::core::semantic::Floor *> floors{};
    if (p_currentMap->getAllFloors(floors) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
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
        return;
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
        if (p_currentMap->eraseMapFloor(p_duplicateFloor) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Refresh the comparable identity from the strongest observed ground. */
    geometric::Plane *p_groundPlane = nullptr;
    if (p_currentMap->getBiggestGroundPlane(p_groundPlane) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundIdentityUpdated = false;
    if (p_groundPlane != nullptr)
    {
        geometric::Plane::GeometrySnapshot groundGeometry{};
        if (p_groundPlane->getGeometrySnapshot(groundGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double groundNormalNorm =
            groundGeometry.equation_World.head<3>().norm();
        if (groundGeometry.cloudGeneration ==
                groundGeometry.successfulRefitGeneration &&
            groundGeometry.finiteSupportCount > 0U &&
            groundGeometry.equation_World.allFinite() &&
            std::isfinite(groundNormalNorm) &&
            std::abs(groundNormalNorm - 1.0) <= 1e-3)
        {
            groundIdentityUpdated =
                (p_keeperFloor->setPlaneIdentity(
                     groundGeometry.equation_World,
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
    std::vector<vs_graphs::core::semantic::Room *> confirmedRooms =
        p_atlas->getAllDetectedMapRooms();

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
        return;
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
}

} // namespace core
} // namespace vs_graphs
