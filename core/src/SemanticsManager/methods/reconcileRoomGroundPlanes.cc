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
 * @file            reconcileRoomGroundPlanes.cc
 *
 * @brief           Implements SemanticsManager::reconcileRoomGroundPlanes(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::reconcileRoomGroundPlanes(void)
{
    Map *p_currentMap = nullptr;
    if (p_atlas->getCurrentMap(p_currentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
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

    std::vector<vs_graphs::core::semantic::Floor *> floors{};
    if (p_currentMap->getAllFloors(floors) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Floor *p_canonicalFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(floors, p_canonicalFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool canonicalFloorHasPlaneIdentity{};
    if (!(p_canonicalFloor == nullptr) &&
        p_canonicalFloor->hasPlaneIdentity(canonicalFloorHasPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_canonicalFloor == nullptr || !canonicalFloorHasPlaneIdentity)
    {
        /* No canonical identity to reconcile against yet. */
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::optional<semantic::Floor::PlaneIdentity> canonicalIdentity{};
    if (p_canonicalFloor->getPlaneIdentity(canonicalIdentity) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!canonicalIdentity.has_value())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    geometric::Plane *p_canonicalGroundPlane = nullptr;
    if (p_currentMap->getBiggestGroundPlane(p_canonicalGroundPlane) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<semantic::Room *> atlasAllDetectedMapRooms{};
    if (p_atlas->getAllDetectedMapRooms(atlasAllDetectedMapRooms) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_room : atlasAllDetectedMapRooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        geometric::Plane *p_roomGroundPlane = nullptr;
        if (p_room->getGroundPlane(p_roomGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGroundPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool roomGroundPlaneIsBad{};
        if (!(p_roomGroundPlane == nullptr) &&
            p_roomGroundPlane->isBad(roomGroundPlaneIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_roomGroundPlane == nullptr || roomGroundPlaneIsBad ||
            p_roomGroundPlane == p_canonicalGroundPlane)
        {
            /* Nothing to reconcile: no ground plane yet, or already the
             * canonical one. */
            continue;
        }

        geometric::Plane::GeometrySnapshot roomGroundGeometry{};
        if (p_roomGroundPlane->getGeometrySnapshot(roomGroundGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double roomGroundNormalNorm =
            roomGroundGeometry.planeEquation_world.head<3>().norm();
        if (roomGroundGeometry.cloudGeneration !=
                roomGroundGeometry.successfulRefitGeneration ||
            roomGroundGeometry.finiteSupportCount == 0U ||
            !roomGroundGeometry.planeEquation_world.allFinite() ||
            !std::isfinite(roomGroundNormalNorm) ||
            std::abs(roomGroundNormalNorm - 1.0) > 1e-3)
        {
            /* Room's ground plane geometry isn't settled yet -- nothing
             * reliable to compare. */
            continue;
        }

        const semantic::Floor::PlaneIdentity roomIdentity{
            roomGroundGeometry.planeEquation_world,
            roomGroundGeometry.finiteSupportCount,
            roomGroundGeometry.observationCount};

        double normalAngle_deg = 0.0;
        double offset_m        = 0.0;
        bool   isMatch{};
        if (semantic::Floor::planeIdentitiesMatch(
                canonicalIdentity.value(),
                roomIdentity,
                semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
                semantic::Floor::kMergeMaxPlaneOffset_m,
                normalAngle_deg,
                offset_m,
                isMatch) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: planeIdentitiesMatch returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (isMatch)
        {
            /* Within tolerance -- nothing to reconcile. */
            continue;
        }

        /* A real flatness disagreement. Re-point the less-observed side to
         * the canonical plane -- a pure pointer rewire, never a geometry
         * mutation, so it can never fight a plane's own cloud refit. */
        if (roomIdentity.observationCount <=
                canonicalIdentity->observationCount &&
            p_canonicalGroundPlane != nullptr)
        {
            if (p_room->setGroundPlane(p_canonicalGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGroundPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int roomId{};
            if (p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int canonicalFloorId{};
            if (p_canonicalFloor->getId(canonicalFloorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] semantic::Room#" << roomId
                      << "'s ground plane disagreed with semantic::Floor#"
                      << canonicalFloorId << "'s canonical level (normal "
                      << normalAngle_deg << " deg, offset " << offset_m
                      << " m) -- re-pointed to the canonical plane."
                      << std::endl;
        }
        else
        {
            int roomId2{};
            if (p_room->getId(roomId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int canonicalFloorId2{};
            if (p_canonicalFloor->getId(canonicalFloorId2) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout
                << "[SemMgr] semantic::Room#" << roomId2
                << "'s ground plane is more observed than semantic::Floor#"
                << canonicalFloorId2 << "'s current canonical level (normal "
                << normalAngle_deg << " deg, offset " << offset_m
                << " m) -- left as-is; the floor will re-select its "
                   "canonical identity next cycle."
                << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
