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
 * @file            fuseDuplicateRoomsAfterMerge.cc
 *
 * @brief           Implements fuseDuplicateRoomsAfterMerge(), declared in
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

void fuseDuplicateRoomsAfterMerge(
    SemanticsManager *p_semanticsManager_inout,
    map::Map         *p_map_inout,
    const std::vector<std::pair<semantic::Room *, semantic::Room *>>
        &duplicateRoomPairs_in)
{
    types::SystemParams *p_systemParameters = nullptr;
    if (types::SystemParams::getParams(p_systemParameters) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long mapId{};
    if ((p_map_inout != nullptr) &&
        (p_map_inout->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Fuse each imported room into the retained room it was paired with. */
    for (const std::pair<semantic::Room *, semantic::Room *> &duplicatePair :
         duplicateRoomPairs_in)
    {
        semantic::Room *p_importedRoom     = duplicatePair.first;
        semantic::Room *p_bestRetainedRoom = duplicatePair.second;
        if (p_map_inout == nullptr || p_importedRoom == nullptr ||
            p_bestRetainedRoom == nullptr ||
            p_importedRoom == p_bestRetainedRoom)
        {
            continue;
        }

        bool importedRoomIsBad{};
        if (p_importedRoom->isBad(importedRoomIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool retainedRoomIsBad{};
        if (p_bestRetainedRoom->isBad(retainedRoomIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (importedRoomIsBad || retainedRoomIsBad)
        {
            continue;
        }

        Eigen::Vector3d importedCentroid_world_m{};
        if (p_importedRoom->getCentroid(importedCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!importedCentroid_world_m.allFinite())
        {
            continue;
        }

        std::vector<geometric::Plane *> importedWalls{};
        if (p_importedRoom->getWalls(importedWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<geometric::Plane *> retainedWalls{};
        if (p_bestRetainedRoom->getWalls(retainedWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * @brief           A wall can bound the retained (near) room only when
         *                  no passable passage aperture separates its centroid
         *                  from the retained room centre. Otherwise it belongs
         *                  to the far-side room (the passage's prospective, or
         *                  a confirmed room that already resolved that
         *                  prospective). Copying such a wall into the retained
         *                  room would both corrupt the near boundary and hand
         *                  the far room's evidence to the near room, so the
         *                  far-side consultation happens here.
         */
        geometric::Plane *p_mergeGroundPlane = nullptr;

        std::vector<geometric::Plane *> mapAllPlanes{};
        if (p_map_inout->getAllPlanes(mapAllPlanes) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_plane : mapAllPlanes)
        {
            bool planeIsBad{};
            if ((p_plane != nullptr) &&
                p_plane->isBad(planeIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            geometric::Plane::PlaneVariant planeType{};
            if ((p_plane != nullptr && !planeIsBad) &&
                p_plane->getPlaneType(planeType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane != nullptr && !planeIsBad &&
                planeType == geometric::Plane::PlaneVariant::GROUND)
            {
                p_mergeGroundPlane = p_plane;
                break;
            }
        }

        Eigen::Vector3d mergeGroundNormal_world = Eigen::Vector3d::Zero();
        if (p_mergeGroundPlane != nullptr)
        {
            g2o::Plane3D mergeGroundPlaneGetGlobalEquation{};
            if (p_mergeGroundPlane->getGlobalEquation(
                    mergeGroundPlaneGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector4d groundEq =
                mergeGroundPlaneGetGlobalEquation.coeffs();
            const double groundNormalNorm = groundEq.head<3>().norm();
            if (groundEq.allFinite() && groundNormalNorm > 1e-8)
            {
                mergeGroundNormal_world = groundEq.head<3>() / groundNormalNorm;
            }
        }

        const double mergeOpeningMargin_m =
            p_systemParameters != nullptr
                ? static_cast<double>(p_systemParameters->roomSeg
                                          .passagePartition.openingMargin_m)
                : 0.20;

        const double mergeMinimumSideDistance_m =
            p_systemParameters != nullptr
                ? static_cast<double>(
                      p_systemParameters->roomSeg.passagePartition
                          .minimumSideDistance_m)
                : 0.30;

        Eigen::Vector3d retainedCentroid_world_m{};
        if (p_bestRetainedRoom->getCentroid(retainedCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<semantic::Passage *> mergePassages{};
        if (p_semanticsManager_inout->getMapPassagesInIdOrder(mapId,
                                                              mergePassages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: the semantics manager has no scene graph for map %lu; "
                "continuing without its passages.",
                __func__,
                mapId);
        }
        bool roomsAreSeparatedByPassage = false;
        for (semantic::Passage *p_passage : mergePassages)
        {
            bool crossesOpening{};
            if (crossesPassablePassageOpening(retainedCentroid_world_m,
                                              importedCentroid_world_m,
                                              p_passage,
                                              mergeGroundNormal_world,
                                              mergeOpeningMargin_m,
                                              mergeMinimumSideDistance_m,
                                              crossesOpening) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                /*!
                 * crossesPassablePassageOpening cannot fail; continue as
                 * before.
                 */
            }
            if (crossesOpening)
            {
                roomsAreSeparatedByPassage = true;
                break;
            }
        }

        if (roomsAreSeparatedByPassage)
        {
            int importedRoomId3{};
            if (p_importedRoom->getId(importedRoomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int bestRetainedRoomId{};
            if (p_bestRetainedRoom->getId(bestRetainedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemanticMerge] Preserved Room#" << importedRoomId3
                      << " and Room#" << bestRetainedRoomId
                      << "; a passable passage separates their centroids."
                      << std::endl;
            continue;
        }

        /* Move the imported walls; a far-side wall goes to its room */
        moveImportedRoomWalls(p_semanticsManager_inout,
                              mapId,
                              p_importedRoom,
                              p_bestRetainedRoom,
                              importedWalls,
                              retainedCentroid_world_m,
                              mergeGroundNormal_world,
                              mergeOpeningMargin_m,
                              mergeMinimumSideDistance_m,
                              mergePassages);

        /* Passages, centroid, ground plane, label and visit flag */
        fuseImportedRoomAttributes(p_semanticsManager_inout,
                                   mapId,
                                   p_importedRoom,
                                   p_bestRetainedRoom,
                                   importedCentroid_world_m,
                                   importedWalls,
                                   retainedWalls);

        std::vector<semantic::Floor *> mapAllFloors{};
        if (p_semanticsManager_inout->getMapFloorsInIdOrder(mapId,
                                                            mapAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: the semantics manager has no scene graph for map %lu; "
                "continuing without its floors.",
                __func__,
                mapId);
        }
        for (semantic::Floor *p_floor : mapAllFloors)
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_importedRoom,
                                         p_bestRetainedRoom,
                                         floorWasRoomReplaced) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    floorWasRoomReplaced = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: replaceRoom rejected its input; "
                                "continuing as before.",
                                __func__);
                }
            }
        }

        if (p_semanticsManager_inout->eraseRoom(mapId, p_importedRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseRoom found no scene graph for map %lu; "
                         "continuing as before.",
                         __func__,
                         mapId);
        }
        /* The surviving room takes the imported room's key frames. */
        if (p_importedRoom->clearWalls() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_importedRoom->transferKeyFrames(p_bestRetainedRoom) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearWalls/transferKeyFrames returned a failure "
                         "status although they cannot fail; continuing.",
                         __func__);
        }
        if (p_importedRoom->clearPassages() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_semanticsManager_inout->markRoomBad(mapId, p_importedRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: markRoomBad found no scene graph for map %lu; the "
                "room is flagged alone; continuing as before.",
                __func__,
                mapId);
        }
        int linkFusedRoomId{};
        int linkKeptRoomId{};
        if ((p_importedRoom->getId(linkFusedRoomId) !=
             semantic::RoomStatus::ROOM_STATUS_SUCCESS) ||
            (p_bestRetainedRoom->getId(linkKeptRoomId) !=
             semantic::RoomStatus::ROOM_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map_inout->replaceLinkedRoom(linkFusedRoomId, linkKeptRoomId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: replaceLinkedRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        int importedRoomId4{};
        if (p_importedRoom->getId(importedRoomId4) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int bestRetainedRoomId2{};
        if (p_bestRetainedRoom->getId(bestRetainedRoomId2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout
            << "[SemanticMerge] Fused duplicate Room#" << importedRoomId4
            << " into Room#" << bestRetainedRoomId2 << " (centroid distance "
            << (retainedCentroid_world_m - importedCentroid_world_m).norm()
            << " m)." << std::endl;
    }
}

} /* namespace core */
} /* namespace vs_graphs */
