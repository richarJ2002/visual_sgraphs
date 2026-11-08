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
 * @brief           Implements
 *                  Utils::fuseDuplicateRoomsAfterMerge(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/private_functions/private_functions.h"
#include "scene_understanding/scene_graph/Floor/Floor.h"
#include "scene_understanding/scene_graph/Floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"
#include "scene_understanding/semantic/value_order/value_order.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::fuseDuplicateRoomsAfterMerge(
    atlas::Atlas *p_atlas_inout,
    map::Map     *p_map_inout,
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
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    unsigned long            mapId{};
    if ((p_atlas_inout->getSceneGraph(p_sceneGraph) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        ((p_map_inout != nullptr) &&
         (p_map_inout->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
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
        if (p_sceneGraph->getMapPassagesInIdOrder(mapId, mergePassages) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the scene graph has no scene map for map %lu; "
                         "continuing without its passages.",
                         __func__,
                         mapId);
        }
        const bool roomsAreSeparatedByPassage = std::any_of(
            mergePassages.begin(),
            mergePassages.end(),
            [&retainedCentroid_world_m,
             &importedCentroid_world_m,
             &mergeGroundNormal_world,
             mergeOpeningMargin_m,
             mergeMinimumSideDistance_m](semantic::Passage *p_passage)
            {
                bool crossesOpening{};
                if (crossesPassablePassageOpening(retainedCentroid_world_m,
                                                  importedCentroid_world_m,
                                                  p_passage,
                                                  mergeGroundNormal_world,
                                                  mergeOpeningMargin_m,
                                                  mergeMinimumSideDistance_m,
                                                  crossesOpening) !=
                    UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    /*!
                     * crossesPassablePassageOpening cannot fail; continue as
                     * before.
                     */
                }
                return crossesOpening;
            });

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

        struct WallTransfer
        {
            geometric::Plane  *p_wall;
            semantic::Room    *p_targetRoom;
            semantic::Passage *p_separatingPassage;
        };

        std::vector<WallTransfer> wallTransfers;
        wallTransfers.reserve(importedWalls.size());
        std::vector<semantic::Room *> mapRooms{};
        if (p_sceneGraph->getMapRoomsInIdOrder(mapId, mapRooms) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the scene graph has no scene map for map %lu; "
                         "continuing without its rooms.",
                         __func__,
                         mapId);
        }
        std::sort(mapRooms.begin(),
                  mapRooms.end(),
                  semantic::isEntityIdLess<semantic::Room>);

        for (geometric::Plane *p_importedWall : importedWalls)
        {
            bool importedWallIsBad2{};
            if (!(p_importedWall == nullptr) &&
                p_importedWall->isBad(importedWallIsBad2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_importedWall == nullptr || importedWallIsBad2)
            {
                continue;
            }

            Eigen::Vector3d importedWallGetCentroid{};
            if (p_importedWall->getCentroid(importedWallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector3d importedWallCentroid_world_m =
                importedWallGetCentroid.cast<double>();

            vs_graphs::core::semantic::Passage *p_separatingPassage = nullptr;

            for (vs_graphs::core::semantic::Passage *p_passage : mergePassages)
            {
                bool crossesOpening{};
                if ((p_passage != nullptr) &&
                    crossesPassablePassageOpening(retainedCentroid_world_m,
                                                  importedWallCentroid_world_m,
                                                  p_passage,
                                                  mergeGroundNormal_world,
                                                  mergeOpeningMargin_m,
                                                  mergeMinimumSideDistance_m,
                                                  crossesOpening) !=
                        UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    /*!
                     * crossesPassablePassageOpening cannot fail; continue as
                     * before.
                     */
                }
                if (p_passage != nullptr && crossesOpening)
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            semantic::Room *p_targetRoom = p_bestRetainedRoom;

            /* Never introduce another owner when a distinct confirmed room
             * already owns this wall. The imported duplicate is retired below,
             * leaving that confirmed owner unchanged. */
            for (semantic::Room *p_existingOwner : mapRooms)
            {
                bool existingOwnerIsBad{};
                if (!(p_existingOwner == nullptr) &&
                    p_existingOwner->isBad(existingOwnerIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                semantic::Room::RoomVariant existingOwnerRoomVariant{};
                if (!(p_existingOwner == nullptr || existingOwnerIsBad ||
                      p_existingOwner == p_importedRoom ||
                      p_existingOwner == p_bestRetainedRoom) &&
                    p_existingOwner->getRoomVariant(existingOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_existingOwner == nullptr || existingOwnerIsBad ||
                    p_existingOwner == p_importedRoom ||
                    p_existingOwner == p_bestRetainedRoom ||
                    existingOwnerRoomVariant !=
                        semantic::Room::RoomVariant::ROOM)
                {
                    continue;
                }

                std::vector<geometric::Plane *> ownerWalls{};
                if (p_existingOwner->getWalls(ownerWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (std::find(ownerWalls.begin(),
                              ownerWalls.end(),
                              p_importedWall) != ownerWalls.end())
                {
                    p_targetRoom = p_existingOwner;
                    break;
                }
            }

            if (p_separatingPassage != nullptr &&
                p_targetRoom == p_bestRetainedRoom)
            {
                vs_graphs::core::semantic::Room *p_farSideRoom = nullptr;
                if (p_separatingPassage->getProspectiveRoom(p_farSideRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                bool farSideRoomIsBad{};
                if (!(p_farSideRoom == nullptr) &&
                    p_farSideRoom->isBad(farSideRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_farSideRoom == nullptr || farSideRoomIsBad ||
                    p_farSideRoom == p_importedRoom)
                {
                    p_farSideRoom = nullptr;
                }

                p_targetRoom = p_farSideRoom;
            }

            wallTransfers.push_back(
                {p_importedWall, p_targetRoom, p_separatingPassage});
        }

        /* Commit the precomputed ownership transaction. Removing the duplicate
         * edge before adding its destination prevents transient double
         * ownership inside the authorized room fusion. */
        for (const WallTransfer &transfer : wallTransfers)
        {
            bool importedRoomWasWallRemoved{};
            if (p_importedRoom->removeWall(transfer.p_wall,
                                           importedRoomWasWallRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                importedRoomWasWallRemoved = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
            }

            if (transfer.p_targetRoom == nullptr)
            {
                int id{};
                if (transfer.p_separatingPassage->getId(id) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int getId2{};
                if (transfer.p_wall->getId(getId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemanticMerge] Far-side Wall#" << getId2
                          << " at Passage#" << id
                          << " has no prospective; left unbound." << std::endl;
                continue;
            }

            if (transfer.p_targetRoom->setWalls(transfer.p_wall) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (transfer.p_separatingPassage != nullptr &&
                transfer.p_targetRoom != p_bestRetainedRoom)
            {
                int id2{};
                if (transfer.p_targetRoom->getId(id2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int getId3{};
                if (transfer.p_wall->getId(getId3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemanticMerge] Redirected far-side Wall#"
                          << getId3 << " to stable Room#" << id2 << "."
                          << std::endl;
            }
        }

        std::vector<vs_graphs::core::semantic::Passage *>
            importedRoomPassages{};
        if (p_importedRoom->getPassages(importedRoomPassages) !=
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
            if (p_bestRetainedRoom->setDoorways(p_importedPassage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        std::vector<vs_graphs::core::semantic::Passage *> mapAllPassages{};
        if (p_sceneGraph->getMapPassagesInIdOrder(mapId, mapAllPassages) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the scene graph has no scene map for map %lu; "
                         "continuing without its passages.",
                         __func__,
                         mapId);
        }
        for (semantic::Passage *p_passage : mapAllPassages)
        {
            if (p_passage != nullptr)
            {
                bool passageWasRoomReplaced{};
                if (p_passage->replaceProspectiveRoom(p_importedRoom,
                                                      p_bestRetainedRoom,
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

        const double retainedWeight = static_cast<double>(
            std::max<std::size_t>(retainedWalls.size(), 1U));

        const double importedWeight = static_cast<double>(
            std::max<std::size_t>(importedWalls.size(), 1U));

        Eigen::Vector3d bestRetainedRoomCentroid{};
        if (p_bestRetainedRoom->getCentroid(bestRetainedRoomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d fusedCentroid_world_m =
            (retainedWeight * bestRetainedRoomCentroid +
             importedWeight * importedCentroid_world_m) /
            (retainedWeight + importedWeight);

        if (p_bestRetainedRoom->setCentroid(fusedCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        geometric::Plane *p_bestRetainedRoomGroundPlane = nullptr;
        if (p_bestRetainedRoom->getGroundPlane(p_bestRetainedRoomGroundPlane) !=
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
            if (p_importedRoom->getGroundPlane(p_importedRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGroundPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_bestRetainedRoom->setGroundPlane(p_importedRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGroundPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        bool bestRetainedRoomHasKnownLabel{};
        if (p_bestRetainedRoom->getHasKnownLabel(
                bestRetainedRoomHasKnownLabel) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool importedRoomHasKnownLabel2{};
        if ((!bestRetainedRoomHasKnownLabel) &&
            p_importedRoom->getHasKnownLabel(importedRoomHasKnownLabel2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHasKnownLabel returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!bestRetainedRoomHasKnownLabel && importedRoomHasKnownLabel2)
        {
            if (p_bestRetainedRoom->setHasKnownLabel(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setHasKnownLabel returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::string importedRoomName{};
            if (p_importedRoom->getName(importedRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getName returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_bestRetainedRoom->setName(importedRoomName) !=
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
        if (p_importedRoom->hasPreviouslyVisited(
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
            if (p_bestRetainedRoom->setPreviouslyVisited(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setPreviouslyVisited returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        std::vector<semantic::Floor *> mapAllFloors{};
        if (p_sceneGraph->getMapFloorsInIdOrder(mapId, mapAllFloors) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: the scene graph has no scene map for map %lu; "
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

        if (p_sceneGraph->eraseRoom(mapId, p_importedRoom) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseRoom found no scene map for map %lu; "
                         "continuing as before.",
                         __func__,
                         mapId);
        }
        if (p_importedRoom->clearWalls() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
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
        if (p_sceneGraph->markRoomBad(mapId, p_importedRoom) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: markRoomBad found no scene map for map %lu; the "
                         "room is flagged alone; continuing as before.",
                         __func__,
                         mapId);
        }
        if (p_map_inout->replaceLinkedRoom(p_importedRoom,
                                           p_bestRetainedRoom) !=
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

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
