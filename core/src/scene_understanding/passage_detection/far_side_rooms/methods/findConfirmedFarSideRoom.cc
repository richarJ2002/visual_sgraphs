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
 * @file            findConfirmedFarSideRoom.cc
 *
 * @brief           Implements SemanticsManager::findConfirmedFarSideRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findConfirmedFarSideRoom(
    const std::vector<semantic::Room *> &allRooms_in,
    semantic::Passage                   *p_passage_inout,
    semantic::Room                      *p_knownRoom_in,
    bool                                &farSideConfirmedRoomExists_out,
    semantic::Room                     *&p_existingFarSideRoom_out)
{
    farSideConfirmedRoomExists_out = false;
    p_existingFarSideRoom_out      = nullptr;

    /*!
     * Same class of flaw as the promotion search further
     * below (and the same fix): a bare same-side-of-the-
     * infinite-passage-plane sign test, with only a 0.05m
     * epsilon, is satisfied by any room past this passage
     * on the far side -- including a room several doors
     * down the same corridor that is nowhere near this
     * specific opening. This gate runs BEFORE any
     * prospective placeholder exists, so it must carry the
     * same rigor itself rather than relying on the
     * promotion search to catch it later: the bounded
     * aperture test (segmentCrossesPassageOpening) plus the
     * intervening-wall test (segmentCrossesForeignWall).
     */
    geometric::Plane *p_anteChurnGroundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_anteChurnGroundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a "
                     "failure status although it cannot fail; "
                     "continuing as before.",
                     __func__);
    }

    bool anteChurnGroundPlaneIsBad{};
    if ((p_knownRoom_in != nullptr && p_anteChurnGroundPlane != nullptr) &&
        p_anteChurnGroundPlane->isBad(anteChurnGroundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_knownRoom_in != nullptr && p_anteChurnGroundPlane != nullptr &&
        !anteChurnGroundPlaneIsBad)
    {
        g2o::Plane3D anteChurnGroundPlaneGetGlobalEquation{};
        if (p_anteChurnGroundPlane->getGlobalEquation(
                anteChurnGroundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            /*!
             * getGlobalEquation cannot fail; continue as
             * before.
             */
        }
        const Eigen::Vector4d anteChurnGroundEq =
            anteChurnGroundPlaneGetGlobalEquation.coeffs();
        const double anteChurnGroundNorm = anteChurnGroundEq.head<3>().norm();

        if (anteChurnGroundEq.allFinite() && anteChurnGroundNorm > 1e-8)
        {
            const Eigen::Vector3d anteChurnGroundNormal_world =
                anteChurnGroundEq.head<3>() / anteChurnGroundNorm;
            const Eigen::Vector3d anteChurnGroundAxisU_world =
                anteChurnGroundNormal_world.unitOrthogonal().normalized();
            const Eigen::Vector3d anteChurnGroundAxisV_world =
                anteChurnGroundNormal_world.cross(anteChurnGroundAxisU_world)
                    .normalized();
            const types::SystemParams::RoomSeg::PassagePartition
                &anteChurnPartitionParameters =
                    p_sysParams->roomSeg.passagePartition;
            const double anteChurnOpeningMargin_m = static_cast<double>(
                anteChurnPartitionParameters.openingMargin_m);
            const double anteChurnMinimumSideDistance_m = static_cast<double>(
                anteChurnPartitionParameters.minimumSideDistance_m);
            const types::SystemParams::RoomSeg::BoundaryTopology
                &anteChurnTopologyParameters =
                    p_sysParams->roomSeg.boundaryTopology;
            Eigen::Vector3d knownRoomCentroid{};
            if (p_knownRoom_in->getCentroid(knownRoomCentroid) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            const std::vector<vs_graphs::core::semantic::Room *>
                anteChurnExcludedRooms = {p_knownRoom_in};

            for (vs_graphs::core::semantic::Room *p_otherRoom : allRooms_in)
            {
                bool otherRoomIsBad{};
                if (!(p_otherRoom == nullptr) &&
                    p_otherRoom->isBad(otherRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status "
                                 "although it cannot fail; continuing "
                                 "as before.",
                                 __func__);
                }
                semantic::Room::RoomVariant otherRoomRoomVariant{};
                if (!(p_otherRoom == nullptr || otherRoomIsBad ||
                      p_otherRoom == p_knownRoom_in) &&
                    p_otherRoom->getRoomVariant(otherRoomRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getRoomVariant cannot fail; continue as
                     * before.
                     */
                }
                if (p_otherRoom == nullptr || otherRoomIsBad ||
                    p_otherRoom == p_knownRoom_in ||
                    otherRoomRoomVariant ==
                        vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
                {
                    continue;
                }

                Eigen::Vector3d otherRoomCentroid{};
                if (p_otherRoom->getCentroid(otherRoomCentroid) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getCentroid cannot fail; continue as
                     * before.
                     */
                }
                bool crossesPassageOpening{};
                if (segmentCrossesPassageOpening(knownRoomCentroid,
                                                 otherRoomCentroid,
                                                 p_passage_inout,
                                                 anteChurnGroundNormal_world,
                                                 anteChurnOpeningMargin_m,
                                                 anteChurnMinimumSideDistance_m,
                                                 crossesPassageOpening) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: segmentCrossesPassageOpening "
                                 "returned a failure status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                if (!crossesPassageOpening)
                {
                    continue;
                }

                Eigen::Vector3d otherRoomCentroid2{};
                if (p_otherRoom->getCentroid(otherRoomCentroid2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getCentroid cannot fail; continue as
                     * before.
                     */
                }
                bool crossesForeignWall{};
                if (segmentCrossesForeignWall(
                        knownRoomCentroid,
                        otherRoomCentroid2,
                        anteChurnExcludedRooms,
                        allRooms_in,
                        anteChurnGroundAxisU_world,
                        anteChurnGroundAxisV_world,
                        anteChurnGroundNormal_world,
                        anteChurnTopologyParameters.endpointTrimRatio,
                        anteChurnTopologyParameters.minimumWallLength_m,
                        crossesForeignWall) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: segmentCrossesForeignWall "
                                 "returned a failure status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
                if (crossesForeignWall)
                {
                    continue;
                }

                farSideConfirmedRoomExists_out = true;
                p_existingFarSideRoom_out      = p_otherRoom;
                break;
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
