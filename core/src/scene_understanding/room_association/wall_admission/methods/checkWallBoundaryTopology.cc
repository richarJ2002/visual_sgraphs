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
 * @file            checkWallBoundaryTopology.cc
 *
 * @brief           Implements SemanticsManager::checkWallBoundaryTopology(),
 *                  declared in SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::checkWallBoundaryTopology(
    semantic::Room                        *p_room_inout,
    geometric::Plane                      *p_candidateWall_in,
    const std::vector<geometric::Plane *> &roomWalls_in,
    bool                                  &isClashFree_out)
{
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_sysParams->roomSeg.boundaryTopology;
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    bool groundPlaneIsBad{};
    if (!(!topologyParameters.enabled || p_groundPlane == nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (!topologyParameters.enabled || p_groundPlane == nullptr ||
        groundPlaneIsBad)
    {
        isClashFree_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D groundPlaneGetGlobalEquation{};
    if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d groundEquation_world =
        groundPlaneGetGlobalEquation.coeffs();
    const double groundNormalNorm = groundEquation_world.head<3>().norm();

    if (!groundEquation_world.allFinite() || groundNormalNorm < 1e-8)
    {
        isClashFree_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector3d groundNormal_world =
        groundEquation_world.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_world =
        groundNormal_world.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_world =
        groundNormal_world.cross(groundAxisU_world).normalized();
    FiniteWallSegment2d candidateSegment;

    bool isBuilt{};
    if (buildFiniteWallSegment2d(p_candidateWall_in,
                                 groundNormal_world,
                                 groundAxisU_world,
                                 groundAxisV_world,
                                 topologyParameters.endpointTrimRatio,
                                 topologyParameters.minimumWallLength_m,
                                 candidateSegment,
                                 isBuilt) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: buildFiniteWallSegment2d returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isBuilt)
    {
        isClashFree_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The candidate's other face (reconcileWallFacePairs()) is the same
     * physical wall. Its fitted plane may differ by a few degrees, so the two
     * finite segments can cross, but that is not a clash; the face rule
     * decides which room owns which face. */
    geometric::Plane *p_candidateTwinFace = nullptr;
    if (p_candidateWall_in->getTwinFace(p_candidateTwinFace) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTwinFace returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<geometric::Plane *> weakerClashingWalls;

    for (geometric::Plane *p_existingWall : roomWalls_in)
    {
        if (p_existingWall == nullptr || p_existingWall == p_candidateTwinFace)
        {
            continue;
        }

        FiniteWallSegment2d existingSegment;

        bool isBuilt2{};
        if (buildFiniteWallSegment2d(p_existingWall,
                                     groundNormal_world,
                                     groundAxisU_world,
                                     groundAxisV_world,
                                     topologyParameters.endpointTrimRatio,
                                     topologyParameters.minimumWallLength_m,
                                     existingSegment,
                                     isBuilt2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: buildFiniteWallSegment2d returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!isBuilt2)
        {
            continue;
        }

        Eigen::Vector2d lineIntersection_world_m;
        double          candidateParameter = 0.0;
        double          existingParameter  = 0.0;

        bool hasIntersection{};
        if (intersectSupportingLines(candidateSegment,
                                     existingSegment,
                                     lineIntersection_world_m,
                                     candidateParameter,
                                     existingParameter,
                                     hasIntersection) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: intersectSupportingLines returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (!hasIntersection || candidateParameter < 0.0 ||
            candidateParameter > 1.0 || existingParameter < 0.0 ||
            existingParameter > 1.0)
        {
            continue;
        }

        const double candidateInteriorDistance_m =
            std::min(candidateParameter, 1.0 - candidateParameter) *
            candidateSegment.length_m;
        const double existingInteriorDistance_m =
            std::min(existingParameter, 1.0 - existingParameter) *
            existingSegment.length_m;

        if (std::max(candidateInteriorDistance_m, existingInteriorDistance_m) <=
            topologyParameters.maximumInteriorIntersection_m)
        {
            continue;
        }

        const double candidateSupport = candidateSegment.supportScore;
        const double existingSupport  = existingSegment.supportScore;
        const double weakerSupport =
            std::max(std::min(candidateSupport, existingSupport), 1e-8);
        const double supportRatio =
            std::max(candidateSupport, existingSupport) / weakerSupport;

        /*
         * Ambiguous or weaker candidates stay outside this room. Their Plane
         * objects remain valid and the orphan-wall pass can attach them to a
         * different room as additional free-space evidence becomes available.
         */
        if (supportRatio < topologyParameters.decisiveConflictSupportRatio ||
            candidateSupport <= existingSupport)
        {
            isClashFree_out = false;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        weakerClashingWalls.push_back(p_existingWall);
    }

    for (geometric::Plane *p_weakerWall : weakerClashingWalls)
    {
        bool room_inoutWasWallRemoved{};
        if (p_room_inout->removeWall(p_weakerWall, room_inoutWasWallRemoved) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            room_inoutWasWallRemoved = false;
            RCLCPP_WARN(
                rclcpp::get_logger("vs_graphs"),
                "%s: removeWall rejected its input; continuing as before.",
                __func__);
        }
        if (room_inoutWasWallRemoved)
        {
            int room_inoutId2{};
            if (p_room_inout->getId(room_inoutId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int weakerWallGetId{};
            if (p_weakerWall->getId(weakerWallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int candidateWallGetId2{};
            if (p_candidateWall_in->getId(candidateWallGetId2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Replaced clashing Wall#" << weakerWallGetId
                      << " in Room#" << room_inoutId2 << " with stronger Wall#"
                      << candidateWallGetId2 << "." << std::endl;
        }
    }

    /* Cross-room check: real walls only meet at shared corners -- a
     * candidate whose finite segment decisively crosses the interior of
     * another room's already-admitted wall is a modeling error, not a
     * legitimate admission. The candidate's twin face is skipped, as in
     * the within-room check above. Reject outright
     * rather than perturb the foreign room's wall: ownership of an
     * already-admitted wall is never taken by another room's admission
     * attempt, only by enforceUniqueWallOwnership()/validateRoomBoundaries()'
     * own intra-room repair. */
    std::vector<semantic::Room *> atlasAllRooms{};
    if (getAllRoomsInIdOrder(atlasAllRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_otherRoom : atlasAllRooms)
    {
        bool otherRoomIsBad{};
        if (!(p_otherRoom == nullptr) &&
            p_otherRoom->isBad(otherRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_otherRoom == nullptr || otherRoomIsBad ||
            p_otherRoom == p_room_inout)
        {
            continue;
        }

        std::vector<geometric::Plane *> otherRoomWalls{};
        if (p_otherRoom->getWalls(otherRoomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_otherWall : otherRoomWalls)
        {
            if (p_otherWall == nullptr || p_otherWall == p_candidateWall_in ||
                p_otherWall == p_candidateTwinFace)
            {
                continue;
            }

            FiniteWallSegment2d otherRoomSegment;

            bool isBuilt3{};
            if (buildFiniteWallSegment2d(p_otherWall,
                                         groundNormal_world,
                                         groundAxisU_world,
                                         groundAxisV_world,
                                         topologyParameters.endpointTrimRatio,
                                         topologyParameters.minimumWallLength_m,
                                         otherRoomSegment,
                                         isBuilt3) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: buildFiniteWallSegment2d returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!isBuilt3)
            {
                continue;
            }

            Eigen::Vector2d otherIntersection_world_m;
            double          candidateOtherParameter = 0.0;
            double          otherRoomParameter      = 0.0;

            bool hasIntersection2{};
            if (intersectSupportingLines(candidateSegment,
                                         otherRoomSegment,
                                         otherIntersection_world_m,
                                         candidateOtherParameter,
                                         otherRoomParameter,
                                         hasIntersection2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: intersectSupportingLines returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!hasIntersection2 || candidateOtherParameter < 0.0 ||
                candidateOtherParameter > 1.0 || otherRoomParameter < 0.0 ||
                otherRoomParameter > 1.0)
            {
                continue;
            }

            const double candidateOtherInteriorDistance_m =
                std::min(candidateOtherParameter,
                         1.0 - candidateOtherParameter) *
                candidateSegment.length_m;
            const double otherRoomInteriorDistance_m =
                std::min(otherRoomParameter, 1.0 - otherRoomParameter) *
                otherRoomSegment.length_m;

            if (std::max(candidateOtherInteriorDistance_m,
                         otherRoomInteriorDistance_m) <=
                topologyParameters.maximumInteriorIntersection_m)
            {
                continue;
            }

            int room_inoutId3{};
            if (p_room_inout->getId(room_inoutId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int otherRoomId{};
            if (p_otherRoom->getId(otherRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int candidateWallGetId3{};
            if (p_candidateWall_in->getId(candidateWallGetId3) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int otherWallGetId{};
            if (p_otherWall->getId(otherWallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Wall#" << candidateWallGetId3
                      << " rejected from Room#" << room_inoutId3
                      << ": crosses Room#" << otherRoomId
                      << "'s already-admitted Wall#" << otherWallGetId << "."
                      << std::endl;
            isClashFree_out = false;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }

    isClashFree_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
