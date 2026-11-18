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
 * @file            admitWallToRoom.cc
 *
 * @brief           Implements SemanticsManager::admitWallToRoom(), declared in
 *                  SemanticsManager.h.
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
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <iterator>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::admitWallToRoom(semantic::Room   *p_room_inout,
                                      geometric::Plane *p_candidateWall_in,
                                      bool             &wasAdmitted_out)
{
    bool room_inoutIsBad{};
    if (!(p_room_inout == nullptr) &&
        p_room_inout->isBad(room_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool candidateWallIsBad{};
    if (!(p_room_inout == nullptr || room_inoutIsBad ||
          p_candidateWall_in == nullptr) &&
        p_candidateWall_in->isBad(candidateWallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_room_inout == nullptr || room_inoutIsBad ||
        p_candidateWall_in == nullptr || candidateWallIsBad)
    {
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<geometric::Plane *> existingWalls{};
    if (p_room_inout->getWalls(existingWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const bool alreadyPresent =
        std::find(existingWalls.begin(),
                  existingWalls.end(),
                  p_candidateWall_in) != existingWalls.end();

    geometric::Plane *p_farSideGroundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_farSideGroundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d farSideGroundNormal_world = Eigen::Vector3d::Zero();
    bool            farSideGroundPlaneIsBad{};
    if ((p_farSideGroundPlane != nullptr) &&
        p_farSideGroundPlane->isBad(farSideGroundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_farSideGroundPlane != nullptr && !farSideGroundPlaneIsBad)
    {
        g2o::Plane3D farSideGroundPlaneGetGlobalEquation{};
        if (p_farSideGroundPlane->getGlobalEquation(
                farSideGroundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation =
            farSideGroundPlaneGetGlobalEquation.coeffs();
        const double groundEquationNormalNorm = groundEquation.head<3>().norm();
        if (groundEquation.allFinite() && groundEquationNormalNorm > 1e-8)
        {
            farSideGroundNormal_world =
                groundEquation.head<3>() / groundEquationNormalNorm;
        }
    }

    std::vector<semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    std::sort(allPassages.begin(),
              allPassages.end(),
              semantic::isEntityIdLess<semantic::Passage>);

    SemanticsManager::PassageSideEnforcementOutcome outcome{};
    if (enforcePassageApertureBackstop(p_room_inout,
                                       p_candidateWall_in,
                                       allPassages,
                                       farSideGroundNormal_world,
                                       outcome) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: enforcePassageApertureBackstop returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    switch (outcome)
    {
    case PassageSideEnforcementOutcome::REMOVED_UNBOUND:
    {
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    case PassageSideEnforcementOutcome::REROUTED:
    {
        wasAdmitted_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    case PassageSideEnforcementOutcome::NO_VIOLATION:
        break;
    }

    if (alreadyPresent)
    {
        wasAdmitted_out = true;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    WallAdmissionEvidence admissionEvidence{};
    if (evaluateWallAdmissionEvidence(p_candidateWall_in,
                                      p_sysParams,
                                      farSideGroundNormal_world,
                                      admissionEvidence) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateWallAdmissionEvidence returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!admissionEvidence.isAdmissible)
    {
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    bool isWallFaceForeignToRoom2{};
    if (isWallFaceForeignToRoom(p_room_inout,
                                p_candidateWall_in,
                                isWallFaceForeignToRoom2) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isWallFaceForeignToRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (isWallFaceForeignToRoom2)
    {
        int room_inoutId{};
        if (p_room_inout->getId(room_inoutId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int candidateWallGetId{};
        if (p_candidateWall_in->getId(candidateWallGetId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Wall#" << candidateWallGetId
                  << " rejected from Room#" << room_inoutId
                  << ": this face was observed from the opposite side, so it "
                     "bounds the neighbouring room."
                  << std::endl;
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    Eigen::Vector3d roomCentroid_world_m{};
    Eigen::Vector3d roomViewpoint_world_m{};
    bool            isCameraViewpoint{};
    if (p_sysParams->roomSeg.isSeeThroughGateEnabled &&
        getRoomViewpoints(p_room_inout,
                          roomCentroid_world_m,
                          roomViewpoint_world_m,
                          isCameraViewpoint) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomViewpoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d   candidateCentroid_world_m{};
    geometric::Plane *p_candidateTwinWall = nullptr;
    if (p_sysParams->roomSeg.isSeeThroughGateEnabled &&
        (p_candidateWall_in->getCentroid(candidateCentroid_world_m) !=
             geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
         p_candidateWall_in->getTwinFace(p_candidateTwinWall) !=
             geometric::PlaneStatus::PLANE_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid or getTwinFace returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * Visibility rules (semantic_constraints/visibility_rules/), in table
     * order: the passage rule judges a room the camera is not in from its
     * centroid, the solid-wall rule judges the camera's room from the camera.
     * A refused wall stays unowned for another room's admission. The
     * condition is re-evaluated on every attempt, so a hold ends by itself.
     */
    std::vector<geometric::Plane *> mappedPlanes{};
    if (p_sysParams->roomSeg.isSeeThroughGateEnabled && isCameraViewpoint &&
        p_atlas->getAllPlanes(mappedPlanes) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    semantic_constraints::VisibilityRuleInput visibilityInput{};
    visibilityInput.isCameraViewpoint         = isCameraViewpoint;
    visibilityInput.roomCentroid_world_m      = roomCentroid_world_m;
    visibilityInput.roomViewpoint_world_m     = roomViewpoint_world_m;
    visibilityInput.candidateCentroid_world_m = candidateCentroid_world_m;
    visibilityInput.p_candidateWall           = p_candidateWall_in;
    visibilityInput.p_candidateTwinWall       = p_candidateTwinWall;
    visibilityInput.p_roomWalls               = &existingWalls;
    visibilityInput.p_mapPassages             = &allPassages;
    visibilityInput.p_mapPlanes               = &mappedPlanes;
    geometric::Plane *p_blockingWall          = nullptr;
    if (semantic_constraints::isSeenThroughAnyWall(visibilityInput,
                                                   *p_sysParams,
                                                   p_blockingWall))
    {
        int candidateWallId{};
        int roomId{};
        if (p_candidateWall_in->getId(candidateWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            p_room_inout->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_blockingWall == nullptr)
        {
            /* Passage rule. Not deduplicated through loggedOrphanWallIds:
             * that set also gates the wall_pending event, which must stay
             * intact. */
            RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                        "[SemMgr] Wall#%d refused from Room#%d: beyond a "
                        "passage wall seen from the room centre.",
                        candidateWallId,
                        roomId);
        }
        else if (loggedOrphanWallIds.insert(candidateWallId).second)
        {
            /* Solid-wall rule. Logged once per pending episode; the set is
             * the same one the orphan-wall pending event uses, so the two
             * never repeat. */
            int nearerWallId{};
            if (p_blockingWall->getId(nearerWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            map::Map *p_activeMap = nullptr;
            if (p_atlas->getCurrentMap(p_activeMap) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long activeMapId{};
            if ((p_activeMap != nullptr) &&
                p_activeMap->getId(activeMapId) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Wall#" << candidateWallId
                      << " held from Room#" << roomId
                      << ": seen through nearer Wall#" << nearerWallId << "."
                      << std::endl;
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << (p_activeMap != nullptr
                              ? static_cast<long long>(activeMapId)
                              : -1)
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << candidateWallId
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\"SEEN_THROUGH_NEARER_WALL\","
                         "\"support\":"
                      << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount
                      << ",\"pending_age\":0}" << std::endl;
        }
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*
     * Boundary topology: a candidate whose finite segment decisively crosses
     * a wall of this room or another room's admitted wall stays out.
     */
    bool isClashFree{};
    if (checkWallBoundaryTopology(p_room_inout,
                                  p_candidateWall_in,
                                  existingWalls,
                                  isClashFree) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: checkWallBoundaryTopology returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isClashFree)
    {
        wasAdmitted_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*
     * Admits the candidate. The see-through gate also holds in reverse: when
     * a far wall was mapped before the nearer wall it is seen through (the
     * camera looked through the doorway first), admitting the nearer wall
     * detaches that far wall again, so admission order cannot let the room
     * own the face beyond a doorway.
     */
    if (attachAdmittedWall(p_room_inout,
                           p_candidateWall_in,
                           existingWalls,
                           allPassages,
                           roomViewpoint_world_m,
                           isCameraViewpoint) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: attachAdmittedWall returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    wasAdmitted_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
