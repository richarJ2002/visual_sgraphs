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

#include "scene_understanding/scene_graph/scene_graph/SceneGraph.h"
#include "scene_understanding/semantic/value_order/value_order.h"
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
    scene_graph::SceneGraph         *p_sceneGraph = nullptr;
    if (p_atlas->getSceneGraph(p_sceneGraph) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_sceneGraph->getAllPassagesInIdOrder(allPassages) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
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
     * Passage-wall gate, for a room the camera is not in. Such a room is
     * judged from its centroid. A wall that carries a passage is a known
     * boundary between two rooms, so a candidate whose centroid lies beyond
     * a passage wall, seen from this room's centroid, bounds the room on the
     * other side and is refused. The aperture backstop above only catches a
     * line through the passage's opening; this catches a line through the
     * solid part of the same wall, e.g. the camera room's side wall offered
     * to the room beyond its door wall by a free-space cluster whose points
     * lie near that side wall's plane. The separation test and widening are
     * the see-through gate's below; the candidate and its twin face never
     * block. A refused wall stays unowned for the camera room's admission.
     */
    if (p_sysParams->roomSeg.isSeeThroughGateEnabled && !isCameraViewpoint)
    {
        std::vector<geometric::Plane *> passageWalls{};
        for (semantic::Passage *p_passage : allPassages)
        {
            bool                            passageIsBad{true};
            std::vector<geometric::Plane *> associateWalls{};
            if (p_passage == nullptr ||
                p_passage->isBad(passageIsBad) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                passageIsBad ||
                p_passage->getAssociateWalls(associateWalls) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                continue;
            }
            std::copy_if(associateWalls.begin(),
                         associateWalls.end(),
                         std::back_inserter(passageWalls),
                         [p_candidateWall_in,
                          p_candidateTwinWall](geometric::Plane *p_passageWall)
                         {
                             return p_passageWall != p_candidateWall_in &&
                                    p_passageWall != p_candidateTwinWall;
                         });
        }
        bool isBeyondPassageWall{};
        if (hasSeparatingFiniteWall(
                passageWalls,
                roomCentroid_world_m,
                candidateCentroid_world_m,
                static_cast<double>(p_sysParams->semSeg.reassociate
                                        .wallExtension.maximumInPlaneGap_m),
                true,
                isBeyondPassageWall) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasSeparatingFiniteWall returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (isBeyondPassageWall)
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
            /* Not deduplicated through loggedOrphanWallIds: that set also
             * gates the wall_pending event, which must stay intact. */
            RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                        "[SemMgr] Wall#%d refused from Room#%d: beyond a "
                        "passage wall seen from the room centre.",
                        candidateWallId,
                        roomId);
            wasAdmitted_out = false;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }

    /*!
     * See-through gate, only for the room the camera is in: a candidate whose
     * centroid the camera sees through a nearer wall N (through N's doorway)
     * lies beyond N, so it is held pending. N is any wall of this room, or
     * any mapped plane whose leading vote is WALL before its type is
     * confirmed: a near wall seen mostly as door-side strips can stay
     * unconfirmed while the far wall behind it is already confirmed, and
     * must still block it. A wall with a passage in it keeps blocking; the
     * far-side backstop above has already routed a candidate seen through
     * the passage's own opening. The candidate's twin face is the same
     * physical wall and never blocks. The condition is re-evaluated on every
     * attempt, so the hold ends by itself when
     *  - the camera crosses N's plane (camera and candidate are then on the
     *    same side of N), or
     *  - N's leading vote or type moves away from WALL.
     * hasSeparatingFiniteWall() requires both points at least 0.10 m (equal
     * to passage_partition.minimum_side_distance) from N's plane and the
     * crossing inside N's observed in-plane extent and height, widened by
     * wall_extension.maximum_in_plane_gap so a doorway at N's end counts.
     */
    if (p_sysParams->roomSeg.isSeeThroughGateEnabled && isCameraViewpoint)
    {
        std::vector<geometric::Plane *> mappedPlanes{};
        if (p_atlas->getAllPlanes(mappedPlanes) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> unconfirmedWalls{};
        for (geometric::Plane *p_mappedPlane : mappedPlanes)
        {
            bool                           mappedPlaneIsBad{true};
            geometric::Plane::PlaneVariant mappedPlaneType{};
            geometric::Plane::PlaneVariant leadingPlaneType{};
            if (p_mappedPlane == nullptr ||
                p_mappedPlane->isBad(mappedPlaneIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                mappedPlaneIsBad ||
                p_mappedPlane->getPlaneType(mappedPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                p_mappedPlane->getExpectedPlaneType(leadingPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                continue;
            }
            if (mappedPlaneType != geometric::Plane::PlaneVariant::WALL &&
                leadingPlaneType == geometric::Plane::PlaneVariant::WALL &&
                std::find(existingWalls.begin(),
                          existingWalls.end(),
                          p_mappedPlane) == existingWalls.end())
            {
                unconfirmedWalls.push_back(p_mappedPlane);
            }
        }
        std::vector<geometric::Plane *> blockerWalls = existingWalls;
        blockerWalls.insert(blockerWalls.end(),
                            unconfirmedWalls.begin(),
                            unconfirmedWalls.end());

        for (geometric::Plane *p_nearerWall : blockerWalls)
        {
            if (p_nearerWall == nullptr || p_nearerWall == p_candidateWall_in ||
                p_nearerWall == p_candidateTwinWall)
            {
                continue;
            }
            bool isSeenThroughNearerWall{};
            if (hasSeparatingFiniteWall(
                    {p_nearerWall},
                    roomViewpoint_world_m,
                    candidateCentroid_world_m,
                    static_cast<double>(p_sysParams->semSeg.reassociate
                                            .wallExtension.maximumInPlaneGap_m),
                    true,
                    isSeenThroughNearerWall) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: hasSeparatingFiniteWall returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!isSeenThroughNearerWall)
            {
                continue;
            }

            int candidateWallId{};
            if (p_candidateWall_in->getId(candidateWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            /* Logged once per pending episode; the set is the same one the
             * orphan-wall pending event uses, so the two never repeat. */
            if (loggedOrphanWallIds.insert(candidateWallId).second)
            {
                int nearerWallId{};
                if (p_nearerWall->getId(nearerWallId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int roomId{};
                if (p_room_inout->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
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
                                 "although it cannot fail; continuing as "
                                 "before.",
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
                          << ": seen through nearer Wall#" << nearerWallId
                          << "." << std::endl;
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
    }

    /*
     * Admits the candidate. The see-through gate also holds in reverse: when
     * a far wall was mapped before the nearer wall it is seen through (the
     * camera looked through the doorway first), admitting the nearer wall
     * detaches that far wall again, so admission order cannot let the room
     * own the face beyond a doorway.
     */
    const auto admitCandidateWall = [&]()
    {
        if (p_room_inout->setWalls(p_candidateWall_in) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_sysParams->roomSeg.isSeeThroughGateEnabled ||
            !isCameraViewpoint ||
            std::any_of(allPassages.begin(),
                        allPassages.end(),
                        [p_candidateWall_in](semantic::Passage *p_passage)
                        {
                            bool                            passageIsBad{true};
                            std::vector<geometric::Plane *> associateWalls{};
                            return p_passage != nullptr &&
                                   p_passage->isBad(passageIsBad) ==
                                       semantic::PassageStatus::
                                           PASSAGE_STATUS_SUCCESS &&
                                   !passageIsBad &&
                                   p_passage->getAssociateWalls(
                                       associateWalls) ==
                                       semantic::PassageStatus::
                                           PASSAGE_STATUS_SUCCESS &&
                                   std::find(associateWalls.begin(),
                                             associateWalls.end(),
                                             p_candidateWall_in) !=
                                       associateWalls.end();
                        }))
        {
            return;
        }
        for (geometric::Plane *p_fartherWall : existingWalls)
        {
            Eigen::Vector3d fartherCentroid_world_m{};
            if (p_fartherWall == nullptr ||
                p_fartherWall == p_candidateWall_in ||
                p_fartherWall->getCentroid(fartherCentroid_world_m) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                continue;
            }
            bool isSeenThroughCandidate{};
            if (hasSeparatingFiniteWall(
                    {p_candidateWall_in},
                    roomViewpoint_world_m,
                    fartherCentroid_world_m,
                    static_cast<double>(p_sysParams->semSeg.reassociate
                                            .wallExtension.maximumInPlaneGap_m),
                    false,
                    isSeenThroughCandidate) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasSeparatingFiniteWall returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            bool wasWallRemoved{};
            if (!isSeenThroughCandidate ||
                p_room_inout->removeWall(p_fartherWall, wasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                !wasWallRemoved)
            {
                continue;
            }
            int fartherWallId{}, candidateWallId{}, roomId{};
            if (p_fartherWall->getId(fartherWallId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                p_candidateWall_in->getId(candidateWallId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
                p_room_inout->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Wall#" << fartherWallId
                      << " detached from Room#" << roomId
                      << ": seen through nearer Wall#" << candidateWallId << "."
                      << std::endl;
        }
    };

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
        admitCandidateWall();
        wasAdmitted_out = true;
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
        admitCandidateWall();
        wasAdmitted_out = true;
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
        admitCandidateWall();
        wasAdmitted_out = true;
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

    for (geometric::Plane *p_existingWall : existingWalls)
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
            wasAdmitted_out = false;
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
    if (p_sceneGraph->getAllRoomsInIdOrder(atlasAllRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
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
            wasAdmitted_out = false;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }

    admitCandidateWall();
    wasAdmitted_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
