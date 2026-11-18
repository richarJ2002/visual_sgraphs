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
 * @file            logPipelineHeartbeat.cc
 *
 * @brief           Implements SemanticsManager::logPipelineHeartbeat(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"
#include "scene_understanding/semantic/semantic_candidates/objects/SemanticCandidatesStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/SemanticGraphSnapshotStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/UnavailableReasonEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/objects/SemanticReportCacheStatusEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/semantic_report_cache.h"
#include "scene_understanding/semantic/sha256_digest/objects/Sha256DigestStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"
#include "scene_understanding/semantic/semantic_candidates/semantic_candidates.h"
#include "scene_understanding/semantic/semantic_canonical_serilization/semantic_canonical_serilization.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"
#include "scene_understanding/semantic/sha256_digest/sha256_digest.h"

#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <chrono>
#include <map>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::logPipelineHeartbeat(void)
{
    /* Compact Phase-1 heartbeat: all values come from this completed
     * semantic transaction and are therefore mutually consistent. */
    map::Map *p_pipelineMap = nullptr;
    if (p_atlas->getCurrentMap(p_pipelineMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> pipelineMapAllPlanes{};
    if ((p_pipelineMap != nullptr) &&
        p_pipelineMap->getAllPlanes(pipelineMapAllPlanes) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::vector<geometric::Plane *> pipelinePlanes =
        p_pipelineMap != nullptr ? pipelineMapAllPlanes
                                 : std::vector<geometric::Plane *>();
    unsigned long pipelineMapId{};
    if ((p_pipelineMap != nullptr) && p_pipelineMap->getId(pipelineMapId) !=
                                          map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> pipelineMapAllRooms{};
    if ((p_pipelineMap != nullptr) &&
        getMapRoomsInIdOrder(pipelineMapId, pipelineMapAllRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    const std::vector<semantic::Room *> pipelineRooms =
        p_pipelineMap != nullptr ? pipelineMapAllRooms
                                 : std::vector<semantic::Room *>();
    std::vector<vs_graphs::core::semantic::Passage *> pipelineMapAllPassages{};
    if ((p_pipelineMap != nullptr) &&
        getMapPassagesInIdOrder(pipelineMapId, pipelineMapAllPassages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPassagesInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    const std::vector<semantic::Passage *> pipelinePassages =
        p_pipelineMap != nullptr ? pipelineMapAllPassages
                                 : std::vector<semantic::Passage *>();
    std::vector<semantic::Floor *> pipelineMapAllFloors{};
    if ((p_pipelineMap != nullptr) &&
        getMapFloorsInIdOrder(pipelineMapId, pipelineMapAllFloors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapFloorsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    const std::vector<semantic::Floor *> pipelineFloors =
        p_pipelineMap != nullptr ? pipelineMapAllFloors
                                 : std::vector<semantic::Floor *>();
    std::vector<std::vector<Eigen::Vector3d>>
        pipelineMapSkeletonClusterPoints{};
    if ((p_pipelineMap != nullptr) && p_pipelineMap->getSkeletonClusterPoints(
                                          pipelineMapSkeletonClusterPoints) !=
                                          map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::vector<std::vector<Eigen::Vector3d>> pipelineClusters =
        p_pipelineMap != nullptr ? pipelineMapSkeletonClusterPoints
                                 : std::vector<std::vector<Eigen::Vector3d>>();

    std::size_t             wallClassCount      = 0U;
    std::size_t             admissibleWallCount = 0U;
    std::size_t             ownedWallCount      = 0U;
    std::unordered_set<int> ownedWallIds;
    geometric::Plane       *p_pipelineMapBiggestGroundPlane = nullptr;
    if ((p_pipelineMap != nullptr) &&
        p_pipelineMap->getBiggestGroundPlane(p_pipelineMapBiggestGroundPlane) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    geometric::Plane *p_pipelineGround =
        p_pipelineMap != nullptr ? p_pipelineMapBiggestGroundPlane : nullptr;
    Eigen::Vector3d pipelineGroundNormal_world = Eigen::Vector3d::Zero();
    bool            pipelineGroundIsBad{};
    if ((p_pipelineGround != nullptr) &&
        p_pipelineGround->isBad(pipelineGroundIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_pipelineGround != nullptr && !pipelineGroundIsBad)
    {
        g2o::Plane3D pipelineGroundGetGlobalEquation{};
        if (p_pipelineGround->getGlobalEquation(
                pipelineGroundGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d equation =
            pipelineGroundGetGlobalEquation.coeffs();
        if (equation.allFinite() && equation.head<3>().norm() > 1e-8)
        {
            pipelineGroundNormal_world = equation.head<3>().normalized();
        }
    }
    for (geometric::Plane *p_plane : pipelinePlanes)
    {
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant planeType{};
        if (!(p_plane == nullptr || planeIsBad) &&
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad ||
            planeType != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }
        wallClassCount++;
        WallAdmissionEvidence admissionEvidence{};
        if (evaluateWallAdmissionEvidence(p_plane,
                                          p_sysParams,
                                          pipelineGroundNormal_world,
                                          admissionEvidence) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateWallAdmissionEvidence returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
        if (admissionEvidence.isAdmissible)
        {
            admissibleWallCount++;
        }
    }
    std::size_t realRoomCount        = 0U;
    std::size_t prospectiveRoomCount = 0U;
    for (semantic::Room *p_room : pipelineRooms)
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
        semantic::Room::RoomVariant roomVariant{};
        if (p_room->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (roomVariant == semantic::Room::RoomVariant::ROOM)
        {
            realRoomCount++;
        }
        else
        {
            prospectiveRoomCount++;
        }
        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
            bool wallIsBad{};
            if ((p_wall != nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall != nullptr && !wallIsBad)
            {
                int wallGetId{};
                if (p_wall->getId(wallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                ownedWallIds.insert(wallGetId);
            }
        }
    }
    ownedWallCount                  = ownedWallIds.size();
    std::size_t skeletonVertexCount = 0U;
    for (const std::vector<Eigen::Vector3d> &cluster : pipelineClusters)
    {
        skeletonVertexCount += cluster.size();
    }
    std::size_t pendingWallCount = 0U;
    for (const std::pair<const int, UndefendedWallState> &undefendedWall :
         undefendedWalls)
    {
        if (ownedWallIds.count(undefendedWall.first) == 0U)
        {
            pendingWallCount++;
        }
    }
    std::size_t livePassageCount = 0U;
    for (semantic::Passage *p_pipelinePassage : pipelinePassages)
    {
        bool passageIsBad{};
        if ((p_pipelinePassage != nullptr) &&
            p_pipelinePassage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_pipelinePassage != nullptr && !passageIsBad)
        {
            livePassageCount++;
        }
    }
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        pipelineMapSkeletonEdges{};
    if ((p_pipelineMap != nullptr) &&
        p_pipelineMap->getSkeletonEdges(pipelineMapSkeletonEdges) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonEdges returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    int getCurrentRoomId2{};
    if (getCurrentRoomId(getCurrentRoomId2) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "SG_PIPELINE {\"event\":\"heartbeat\",\"map_id\":"
              << (p_pipelineMap != nullptr
                      ? static_cast<long long>(pipelineMapId)
                      : -1)
              << ",\"semantic_cycle\":" << pipelineSemanticCycle
              << ",\"current_room_id\":" << getCurrentRoomId2
              << ",\"raw_planes\":" << pipelinePlanes.size()
              << ",\"wall_class_planes\":" << wallClassCount
              << ",\"admissible_walls\":" << admissibleWallCount
              << ",\"pending_walls\":" << pendingWallCount
              << ",\"owned_walls\":" << ownedWallCount
              << ",\"skeleton_clusters\":" << pipelineClusters.size()
              << ",\"skeleton_vertices\":" << skeletonVertexCount
              << ",\"skeleton_edges\":"
              << (p_pipelineMap != nullptr ? pipelineMapSkeletonEdges.size()
                                           : 0U)
              << ",\"real_rooms\":" << realRoomCount
              << ",\"prospective_rooms\":" << prospectiveRoomCount
              << ",\"passages\":" << livePassageCount
              << ",\"floors\":" << pipelineFloors.size() << "}" << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
