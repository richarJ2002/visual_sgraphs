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

#include "SemanticsManager.h"

#include "../private_functions.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::suppressUndefendedWalls(void)
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
        undefendedWalls.clear();
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<semantic::Room *> allRooms{};
    if (p_currentMap->getAllRooms(allRooms) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Passage *> allPassages{};
    if (p_currentMap->getAllPassages(allPassages) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> allPlanes{};
    if (p_currentMap->getAllPlanes(allPlanes) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<std::vector<Eigen::Vector3d>> skeletonClusters{};
    if (p_currentMap->getSkeletonClusterPoints(skeletonClusters) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::unordered_set<int> mappedWallIds;

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition). */
    geometric::Plane *p_groundPlaneForEvidence = nullptr;
    if (p_currentMap->getBiggestGroundPlane(p_groundPlaneForEvidence) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    bool            groundPlaneForEvidenceIsBad{};
    if ((p_groundPlaneForEvidence != nullptr) &&
        p_groundPlaneForEvidence->isBad(groundPlaneForEvidenceIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlaneForEvidence != nullptr && !groundPlaneForEvidenceIsBad)
    {
        g2o::Plane3D groundPlaneForEvidenceGetGlobalEquation{};
        if (p_groundPlaneForEvidence->getGlobalEquation(
                groundPlaneForEvidenceGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEq =
            groundPlaneForEvidenceGetGlobalEquation.coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormalForEvidence_World = groundEq.head<3>() / groundNorm;
        }
    }

    for (geometric::Plane *p_wall : allPlanes)
    {
        if (p_wall == nullptr)
        {
            continue;
        }

        int wallId{};
        if (p_wall->getId(wallId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mappedWallIds.insert(wallId);

        bool wallIsBad{};
        if (p_wall->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant wallPlaneType{};
        if (!(wallIsBad) && p_wall->getPlaneType(wallPlaneType) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallIsBad || wallPlaneType != geometric::Plane::PlaneVariant::WALL)
        {
            undefendedWalls.erase(wallId);
            continue;
        }

        const bool ownedByLiveRoom = std::any_of(
            allRooms.begin(),
            allRooms.end(),
            [p_wall](semantic::Room *p_room)
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
                if (p_room == nullptr || roomIsBad)
                {
                    return false;
                }

                std::vector<geometric::Plane *> roomWalls{};
                if (p_room->getWalls(roomWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                return std::find(roomWalls.begin(), roomWalls.end(), p_wall) !=
                       roomWalls.end();
            });
        const bool associatedWithPassage = std::any_of(
            allPassages.begin(),
            allPassages.end(),
            [p_wall](semantic::Passage *p_passage)
            {
                bool passageIsBad{};
                if (!(p_passage == nullptr) &&
                    p_passage->isBad(passageIsBad) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_passage == nullptr || passageIsBad)
                {
                    return false;
                }

                std::vector<geometric::Plane *> passageWalls{};
                if (p_passage->getAssociateWalls(passageWalls) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getAssociateWalls returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                vs_graphs::core::geometric::Plane *p_passageAssociateDoor =
                    nullptr;
                if (p_passage->getAssociateDoor(p_passageAssociateDoor) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getAssociateDoor returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                return p_passageAssociateDoor == p_wall ||
                       std::find(passageWalls.begin(),
                                 passageWalls.end(),
                                 p_wall) != passageWalls.end();
            });
        WallAdmissionEvidence evidence{};
        if (evaluateWallAdmissionEvidence(p_wall,
                                          p_sysParams,
                                          groundNormalForEvidence_World,
                                          evidence) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateWallAdmissionEvidence returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        geometric::Plane::GeometrySnapshot wallGeometry{};
        if (p_wall->getGeometrySnapshot(wallGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
        const double wallNormalNorm = wallEquation_World.head<3>().norm();
        bool         hasCompatibleLiveCluster = false;
        if (wallGeometry.centroid_World_m.allFinite() &&
            wallEquation_World.allFinite() && wallNormalNorm > 1e-8)
        {
            const Eigen::Vector3d wallNormal_World =
                wallEquation_World.head<3>() / wallNormalNorm;
            const double wallOffset_m = wallEquation_World(3) / wallNormalNorm;
            const double maximumCentroidDistance_m =
                2.0 * static_cast<double>(
                          p_sysParams->roomSeg
                              .clusterCentroidWallCentroidDistanceThresh);
            const double maximumPointDistance_m = static_cast<double>(
                p_sysParams->roomSeg.clusterPointWallDistanceThresh);

            for (const std::vector<Eigen::Vector3d> &cluster : skeletonClusters)
            {
                if (cluster.empty())
                {
                    continue;
                }
                Eigen::Vector3d clusterCentroid_World_m{};
                if (utils::utils::Utils::computeCentroidFromPoints(
                        cluster,
                        clusterCentroid_World_m) !=
                    utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    // computeCentroidFromPoints cannot fail; continue as
                    // before.
                }
                if (!clusterCentroid_World_m.allFinite() ||
                    (clusterCentroid_World_m - wallGeometry.centroid_World_m)
                            .norm() > maximumCentroidDistance_m)
                {
                    continue;
                }
                hasCompatibleLiveCluster = std::any_of(
                    cluster.begin(),
                    cluster.end(),
                    [&wallNormal_World, wallOffset_m, maximumPointDistance_m](
                        const Eigen::Vector3d &point)
                    {
                        return point.allFinite() &&
                               std::abs(wallNormal_World.dot(point) +
                                        wallOffset_m) <= maximumPointDistance_m;
                    });
                if (hasCompatibleLiveCluster)
                {
                    break;
                }
            }
        }

        /*!
         * Good geometry alone does NOT defend a wall (user rule: a wall
         * with no room does not get to persist as a wall plane, regardless
         * of how clean its evidence looks). Only an actual owning room or
         * a real passage association counts. A wall genuinely mid-way
         * through bootstrapping a brand-new room is still protected below
         * by the stagnant-cycle counter resetting on cloudGrew/
         * observationGrew -- it keeps gaining points/observations every
         * cycle the UAV still looks at it, so it never goes stagnant long
         * enough to retire while real room-formation is in progress.
         */
        if (ownedByLiveRoom || associatedWithPassage ||
            hasCompatibleLiveCluster)
        {
            undefendedWalls.erase(wallId);
            if (hasCompatibleLiveCluster && !ownedByLiveRoom &&
                !associatedWithPassage)
            {
                unsigned long currentMapId{};
                if (p_currentMap->getId(currentMapId) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                             "\"map_id\":"
                          << currentMapId
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle
                          << ",\"wall_id\":" << wallId
                          << ",\"class\":\"WALL\","
                             "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                             "\"reason\":\"LIVE_CLUSTER_SUPPORT\",\"support\":"
                          << evidence.fittedPointCount
                          << ",\"observations\":" << evidence.observationCount
                          << ",\"pending_age\":0}" << std::endl;
            }
            continue;
        }

        geometric::Plane::GeometrySnapshot wallGetGeometrySnapshot{};
        if (p_wall->getGeometrySnapshot(wallGetGeometrySnapshot) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud =
            wallGetGeometrySnapshot.supportCloud;
        const std::size_t cloudPointCount =
            p_cloud != nullptr ? p_cloud->size() : 0U;
        std::size_t observationCount{};
        if (p_wall->getObservationCount(observationCount) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservationCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        auto [stateIterator, inserted] = undefendedWalls.try_emplace(
            wallId,
            UndefendedWallState{p_wall, 0U, cloudPointCount, observationCount});

        if (!inserted && stateIterator->second.p_wall != p_wall)
        {
            stateIterator->second = UndefendedWallState{p_wall,
                                                        0U,
                                                        cloudPointCount,
                                                        observationCount};
            inserted              = true;
        }

        UndefendedWallState &state = stateIterator->second;

        if (inserted)
        {
            /* The first sighting is recent evidence; age starts next cycle. */
            continue;
        }

        const bool cloudGrew       = cloudPointCount > state.cloudPointCount;
        const bool observationGrew = observationCount > state.observationCount;
        state.cloudPointCount      = cloudPointCount;
        state.observationCount     = observationCount;

        if ((evidence.hasAdequateFiniteFit && cloudGrew) || observationGrew)
        {
            state.unresolvedCycles = 0U;
            continue;
        }

        state.unresolvedCycles++;

        if (state.unresolvedCycles <
            p_sysParams->roomSeg.minimumUndefendedWallHoldCycles)
        {
            unsigned long currentMapId2{};
            if (p_currentMap->getId(currentMapId2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << currentMapId2
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << wallId
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\"GRACE_ACTIVE_NO_GROWTH\","
                         "\"support\":"
                      << evidence.fittedPointCount
                      << ",\"observations\":" << evidence.observationCount
                      << ",\"pending_age\":" << state.unresolvedCycles << "}"
                      << std::endl;
            continue;
        }

        const unsigned int retiredAfterCycles = state.unresolvedCycles;

        /* Relationships were checked above under the semantic transaction. */
        std::map<KeyFrame *, geometric::Plane::Observation> wallObservations{};
        if (p_wall->getObservations(wallObservations) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Sweep every keyframe that references this plane, including
         * those that hold it in mvpMapPlanes without an Observation
         * entry, so no stale pointer survives retirement. */
        std::vector<KeyFrame *> allKeyFrames{};
        if (p_currentMap->getAllKeyFrames(allKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (KeyFrame *p_keyFrame : allKeyFrames)
        {
            if (p_keyFrame != nullptr)
            {
                if (p_keyFrame->removeMapPlane(p_wall) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: removeMapPlane returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        for (const auto &[p_keyFrame, observation] : wallObservations)
        {
            static_cast<void>(observation);
            if (p_keyFrame != nullptr)
            {
                if (p_wall->eraseObservation(p_keyFrame) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseObservation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        if (p_wall->setBad() != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->eraseRoomWallPlane(p_wall) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseRoomWallPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->eraseMapPlane(p_wall) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_wall->p_refKeyFrame = nullptr;
        if (p_wall->setMap(nullptr) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        undefendedWalls.erase(wallId);

        if (loggedRetiredWallIds.insert(wallId).second)
        {
            unsigned long currentMapId3{};
            if (p_currentMap->getId(currentMapId3) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"wall_retirement\","
                         "\"map_id\":"
                      << currentMapId3
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << wallId
                      << ",\"class\":\"WALL\",\"lifecycle\":\"RETIRED\","
                         "\"reason\":\"GRACE_EXPIRED_NO_GROWTH_NO_OWNER_NO_"
                         "PASSAGE_NO_CLUSTER\","
                         "\"support\":"
                      << evidence.fittedPointCount
                      << ",\"observations\":" << evidence.observationCount
                      << ",\"pending_age\":" << retiredAfterCycles << "}"
                      << std::endl;
        }
    }

    for (std::unordered_map<int, UndefendedWallState>::iterator stateIterator =
             undefendedWalls.begin();
         stateIterator != undefendedWalls.end();)
    {
        stateIterator = mappedWallIds.count(stateIterator->first) == 0U
                            ? undefendedWalls.erase(stateIterator)
                            : std::next(stateIterator);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
