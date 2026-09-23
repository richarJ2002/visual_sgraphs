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

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::suppressUndefendedWalls(void)
{
    Map *p_currentMap = p_atlas->getCurrentMap();

    if (p_currentMap == nullptr)
    {
        undefendedWalls_.clear();
        return;
    }

    const std::vector<semantic::Room *> allRooms = p_currentMap->getAllRooms();
    const std::vector<semantic::Passage *> allPassages =
        p_currentMap->getAllPassages();
    const std::vector<geometric::Plane *> allPlanes =
        p_currentMap->getAllPlanes();
    const std::vector<std::vector<Eigen::Vector3d>> skeletonClusters =
        p_currentMap->getSkeletonClusterPoints();
    std::unordered_set<int> mappedWallIds;

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition). */
    geometric::Plane *p_groundPlaneForEvidence =
        p_currentMap->getBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    if (p_groundPlaneForEvidence != nullptr &&
        !p_groundPlaneForEvidence->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlaneForEvidence->getGlobalEquation().coeffs();
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

        const int wallId = p_wall->getId();
        mappedWallIds.insert(wallId);

        if (p_wall->isBad() ||
            p_wall->getPlaneType() != geometric::Plane::PlaneVariant::WALL)
        {
            undefendedWalls_.erase(wallId);
            continue;
        }

        const bool ownedByLiveRoom = std::any_of(
            allRooms.begin(),
            allRooms.end(),
            [p_wall](semantic::Room *p_room)
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    return false;
                }

                const std::vector<geometric::Plane *> roomWalls =
                    p_room->getWalls();
                return std::find(roomWalls.begin(), roomWalls.end(), p_wall) !=
                       roomWalls.end();
            });
        const bool associatedWithPassage =
            std::any_of(allPassages.begin(),
                        allPassages.end(),
                        [p_wall](semantic::Passage *p_passage)
                        {
                            if (p_passage == nullptr || p_passage->isBad())
                            {
                                return false;
                            }

                            const std::vector<geometric::Plane *> passageWalls =
                                p_passage->getAssociateWalls();
                            return p_passage->getAssociateDoor() == p_wall ||
                                   std::find(passageWalls.begin(),
                                             passageWalls.end(),
                                             p_wall) != passageWalls.end();
                        });
        const WallAdmissionEvidence evidence =
            evaluateWallAdmissionEvidence(p_wall,
                                          p_sysParams,
                                          groundNormalForEvidence_World);

        const geometric::Plane::GeometrySnapshot wallGeometry =
            p_wall->getGeometrySnapshot();
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
                const Eigen::Vector3d clusterCentroid_World_m =
                    utils::utils::Utils::computeCentroidFromPoints(cluster);
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
            undefendedWalls_.erase(wallId);
            if (hasCompatibleLiveCluster && !ownedByLiveRoom &&
                !associatedWithPassage)
            {
                std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                             "\"map_id\":"
                          << p_currentMap->getId()
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle_
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

        const auto        p_cloud = p_wall->getGeometrySnapshot().supportCloud;
        const std::size_t cloudPointCount =
            p_cloud != nullptr ? p_cloud->size() : 0U;
        const std::size_t observationCount = p_wall->getObservationCount();
        auto [stateIterator, inserted]     = undefendedWalls_.try_emplace(
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

        if ((evidence.adequateFiniteFit && cloudGrew) || observationGrew)
        {
            state.unresolvedCycles = 0U;
            continue;
        }

        state.unresolvedCycles++;

        if (state.unresolvedCycles <
            p_sysParams->roomSeg.minimumUndefendedWallHoldCycles)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << p_currentMap->getId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
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
        const std::map<KeyFrame *, geometric::Plane::Observation>
            wallObservations = p_wall->getObservations();

        /* Sweep every keyframe that references this plane, including
         * those that hold it in mvpMapPlanes without an Observation
         * entry, so no stale pointer survives retirement. */
        const std::vector<KeyFrame *> allKeyFrames =
            p_currentMap->getAllKeyFrames();
        for (KeyFrame *p_keyFrame : allKeyFrames)
        {
            if (p_keyFrame != nullptr)
            {
                p_keyFrame->removeMapPlane(p_wall);
            }
        }

        for (const auto &[p_keyFrame, observation] : wallObservations)
        {
            static_cast<void>(observation);
            if (p_keyFrame != nullptr)
            {
                p_wall->eraseObservation(p_keyFrame);
            }
        }

        p_wall->setBad();
        p_currentMap->eraseRoomWallPlane(p_wall);
        p_currentMap->eraseMapPlane(p_wall);
        p_wall->p_refKeyFrame = nullptr;
        p_wall->setMap(nullptr);
        undefendedWalls_.erase(wallId);

        if (loggedRetiredWallIds_.insert(wallId).second)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_retirement\","
                         "\"map_id\":"
                      << p_currentMap->getId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
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

    for (auto stateIterator = undefendedWalls_.begin();
         stateIterator != undefendedWalls_.end();)
    {
        stateIterator = mappedWallIds.count(stateIterator->first) == 0U
                            ? undefendedWalls_.erase(stateIterator)
                            : std::next(stateIterator);
    }
}

} // namespace core
} // namespace vs_graphs
