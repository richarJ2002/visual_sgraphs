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
#include <limits>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::detectRoom_FreeSpaceCluster(void)
{
    /* Extract latest skeleton cluster */
    const std::vector<std::vector<Eigen::Vector3d>> clusters =
        partitionFreeSpaceAtPassages(getLatestSkeletonCluster());

    /* If cluster is empty then return */
    if (clusters.empty())
    {
        return;
    }

    /* Extract all planes from the map */
    const std::vector<vs_graphs::core::geometric::Plane *> allPlanes =
        p_atlas->getAllPlanes();

    /* Create a list of all the walls there are in the SGraph */
    std::vector<vs_graphs::core::geometric::Plane *> allWalls;
    allWalls.reserve(allPlanes.size());

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition for why this must be
     * ground-anchored rather than an arbitrary in-plane axis). */
    geometric::Plane *p_groundPlaneForEvidence =
        p_atlas->getBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    bool            groundPlaneForEvidenceIsBad{};
    if ((p_groundPlaneForEvidence != nullptr) &&
        p_groundPlaneForEvidence->isBad(groundPlaneForEvidenceIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    if (p_groundPlaneForEvidence != nullptr && !groundPlaneForEvidenceIsBad)
    {
        g2o::Plane3D groundPlaneForEvidenceGetGlobalEquation{};
        if (p_groundPlaneForEvidence->getGlobalEquation(
                groundPlaneForEvidenceGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getGlobalEquation cannot fail; continue as before.
        }
        const Eigen::Vector4d groundEq =
            groundPlaneForEvidenceGetGlobalEquation.coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormalForEvidence_World = groundEq.head<3>() / groundNorm;
        }
    }

    /* For every plane, extract walls */
    for (vs_graphs::core::geometric::Plane *p_plane : allPlanes)
    {
        /* Skip bad planes */
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        /* Append valid wall planes to list */
        if (evaluateWallAdmissionEvidence(p_plane,
                                          p_sysParams,
                                          groundNormalForEvidence_World)
                .isAdmissible)
        {
            allWalls.push_back(p_plane);
        }
    }

    /* If there are no walls, then return */
    if (allWalls.empty())
    {
        return;
    }

    /* Track room IDs matched in this cycle to avoid double-matching */
    std::unordered_set<int> matchedRoomIds;

    /* Iterate through all the clusters */
    for (std::size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
    {
        /* Extract cluster */
        const std::vector<Eigen::Vector3d> &cluster = clusters[clusterId];

        /* If cluster is empty skip */
        if (cluster.empty())
        {
            continue;
        }

        /* Extract the cluster centroid */
        Eigen::Vector3d clusterCentroid{};
        if (utils::utils::Utils::computeCentroidFromPoints(cluster,
                                                           clusterCentroid) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // computeCentroidFromPoints cannot fail; continue as before.
        }

        /* Initialize a list of planes to track the closest walls */
        std::vector<vs_graphs::core::geometric::Plane *> closestWalls;
        closestWalls.reserve(allWalls.size());

        /* For each wall */
        for (vs_graphs::core::geometric::Plane *wall : allWalls)
        {
            /* Skip wall if it is bad */
            bool wallIsBad{};
            if (!(wall == nullptr) &&
                wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (wall == nullptr || wallIsBad)
            {
                continue;
            }

            /* Extract the point cloud for the wall */
            geometric::Plane::GeometrySnapshot wallGeometry{};
            if (wall->getGeometrySnapshot(wallGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGeometrySnapshot cannot fail; continue as before.
            }
            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
                wallGeometry.supportCloud;

            /* Skip wall if the point cloud is invalid */
            if (p_wallCloud == nullptr || p_wallCloud->empty())
            {
                continue;
            }

            /* Extract the centroid of the wall */
            const Eigen::Vector3d wallCentroid = wallGeometry.centroid_World_m;

            /* Find the distance from the wall centroid and cluster centroid */
            const double centroidDistance =
                (wallCentroid - clusterCentroid).norm();

            /*!
             * Use centroid distance only as a coarse rejection condition.
             *
             * @note        A long wall may have a centroid far from the room
             *              centre while still forming a valid boundary of the
             *              room.
             */
            const double coarseCentroidDistanceThreshold =
                2.0 * static_cast<double>(
                          p_sysParams->roomSeg
                              .clusterCentroidWallCentroidDistanceThresh);

            if (centroidDistance >= coarseCentroidDistanceThreshold)
            {
                continue;
            }

            /* Extract plane equation for the wall */
            g2o::Plane3D wallGetGlobalEquation{};
            if (wall->getGlobalEquation(wallGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector4d wallEquation = wallGetGlobalEquation.coeffs();

            /* Extract the norm of the normal of the wall */
            const double normalMagnitude = wallEquation.head<3>().norm();

            /* Skip wall if norm is invalid */
            if (!std::isfinite(normalMagnitude) || normalMagnitude < 1e-8)
            {
                continue;
            }

            /* Find the normal vector of the wall */
            const Eigen::Vector3d wallNormalVector = wallEquation.head<3>();

            /* Find distance to normal */
            const double wallDistance = wallEquation(3) / normalMagnitude;

            /* Find the normalized vector */
            const Eigen::Vector3d wallNormUnitVector =
                wallNormalVector / normalMagnitude;

            /* Find an axis U which tangental to the wall plane */
            const Eigen::Vector3d axisU =
                wallNormUnitVector.unitOrthogonal().normalized();

            /* Find orthogonal axis to make handed axis with U and normal */
            const Eigen::Vector3d axisV =
                wallNormUnitVector.cross(axisU).normalized();

            /* Measure finite wall bounds in the same basis used below. */
            std::size_t validWallPoints = 0;
            double      minimumWallU_m  = std::numeric_limits<double>::max();
            double      maximumWallU_m  = std::numeric_limits<double>::lowest();
            double      minimumWallV_m  = std::numeric_limits<double>::max();
            double      maximumWallV_m  = std::numeric_limits<double>::lowest();

            /* Iterate through each point in the wall point cloud */
            for (const pcl::PointXYZRGBA &point : p_wallCloud->points)
            {
                /* If point is invalid, skip */
                if (!pcl::isFinite(point))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint_World_m(
                    static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));

                const Eigen::Vector3d wallPointRelToCentroid_World_m =
                    wallPoint_World_m - wallCentroid;

                const double wallPointU_m =
                    wallPointRelToCentroid_World_m.dot(axisU);
                const double wallPointV_m =
                    wallPointRelToCentroid_World_m.dot(axisV);

                minimumWallU_m = std::min(minimumWallU_m, wallPointU_m);
                maximumWallU_m = std::max(maximumWallU_m, wallPointU_m);
                minimumWallV_m = std::min(minimumWallV_m, wallPointV_m);
                maximumWallV_m = std::max(maximumWallV_m, wallPointV_m);

                validWallPoints++;
            }

            /* A finite wall cannot be measured without a valid cloud point. */
            if (validWallPoints == 0)
            {
                continue;
            }

            /* Confirm wall bounds are valid, otherwise skip */
            if (!std::isfinite(minimumWallU_m) ||
                !std::isfinite(maximumWallU_m) ||
                !std::isfinite(minimumWallV_m) ||
                !std::isfinite(maximumWallV_m))
            {
                continue;
            }

            const double wallExtentU_m = maximumWallU_m - minimumWallU_m;
            const double wallExtentV_m = maximumWallV_m - minimumWallV_m;

            /* Skip small fragments without erasing their semantic evidence. */
            if (wallExtentU_m <
                    p_sysParams->roomSeg.minimumFiniteWallExtent_m ||
                wallExtentV_m < p_sysParams->roomSeg.minimumFiniteWallExtent_m)
            {
                continue;
            }

            /*!
             * Number of close points whose projections lie inside the finite
             * wall patch.
             */
            std::size_t supportedPointCount = 0;
            std::size_t nearbyPointCount    = 0;

            /* Init variable to track minimum distance from wall and cluster */
            double minimumPlaneDistance = std::numeric_limits<double>::max();

            /* Iterate through each point in cluster to find point in wall */
            for (const Eigen::Vector3d &clusterPoint : cluster)
            {
                /* Find the signed distance along wall normal*/
                const double signedPlaneDistance =
                    wallNormUnitVector.dot(clusterPoint) + wallDistance;

                /* Find the absolute value of the plane distance */
                const double planeDistance = std::abs(signedPlaneDistance);

                /* Update if the disatance is smaller than currently tracked */
                minimumPlaneDistance =
                    std::min(minimumPlaneDistance, planeDistance);

                /* If the plane distance is larger than threshold, skip point */
                if (planeDistance >=
                    p_sysParams->roomSeg.clusterPointWallDistanceThresh)
                {
                    continue;
                }

                nearbyPointCount++;

                /* Find the projected point on the plane */
                const Eigen::Vector3d projectedPoint =
                    clusterPoint - signedPlaneDistance * wallNormUnitVector;

                /* Find relative distance between point and wall centroid */
                const Eigen::Vector3d projectedRelative =
                    projectedPoint - wallCentroid;

                /* Find distance in U axis on wall */
                const double projectedU = projectedRelative.dot(axisU);

                /* Find distance in V axis on wall */
                const double projectedV = projectedRelative.dot(axisV);

                /* Confirm if the wall encapsulates the projected point */
                const bool insideFiniteWall =
                    projectedU >=
                        minimumWallU_m -
                            p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
                    projectedU <=
                        maximumWallU_m +
                            p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
                    projectedV >=
                        minimumWallV_m -
                            p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
                    projectedV <=
                        maximumWallV_m +
                            p_sysParams->roomSeg.finiteWallBoundsMargin_m;

                /*!
                 * If the point is within the wall plane, incriment counter or
                 * otherwise skip.
                 */
                if (insideFiniteWall)
                {
                    supportedPointCount++;
                }
                else
                {
                    continue;
                }
            }

            /* If the plane distance from cluster is far from threshold, skip */
            if (minimumPlaneDistance >
                p_sysParams->roomSeg.clusterPointWallDistanceThresh)
            {
                continue;
            }

            /* Calculate the fraction of nearby points inside finite bounds. */
            const double finiteSupportRatio =
                nearbyPointCount > 0
                    ? static_cast<double>(supportedPointCount) /
                          static_cast<double>(nearbyPointCount)
                    : 0.0;

            /*!
             * Accept the wall only when there is sufficient absolute support
             * and the majority of nearby points project inside the finite wall
             * patch.
             */

            if (supportedPointCount >=
                    p_sysParams->roomSeg.minimumWallSupportPointCount &&
                finiteSupportRatio >=
                    p_sysParams->roomSeg.minimumWallSupportRatio)
            {
                closestWalls.push_back(wall);
            }
        }

        /* Organise closest walls in order of ids */
        std::sort(closestWalls.begin(),
                  closestWalls.end(),
                  [](vs_graphs::core::geometric::Plane *p_first,
                     vs_graphs::core::geometric::Plane *p_second)
                  {
                      int firstGetId{};
                      if (p_first->getId(firstGetId) !=
                          geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                      {
                          // getId cannot fail; continue as before.
                      }
                      int secondGetId{};
                      if (p_second->getId(secondGetId) !=
                          geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                      {
                          // getId cannot fail; continue as before.
                      }
                      return firstGetId < secondGetId;
                  });

        /* Remove duplicate walls using IDs rather than pointers */
        closestWalls.erase(
            std::unique(closestWalls.begin(),
                        closestWalls.end(),
                        [](vs_graphs::core::geometric::Plane *p_first,
                           vs_graphs::core::geometric::Plane *p_second)
                        {
                            int firstGetId{};
                            if (p_first->getId(firstGetId) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getId cannot fail; continue as before.
                            }
                            int secondGetId{};
                            if (p_second->getId(secondGetId) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getId cannot fail; continue as before.
                            }
                            return firstGetId == secondGetId;
                        }),
            closestWalls.end());

        /* If there are no closest walls then skip to next cluster */
        if (closestWalls.empty())
        {
            continue;
        }

        /*
         * Match external far-side evidence before admitting any new walls.
         * Taking the wall snapshot here prevents admission by this cluster
         * from manufacturing its own promotion evidence.
         */
        vs_graphs::core::semantic::Room    *p_clusterProspective = nullptr;
        vs_graphs::core::semantic::Passage *p_clusterPassage     = nullptr;
        std::vector<vs_graphs::core::geometric::Plane *>
               prospectiveWallsBeforeCluster;
        double bestProspectiveDistance_m =
            std::numeric_limits<double>::infinity();

        for (vs_graphs::core::semantic::Passage *p_passage :
             p_atlas->getAllPassages())
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            vs_graphs::core::semantic::Room *p_prospective = nullptr;
            if (p_passage->getProspectiveRoom(p_prospective) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getProspectiveRoom cannot fail; continue as before.
            }

            bool prospectiveIsBad{};
            if (!(p_prospective == nullptr) &&
                p_prospective->isBad(prospectiveIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            semantic::Room::RoomVariant prospectiveRoomVariant{};
            if (!(p_prospective == nullptr || prospectiveIsBad) &&
                p_prospective->getRoomVariant(prospectiveRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomVariant cannot fail; continue as before.
            }
            int prospectiveId{};
            if (!(p_prospective == nullptr || prospectiveIsBad ||
                  prospectiveRoomVariant != vs_graphs::core::semantic::Room::
                                                RoomVariant::UNDEFINED) &&
                p_prospective->getId(prospectiveId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (p_prospective == nullptr || prospectiveIsBad ||
                prospectiveRoomVariant !=
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
                matchedRoomIds.count(prospectiveId) > 0U)
            {
                continue;
            }

            g2o::Plane3D passageGlobalEquation{};
            if (p_passage->getGlobalEquation(passageGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector4d passageEquation_World =
                passageGlobalEquation.coeffs();
            const double passageNormalNorm =
                passageEquation_World.head<3>().norm();

            if (!passageEquation_World.allFinite() || passageNormalNorm < 1e-8)
            {
                continue;
            }

            passageEquation_World /= passageNormalNorm;

            Eigen::Vector3d prospectiveCentroid_World_m{};
            if (p_prospective->getCentroid(prospectiveCentroid_World_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            const double prospectiveSide_m =
                passageEquation_World.head<3>().dot(
                    prospectiveCentroid_World_m) +
                passageEquation_World(3);
            const double clusterSide_m =
                passageEquation_World.head<3>().dot(clusterCentroid) +
                passageEquation_World(3);
            const double centroidDistance_m =
                (prospectiveCentroid_World_m - clusterCentroid).norm();

            if (!prospectiveCentroid_World_m.allFinite() ||
                prospectiveSide_m * clusterSide_m <= 0.0 ||
                std::abs(prospectiveSide_m) <= 0.20 ||
                std::abs(clusterSide_m) <= 0.20 ||
                centroidDistance_m > kProspectiveDedupDistance_m)
            {
                continue;
            }

            std::vector<vs_graphs::core::geometric::Plane *> prospectiveWalls{};
            if (p_prospective->getWalls(prospectiveWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }
            const bool sharesWallEvidence = std::any_of(
                prospectiveWalls.begin(),
                prospectiveWalls.end(),
                [&closestWalls](
                    vs_graphs::core::geometric::Plane *p_prospectiveWall)
                {
                    bool prospectiveWallIsBad{};
                    if ((p_prospectiveWall != nullptr) &&
                        p_prospectiveWall->isBad(prospectiveWallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    return p_prospectiveWall != nullptr &&
                           !prospectiveWallIsBad &&
                           std::find(closestWalls.begin(),
                                     closestWalls.end(),
                                     p_prospectiveWall) != closestWalls.end();
                });

            if (!sharesWallEvidence ||
                centroidDistance_m >= bestProspectiveDistance_m)
            {
                continue;
            }

            p_clusterProspective          = p_prospective;
            p_clusterPassage              = p_passage;
            prospectiveWallsBeforeCluster = prospectiveWalls;
            bestProspectiveDistance_m     = centroidDistance_m;
        }

        /* Prefer the passage's stable handle; otherwise use normal matching. */
        vs_graphs::core::semantic::Room *p_room =
            p_clusterProspective != nullptr ? p_clusterProspective
                                            : associateRooms(clusterCentroid,
                                                             closestWalls,
                                                             cluster,
                                                             matchedRoomIds);

        /* Track matched room to prevent double-matching in this cycle */
        if (p_room != nullptr)
        {
            int roomId{};
            if (p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            matchedRoomIds.insert(roomId);
        }

        /*!
         * If no existing room describes this free-space cluster.
         * Create one.
         *
         * Do not remove walls that are globally registered. Each mapped wall
         * surface remains owned by one room.
         *
         * Anti-duplicate guard: a wall already claimed by an existing room
         * means this cluster is the same open space (or the boundary-only
         * extension) of that room. Reusing the owner keeps the
         * one-wall-one-room invariant and prevents the classic two-room
         * duplicate derived from two walls of one open space. Only create a
         * fresh candidate when none of the cluster's walls is owned yet.
         */
        if (p_room == nullptr)
        {
            vs_graphs::core::semantic::Room *p_wallOwnerRoom = nullptr;

            const std::vector<vs_graphs::core::semantic::Room *>
                existingRooms_World = p_atlas->getAllRooms();

            for (vs_graphs::core::geometric::Plane *p_candidateWall :
                 closestWalls)
            {
                bool candidateWallIsBad{};
                if (!(p_candidateWall == nullptr) &&
                    p_candidateWall->isBad(candidateWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_candidateWall == nullptr || candidateWallIsBad)
                {
                    continue;
                }

                for (vs_graphs::core::semantic::Room *p_existingRoom :
                     existingRooms_World)
                {
                    bool existingRoomIsBad{};
                    if (!(p_existingRoom == nullptr) &&
                        p_existingRoom->isBad(existingRoomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    int existingRoomId{};
                    if (!(p_existingRoom == nullptr || existingRoomIsBad) &&
                        p_existingRoom->getId(existingRoomId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    if (p_existingRoom == nullptr || existingRoomIsBad ||
                        matchedRoomIds.count(existingRoomId) > 0)
                    {
                        continue;
                    }

                    std::vector<vs_graphs::core::geometric::Plane *>
                        roomWallsList{};
                    if (p_existingRoom->getWalls(roomWallsList) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getWalls cannot fail; continue as before.
                    }

                    if (std::find(roomWallsList.begin(),
                                  roomWallsList.end(),
                                  p_candidateWall) != roomWallsList.end())
                    {
                        p_wallOwnerRoom = p_existingRoom;
                        break;
                    }
                }

                if (p_wallOwnerRoom != nullptr)
                {
                    break;
                }
            }

            if (p_wallOwnerRoom != nullptr)
            {
                p_room = p_wallOwnerRoom;

                int roomId2{};
                if (p_room->getId(roomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemMgr] Reusing existing semantic::Room#"
                          << roomId2 << " for cluster " << clusterId
                          << " (cluster walls already owned)." << std::endl;
            }
        }

        if (p_room == nullptr)
        {
            /*! Axiom: every room after the first must be discovered through
             * a passage (a prospective-room handle, checked above via
             * p_clusterProspective/matched wall ownership), not conjured
             * directly from free-space geometry alone -- "if a wall is
             * observed it must be linked to a room; if that room is new, it
             * must be observed through a passage" (user rule). The
             * exception is the first room -- but per-MAP, not per-mission:
             * every tracking-loss reset starts an entirely new Map with no
             * passages yet either, so it needs its own bootstrap room the
             * same way the mission's very first map did. Atlas::GetAllRooms()
             * spans every map (confirmed by reading it), so scoping this to
             * the CURRENT map only is required -- otherwise a confirmed room
             * surviving in an old, now-inactive map would permanently block
             * every future map from ever bootstrapping its own first room. */
            Map *p_currentMapForBootstrapCheck = p_atlas->getCurrentMap();
            const std::vector<semantic::Room *> currentMapRooms =
                p_currentMapForBootstrapCheck != nullptr
                    ? p_currentMapForBootstrapCheck->getAllRooms()
                    : std::vector<semantic::Room *>();
            const bool anyConfirmedRoomExistsInCurrentMap = std::any_of(
                currentMapRooms.begin(),
                currentMapRooms.end(),
                [](vs_graphs::core::semantic::Room *p_existingRoom)
                {
                    bool existingRoomIsBad{};
                    if ((p_existingRoom != nullptr) &&
                        p_existingRoom->isBad(existingRoomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    semantic::Room::RoomVariant existingRoomRoomVariant{};
                    if ((p_existingRoom != nullptr && !existingRoomIsBad) &&
                        p_existingRoom->getRoomVariant(
                            existingRoomRoomVariant) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getRoomVariant cannot fail; continue as before.
                    }
                    return p_existingRoom != nullptr && !existingRoomIsBad &&
                           existingRoomRoomVariant ==
                               vs_graphs::core::semantic::Room::RoomVariant::
                                   ROOM;
                });

            if (anyConfirmedRoomExistsInCurrentMap)
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " matches no existing room, wall owner, or "
                             "passage-linked prospective room; deferring "
                             "(not the first room of this map, so it must be "
                             "discovered through a passage, not created from "
                             "geometry alone)."
                          << std::endl;
                continue;
            }

            /* Reset transient: the active map was just cleared (no live ROOM)
             * but a last-known hierarchy exists. Bootstrap owns first-room
             * creation with the stable ID; free-space must not conjure a
             * fresh SE# from stale cross-map walls/clusters in this cycle. */
            const int pendingRecoveryRoomId =
                p_atlas != nullptr ? p_atlas->getCurrentSemanticRoomIdentity()
                                   : -1;
            if (pendingRecoveryRoomId >= 0 &&
                p_atlas->copyLatestRoomContext(pendingRecoveryRoomId)
                    .has_value())
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " deferred: recovery semantic::Room#"
                          << pendingRecoveryRoomId
                          << " owns first-room creation on this map."
                          << std::endl;
                continue;
            }

            /*! First-room creation lives in the bootstrap hierarchy, which
             * runs before free-space detection every cycle and promotes its
             * room immediately: a cluster that matches no room, wall owner,
             * or passage-linked prospective at this point describes no known
             * space, so it is deferred rather than conjured into a duplicate
             * first room. */
            std::cout << "[SemMgr] Cluster " << clusterId
                      << " deferred: first-room creation is owned by "
                         "bootstrap; cluster matches no known space."
                      << std::endl;
            continue;
        }

        /* The room centre is owned by its walls, not by free space: rooms
         * keep their creation centroid until walls arrive, then track the
         * damped wall-centroid mean in the consolidation below. Overwriting
         * from the cluster centroid every cycle drags the centre toward
         * whichever free space was observed last (live-observed: across a
         * passage onto its far side) and couples maintenance to Voxblox
         * liveness. Free-space evidence places a room once, at creation. */

        const std::vector<vs_graphs::core::semantic::Passage *> activePassages =
            p_atlas->getAllPassages();
        const bool roomIsPassageBoundProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [p_room](vs_graphs::core::semantic::Passage *p_passage)
            {
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                    nullptr;
                if ((p_passage != nullptr) &&
                    p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getProspectiveRoom cannot fail; continue as before.
                }
                semantic::Room::RoomVariant roomVariant{};
                if ((p_passage != nullptr &&
                     p_passageProspectiveRoom == p_room) &&
                    p_room->getRoomVariant(roomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getRoomVariant cannot fail; continue as before.
                }
                return p_passage != nullptr &&
                       p_passageProspectiveRoom == p_room &&
                       roomVariant == vs_graphs::core::semantic::Room::
                                          RoomVariant::UNDEFINED;
            });

        /* A prospective must first pass the external cluster validation below;
         * generic consolidation must not classify or replace it early. */
        if (!roomIsPassageBoundProspective)
        {
            consolidateRoomsInFreeSpaceCluster(p_room, cluster, allWalls);
        }

        /* Find the walls of the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

        /*! Perspective guard: a wall observed through an opening is on the far
         * side of the passage's supporting wall and therefore cannot bound the
         * near room. Bind such walls to the passage's prospective room instead,
         * where they can later be matched by independently validated far-side
         * cluster evidence. Keyed by the room centroid so the near side stays
         * stable. When no prospective exists yet, the wall is left off the near
         * room so it can bind onto a prospective created later in the same
         * cycle. The crossing geometry alone drives the far-side decision,
         * independent of the passage's passability state.
         */
        for (vs_graphs::core::geometric::Plane *wall : closestWalls)
        {
            /* Skip invalid walls */
            bool wallIsBad2{};
            if (!(wall == nullptr) &&
                wall->isBad(wallIsBad2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (wall == nullptr || wallIsBad2)
            {
                continue;
            }

            /*! Reroute far-side walls to the passage's prospective room.
             * The ground normal is needed to project the aperture crossing. */
            bool farSideBound = false;
            {
                geometric::Plane *p_groundPlane =
                    p_atlas->getBiggestGroundPlane();
                Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();
                bool            groundPlaneIsBad{};
                if ((p_groundPlane != nullptr) &&
                    p_groundPlane->isBad(groundPlaneIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_groundPlane != nullptr && !groundPlaneIsBad)
                {
                    g2o::Plane3D groundPlaneGetGlobalEquation{};
                    if (p_groundPlane->getGlobalEquation(
                            groundPlaneGetGlobalEquation) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getGlobalEquation cannot fail; continue as before.
                    }
                    const Eigen::Vector4d groundEq =
                        groundPlaneGetGlobalEquation.coeffs();
                    const double groundNorm = groundEq.head<3>().norm();
                    if (groundEq.allFinite() && groundNorm > 1e-8)
                    {
                        groundNormal_World = groundEq.head<3>() / groundNorm;
                    }
                }

                for (semantic::Passage *p_passage : p_atlas->getAllPassages())
                {
                    if (p_passage == nullptr)
                    {
                        continue;
                    }

                    /* The aperture geometry alone decides the far side; the
                     * passage need not yet be passable or have a prospective.
                     */
                    Eigen::Vector3d roomCentroid{};
                    if (p_room->getCentroid(roomCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getCentroid cannot fail; continue as before.
                    }
                    Eigen::Vector3d wallGetCentroid{};
                    if (wall->getCentroid(wallGetCentroid) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getCentroid cannot fail; continue as before.
                    }
                    if (!segmentCrossesPassageOpening(
                            roomCentroid,
                            wallGetCentroid.cast<double>(),
                            p_passage,
                            groundNormal_World,
                            static_cast<double>(
                                p_sysParams->roomSeg.passagePartition
                                    .openingMargin_m),
                            static_cast<double>(
                                p_sysParams->roomSeg.passagePartition
                                    .minimumSideDistance_m),
                            false))
                    {
                        continue;
                    }

                    /* The wall lies beyond the opening: it belongs to the far
                     * room, which the free-space clustering will create and
                     * claim. The wall is held on the passage's prospective room
                     * so it keeps accumulating walls and is not re-absorbed by
                     * the near room, nor stolen by a confirmed room. Only
                     * divert a wall that is still unowned or owned by the near
                     * room, so a wall already bound to a distinct confirmed
                     * room is never stolen. */
                    bool ownedByConfirmedRoom = false;
                    for (vs_graphs::core::semantic::Room *p_other :
                         p_atlas->getAllRooms())
                    {
                        bool otherIsBad{};
                        if (!(p_other == nullptr) &&
                            p_other->isBad(otherIsBad) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // isBad cannot fail; continue as before.
                        }
                        semantic::Room::RoomVariant otherRoomVariant{};
                        if (!(p_other == nullptr || otherIsBad ||
                              p_other == p_room) &&
                            p_other->getRoomVariant(otherRoomVariant) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // getRoomVariant cannot fail; continue as before.
                        }
                        if (p_other == nullptr || otherIsBad ||
                            p_other == p_room ||
                            otherRoomVariant ==
                                vs_graphs::core::semantic::Room::RoomVariant::
                                    UNDEFINED)
                        {
                            continue;
                        }
                        std::vector<geometric::Plane *> otherWalls{};
                        if (p_other->getWalls(otherWalls) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // getWalls cannot fail; continue as before.
                        }
                        if (std::find(otherWalls.begin(),
                                      otherWalls.end(),
                                      wall) != otherWalls.end())
                        {
                            ownedByConfirmedRoom = true;
                            break;
                        }
                    }
                    if (ownedByConfirmedRoom)
                    {
                        break;
                    }

                    vs_graphs::core::semantic::Room *p_prospectiveRoom =
                        nullptr;
                    if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getProspectiveRoom cannot fail; continue as before.
                    }

                    bool prospectiveRoomIsBad{};
                    if (!(p_prospectiveRoom == nullptr) &&
                        p_prospectiveRoom->isBad(prospectiveRoomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    if (p_prospectiveRoom == nullptr || prospectiveRoomIsBad)
                    {
                        /* The far-side room does not exist yet (it may be
                         * created later in this cycle by
                         * associatePassagesToRooms). Keep the wall off the
                         * near room so it can bind onto the prospective. */
                        bool roomWasWallRemoved{};
                        if (p_room->removeWall(wall, roomWasWallRemoved) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasWallRemoved =
                                false; // rejected input reads as before
                        }
                        int wallGetId{};
                        if (wall->getId(wallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        if (p_atlas->getRoomWallPlaneById(wallGetId) == nullptr)
                        {
                            p_atlas->addRoomWallPlane(wall);
                        }
                        int passageId{};
                        if (p_passage->getId(passageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        int wallGetId2{};
                        if (wall->getId(wallGetId2) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        std::cout
                            << "[SemMgr] Far-side Wall#" << wallGetId2
                            << " at semantic::Passage#" << passageId
                            << " has no prospective yet; held unbound for the "
                               "far-side room."
                            << std::endl;
                        farSideBound = true;
                        break;
                    }

                    bool roomWasWallRemoved2{};
                    if (p_room->removeWall(wall, roomWasWallRemoved2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        roomWasWallRemoved2 =
                            false; // rejected input reads as before
                    }
                    int wallGetId3{};
                    if (wall->getId(wallGetId3) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    if (p_atlas->getRoomWallPlaneById(wallGetId3) == nullptr)
                    {
                        p_atlas->addRoomWallPlane(wall);
                    }
                    admitWallToRoom(p_prospectiveRoom, wall);
                    farSideBound = true;
                    break;
                }

                /* No CONFIRMED passage's aperture caught this wall. That does
                 * not mean no opening exists here -- confirmation requires
                 * several genuinely independent skeleton snapshots
                 * (minimumConfirmationSnapshots) and therefore real elapsed
                 * exploration time, so a real, already skeleton-evidenced
                 * opening can sit here well before it earns a Passage
                 * object. Falling through to ordinary admission for that
                 * entire window is exactly the bug: a wall on the far side
                 * of a genuine (if not yet confirmed) doorway gets bound to
                 * the WRONG (near) room. Re-run the same aperture test
                 * against each pending hypothesis's own evidence (its
                 * supporting wall's plane + accumulated opening size) --
                 * still no prospective room can exist yet (that requires a
                 * confirmed Passage), so the only action available is the
                 * same conservative one already used above for a confirmed
                 * passage with no prospective yet: hold the wall off the
                 * near room rather than admit it anywhere. */
                if (!farSideBound)
                {
                    for (OpenPassageEvidence &evidence : openPassageEvidence)
                    {
                        Eigen::Vector3d roomCentroid2{};
                        if (p_room->getCentroid(roomCentroid2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // getCentroid cannot fail; continue as before.
                        }
                        Eigen::Vector3d wallGetCentroid2{};
                        if (wall->getCentroid(wallGetCentroid2) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getCentroid cannot fail; continue as before.
                        }
                        if (!segmentCrossesOpenPassageEvidence(
                                roomCentroid2,
                                wallGetCentroid2.cast<double>(),
                                evidence.p_supportingWall,
                                evidence.centroid_World_m,
                                evidence.openingRadius_m,
                                evidence.heightSpan_m,
                                groundNormal_World,
                                static_cast<double>(
                                    p_sysParams->roomSeg.passagePartition
                                        .openingMargin_m),
                                static_cast<double>(
                                    p_sysParams->roomSeg.passagePartition
                                        .minimumSideDistance_m)))
                        {
                            continue;
                        }

                        bool roomWasWallRemoved3{};
                        if (p_room->removeWall(wall, roomWasWallRemoved3) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasWallRemoved3 =
                                false; // rejected input reads as before
                        }
                        int wallGetId4{};
                        if (wall->getId(wallGetId4) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        if (p_atlas->getRoomWallPlaneById(wallGetId4) ==
                            nullptr)
                        {
                            p_atlas->addRoomWallPlane(wall);
                        }
                        int wallGetId5{};
                        if (wall->getId(wallGetId5) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        int getId2{};
                        if ((evidence.p_supportingWall != nullptr) &&
                            evidence.p_supportingWall->getId(getId2) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getId cannot fail; continue as before.
                        }
                        std::cout
                            << "[SemMgr] Far-side Wall#" << wallGetId5
                            << " crosses an unconfirmed passage opening "
                               "(evidence at wall "
                            << (evidence.p_supportingWall != nullptr ? getId2
                                                                     : -1)
                            << "); held unbound pending confirmation."
                            << std::endl;
                        farSideBound = true;
                        break;
                    }
                }
            }

            if (farSideBound)
            {
                continue;
            }

            /* Check to see if closest wall is in any of the current rooms */
            const bool alreadyInRoom = std::any_of(
                roomWalls.begin(),
                roomWalls.end(),
                [wall](vs_graphs::core::geometric::Plane *p_existingWall)
                {
                    int existingWallGetId{};
                    if ((p_existingWall != nullptr) &&
                        p_existingWall->getId(existingWallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int wallGetId{};
                    if ((p_existingWall != nullptr) &&
                        wall->getId(wallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    return p_existingWall != nullptr &&
                           existingWallGetId == wallGetId;
                });

            /* If the wall is already in a room, skip to next slosest wall */
            if (alreadyInRoom)
            {
                continue;
            }

            /*
             * A mapped wall surface has one room owner. A room detected on
             * the opposite side must be bounded by its independently observed
             * wall surface rather than sharing this plane object.
             */
            const std::vector<vs_graphs::core::semantic::Room *> mappedRooms =
                p_atlas->getAllRooms();

            const auto existingOwnerIterator = std::find_if(
                mappedRooms.begin(),
                mappedRooms.end(),
                [p_room, wall](vs_graphs::core::semantic::Room *p_otherRoom)
                {
                    bool otherRoomIsBad{};
                    if (!(p_otherRoom == nullptr || p_otherRoom == p_room) &&
                        p_otherRoom->isBad(otherRoomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    if (p_otherRoom == nullptr || p_otherRoom == p_room ||
                        otherRoomIsBad)
                    {
                        return false;
                    }

                    std::vector<vs_graphs::core::geometric::Plane *>
                        otherRoomWalls{};
                    if (p_otherRoom->getWalls(otherRoomWalls) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getWalls cannot fail; continue as before.
                    }

                    return std::find(otherRoomWalls.begin(),
                                     otherRoomWalls.end(),
                                     wall) != otherRoomWalls.end();
                });

            semantic::Room *p_existingWallOwner =
                existingOwnerIterator != mappedRooms.end()
                    ? *existingOwnerIterator
                    : nullptr;
            bool               existingOwnerIsTransferableProvisional = false;
            semantic::Passage *p_transferPassage                      = nullptr;

            if (p_existingWallOwner != nullptr)
            {
                g2o::Plane3D wallGetGlobalEquation2{};
                if (wall->getGlobalEquation(wallGetGlobalEquation2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                Eigen::Vector4d wallEquation_World =
                    wallGetGlobalEquation2.coeffs();
                const double wallNormalNorm =
                    wallEquation_World.head<3>().norm();

                if (wallEquation_World.allFinite() && wallNormalNorm > 1e-8)
                {
                    wallEquation_World /= wallNormalNorm;

                    constexpr double maximumProvisionalPlaneDistance_m = 0.20;
                    Eigen::Vector3d  existingWallOwnerCentroid{};
                    if (p_existingWallOwner->getCentroid(
                            existingWallOwnerCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getCentroid cannot fail; continue as before.
                    }
                    const double ownerPlaneDistance_m =
                        std::abs(wallEquation_World.head<3>().dot(
                                     existingWallOwnerCentroid) +
                                 wallEquation_World(3));

                    semantic::Room::RoomVariant existingWallOwnerRoomVariant{};
                    if (p_existingWallOwner->getRoomVariant(
                            existingWallOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getRoomVariant cannot fail; continue as before.
                    }
                    std::vector<geometric::Plane *> existingWallOwnerWalls{};
                    if ((existingWallOwnerRoomVariant ==
                         semantic::Room::RoomVariant::UNDEFINED) &&
                        p_existingWallOwner->getWalls(existingWallOwnerWalls) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getWalls cannot fail; continue as before.
                    }
                    existingOwnerIsTransferableProvisional =
                        existingWallOwnerRoomVariant ==
                            semantic::Room::RoomVariant::UNDEFINED &&
                        existingWallOwnerWalls.size() == 1U &&
                        ownerPlaneDistance_m <=
                            maximumProvisionalPlaneDistance_m;
                }

                /*
                 * A wall initially assigned before passage partitioning may
                 * belong to the room on the far side. Transfer it only when
                 * the confirmed opening separates both room centres and the
                 * wall's observing cameras lie on the candidate-room side.
                 */
                geometric::Plane *p_groundPlane =
                    p_atlas->getBiggestGroundPlane();
                Eigen::Vector3d meanObservationPosition_World_m =
                    Eigen::Vector3d::Zero();
                std::size_t validObservationCount = 0U;

                std::map<core::KeyFrame *, geometric::Plane::Observation>
                    wallGetObservations{};
                if (wall->getObservations(wallGetObservations) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getObservations cannot fail; continue as before.
                }
                for (const auto &[p_keyFrame, observation] :
                     wallGetObservations)
                {
                    static_cast<void>(observation);

                    if (p_keyFrame == nullptr || p_keyFrame->isBad())
                    {
                        continue;
                    }

                    const Eigen::Vector3d cameraCentre_World_m =
                        p_keyFrame->getCameraCenter().cast<double>();

                    if (cameraCentre_World_m.allFinite())
                    {
                        meanObservationPosition_World_m += cameraCentre_World_m;
                        validObservationCount++;
                    }
                }

                bool groundPlaneIsBad2{};
                if ((!existingOwnerIsTransferableProvisional &&
                     validObservationCount > 0U && p_groundPlane != nullptr) &&
                    p_groundPlane->isBad(groundPlaneIsBad2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (!existingOwnerIsTransferableProvisional &&
                    validObservationCount > 0U && p_groundPlane != nullptr &&
                    !groundPlaneIsBad2)
                {
                    meanObservationPosition_World_m /=
                        static_cast<double>(validObservationCount);

                    g2o::Plane3D groundPlaneGetGlobalEquation2{};
                    if (p_groundPlane->getGlobalEquation(
                            groundPlaneGetGlobalEquation2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getGlobalEquation cannot fail; continue as before.
                    }
                    Eigen::Vector4d groundEquation_World =
                        groundPlaneGetGlobalEquation2.coeffs();
                    const double groundNormalNorm =
                        groundEquation_World.head<3>().norm();

                    if (groundEquation_World.allFinite() &&
                        groundNormalNorm > 1e-8)
                    {
                        const Eigen::Vector3d groundNormal_World =
                            groundEquation_World.head<3>() / groundNormalNorm;

                        for (semantic::Passage *p_passage :
                             p_atlas->getAllPassages())
                        {
                            Eigen::Vector3d existingWallOwnerCentroid2{};
                            if (p_existingWallOwner->getCentroid(
                                    existingWallOwnerCentroid2) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                // getCentroid cannot fail; continue as before.
                            }
                            if (!segmentCrossesPassageOpening(
                                    existingWallOwnerCentroid2,
                                    clusterCentroid,
                                    p_passage,
                                    groundNormal_World,
                                    p_sysParams->roomSeg.passagePartition
                                        .openingMargin_m,
                                    p_sysParams->roomSeg.passagePartition
                                        .minimumSideDistance_m))
                            {
                                continue;
                            }

                            g2o::Plane3D passageGlobalEquation2{};
                            if (p_passage->getGlobalEquation(
                                    passageGlobalEquation2) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                // getGlobalEquation cannot fail; continue as
                                // before.
                            }
                            Eigen::Vector4d passageEquation_World =
                                passageGlobalEquation2.coeffs();
                            const double passageNormalNorm =
                                passageEquation_World.head<3>().norm();

                            if (!passageEquation_World.allFinite() ||
                                passageNormalNorm < 1e-8)
                            {
                                continue;
                            }

                            passageEquation_World /= passageNormalNorm;
                            const double candidateSide_m =
                                passageEquation_World.head<3>().dot(
                                    clusterCentroid) +
                                passageEquation_World(3);
                            const double observationSide_m =
                                passageEquation_World.head<3>().dot(
                                    meanObservationPosition_World_m) +
                                passageEquation_World(3);

                            constexpr double minimumEvidenceSideDistance_m =
                                0.10;

                            if (candidateSide_m * observationSide_m > 0.0 &&
                                std::abs(candidateSide_m) >=
                                    minimumEvidenceSideDistance_m &&
                                std::abs(observationSide_m) >=
                                    minimumEvidenceSideDistance_m)
                            {
                                p_transferPassage = p_passage;
                                break;
                            }
                        }
                    }
                }

                if (!existingOwnerIsTransferableProvisional)
                {
                    /* Never transfer a wall already owned by a confirmed room
                     * across a confirmed passage: a wall on the far side of an
                     * opening cannot bound the candidate room. Only a
                     * provisional single-wall structural element may hand its
                     * wall to the room that actually observes it. */
                    continue;
                }
            }

            /* Reject wall hypotheses which would corrupt this boundary. */
            if (!admitWallToRoom(p_room, wall))
            {
                continue;
            }

            bool existingWallOwnerWasWallRemoved{};
            if ((p_existingWallOwner != nullptr) &&
                p_existingWallOwner->removeWall(
                    wall,
                    existingWallOwnerWasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                existingWallOwnerWasWallRemoved =
                    false; // rejected input reads as before
            }
            if (p_existingWallOwner != nullptr &&
                existingWallOwnerWasWallRemoved)
            {
                if (existingOwnerIsTransferableProvisional)
                {
                    Map *p_currentMap = p_atlas->getCurrentMap();

                    if (p_currentMap != nullptr)
                    {
                        p_currentMap->eraseDetectedMapRoom(p_existingWallOwner);
                        p_currentMap->eraseMarkerBasedMapRoom(
                            p_existingWallOwner);
                    }

                    if (p_existingWallOwner->setBad() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // setBad cannot fail; continue as before.
                    }

                    int existingWallOwnerId{};
                    if (p_existingWallOwner->getId(existingWallOwnerId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int roomId3{};
                    if (p_room->getId(roomId3) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int wallGetId6{};
                    if (wall->getId(wallGetId6) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    std::cout << "[SemMgr] Transferred orphan Wall#"
                              << wallGetId6 << " from provisional SE#"
                              << existingWallOwnerId << " to semantic::Room#"
                              << roomId3 << "." << std::endl;
                }
                else
                {
                    int existingWallOwnerId2{};
                    if (p_existingWallOwner->getId(existingWallOwnerId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int roomId4{};
                    if (p_room->getId(roomId4) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int transferPassageId{};
                    if (p_transferPassage->getId(transferPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int wallGetId7{};
                    if (wall->getId(wallGetId7) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    std::cout
                        << "[SemMgr] Transferred Wall#" << wallGetId7
                        << " from semantic::Room#" << existingWallOwnerId2
                        << " to semantic::Room#" << roomId4
                        << " through semantic::Passage#" << transferPassageId
                        << " using wall-observation evidence." << std::endl;
                }
            }

            std::vector<geometric::Plane *> roomWalls2{};
            if (p_room->getWalls(roomWalls2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }
            roomWalls = roomWalls2;

            /* Register the uniquely owned room-wall surface. */
            int wallGetId8{};
            if (wall->getId(wallGetId8) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (p_atlas->getRoomWallPlaneById(wallGetId8) == nullptr)
            {
                p_atlas->addRoomWallPlane(wall);
            }
        }

        /* Find all the walls in a room */
        std::vector<geometric::Plane *> roomWalls3{};
        if (p_room->getWalls(roomWalls3) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        roomWalls = roomWalls3;

        /*!
         * Consolidate provisional single-wall structural elements whose wall
         * has now been absorbed into the cluster-backed room.
         *
         * @note        This is deliberately more restrictive than the old
         *              centroid-only reAssociateRooms() implementation.
         */
        if (utils::utils::Utils::consolidateProvisionalRooms(p_room, p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // consolidateProvisionalRooms cannot fail; continue as before.
        }

        /* Remove invalid relationships from the room's persistent graph. */
        std::size_t roomRemovedWallCount{};
        if (p_room->removeInvalidWalls(roomRemovedWallCount) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // removeInvalidWalls cannot fail; continue as before.
        }
        std::vector<geometric::Plane *> roomWalls4{};
        if (p_room->getWalls(roomWalls4) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        roomWalls = roomWalls4;

        /*!
         * The semantic room centre is the mean of each wall's centroid
         * nudged INWARD along that wall's own room-facing normal
         * (Room::getWallNormalTowardRoom_World(), oriented against the
         * room's own current centroid before this update replaces it) by
         * a fixed offset, not the raw wall centroids themselves. A plain
         * mean of wall centroids is NOT guaranteed to land inside the room:
         * for a room only partially observed so far (e.g. two adjacent
         * walls, no opposite pair yet), the raw mean sits near the shared
         * corner, which can be right on -- or, depending on geometry,
         * outside -- the room's true interior. Nudging each wall centroid
         * inward before averaging keeps every contributing point already
         * inside the room, so their mean is too. Rooms without walls keep
         * their creation centroid until walls arrive: free-space evidence
         * places a room once, at creation, and never maintains it.
         */
        if (!roomWalls.empty())
        {
            constexpr double centroidInwardOffset_m   = 0.10;
            Eigen::Vector3d  wallMeanCentroid_World_m = Eigen::Vector3d::Zero();
            std::size_t      validWallCount           = 0U;

            for (vs_graphs::core::geometric::Plane *p_roomWall : roomWalls)
            {
                bool roomWallIsBad{};
                if (!(p_roomWall == nullptr) &&
                    p_roomWall->isBad(roomWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_roomWall == nullptr || roomWallIsBad)
                {
                    continue;
                }

                Eigen::Vector3d roomWallGetCentroid{};
                if (p_roomWall->getCentroid(roomWallGetCentroid) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getCentroid cannot fail; continue as before.
                }
                const Eigen::Vector3d wallCentroid_World_m =
                    roomWallGetCentroid.cast<double>();

                if (!wallCentroid_World_m.allFinite())
                {
                    continue;
                }

                std::optional<Eigen::Vector3d> inwardNormal_World{};
                if (p_room->getWallNormalTowardRoom_World(p_roomWall,
                                                          inwardNormal_World) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWallNormalTowardRoom_World cannot fail; continue as
                    // before.
                }

                const Eigen::Vector3d nudgedCentroid_World_m =
                    inwardNormal_World
                        ? wallCentroid_World_m +
                              centroidInwardOffset_m * (*inwardNormal_World)
                        : wallCentroid_World_m;

                wallMeanCentroid_World_m += nudgedCentroid_World_m;
                validWallCount++;
            }

            if (validWallCount > 0U)
            {
                const Eigen::Vector3d correctedCentroid_World_m =
                    wallMeanCentroid_World_m /
                    static_cast<double>(validWallCount);

                /*!
                 * Damped update, not a snap. This centroid feeds two
                 * decisions that REMOVE walls -- isWallFaceForeignToRoom's
                 * foreign-face check (enforcePassageSideInvariant) and the
                 * passage far-side router -- which change roomWalls, which
                 * changes the raw mean computed above. An undamped snap
                 * closes an unstable feedback loop with no damping: remove
                 * a wall -> centroid shifts -> another wall's side test
                 * flips -> remove that too -> centroid shifts further.
                 * Live-observed 2026-09-04 crossing Office 6's passage: the
                 * same handful of walls near the doorway repeatedly
                 * removed and re-admitted, tens of times in a row, a
                 * previously COMPLETE room's boundary never settling.
                 * Blending in a minority weight of the fresh mean still
                 * lets the centroid track genuinely new evidence over
                 * several cycles, but one cycle's wall churn can no longer
                 * swing it far enough to flip another wall's side test.
                 */
                Eigen::Vector3d previousCentroid_World_m{};
                if (p_room->getCentroid(previousCentroid_World_m) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getCentroid cannot fail; continue as before.
                }
                constexpr double      centroidDampingWeight = 0.25;
                const Eigen::Vector3d dampedCentroid_World_m =
                    previousCentroid_World_m.allFinite()
                        ? (centroidDampingWeight * correctedCentroid_World_m +
                           (1.0 - centroidDampingWeight) *
                               previousCentroid_World_m)
                        : correctedCentroid_World_m;

                if (p_room->setCentroid(dampedCentroid_World_m) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setCentroid cannot fail; continue as before.
                }
            }
        }

        /*!
         * Confirm the room from its free-space cluster.
         *
         * @note        A room is defined by connected free space, not by having
         *              a particular arrangement or number of walls. The
         *              associated walls describe the room boundary but do not
         *              define whether the free-space region is a room.
         */
        const bool validFreeSpaceCluster =
            cluster.size() >=
            static_cast<std::size_t>(p_sysParams->roomSeg.minClusterVertices);

        /*!
         * Require at least one associated wall before inserting the free-space
         * cluster into the structural hierarchy.
         */
        const bool hasBoundaryEvidence = !roomWalls.empty();

        const bool prospectiveWallEvidenceStillMatches =
            p_clusterProspective == p_room && p_clusterPassage != nullptr &&
            std::any_of(
                prospectiveWallsBeforeCluster.begin(),
                prospectiveWallsBeforeCluster.end(),
                [&closestWalls,
                 &roomWalls](vs_graphs::core::geometric::Plane *p_wall)
                {
                    bool wallIsBad{};
                    if ((p_wall != nullptr) &&
                        p_wall->isBad(wallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    return p_wall != nullptr && !wallIsBad &&
                           std::find(closestWalls.begin(),
                                     closestWalls.end(),
                                     p_wall) != closestWalls.end() &&
                           std::find(roomWalls.begin(),
                                     roomWalls.end(),
                                     p_wall) != roomWalls.end();
                });

        /* Confirm the cluster-backed structural element as a room. */
        semantic::Room::RoomVariant roomVariant{};
        if ((validFreeSpaceCluster && hasBoundaryEvidence) &&
            p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        if (validFreeSpaceCluster && hasBoundaryEvidence &&
            roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            if (prospectiveWallEvidenceStillMatches)
            {
                Map *p_roomMap = nullptr;
                if (p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getMap cannot fail; continue as before.
                }
                if (p_roomMap != nullptr)
                {
                    p_roomMap->promoteCandidateMapRoom(p_room);
                }

                if (p_room->setRoomVariant(
                        vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setRoomVariant cannot fail; continue as before.
                }
                int roomId5{};
                if (p_room->getId(roomId5) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                if (p_room->setName("semantic::Room#" +
                                    std::to_string(roomId5)) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setName cannot fail; continue as before.
                }
                int roomId6{};
                if (p_room->getId(roomId6) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                prospectiveRoomCycles.erase(roomId6);

                if (p_room->setDoorways(p_clusterPassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setDoorways cannot fail; continue as before.
                }
                if (p_clusterPassage->setProspectiveRoom(p_room) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setProspectiveRoom cannot fail; continue as before.
                }

                int roomId7{};
                if (p_room->getId(roomId7) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemMgr] Promoted prospective semantic::Room#"
                          << roomId7
                          << " to ROOM from far-side cluster evidence."
                          << std::endl;
            }
            else if (!roomIsPassageBoundProspective)
            {
                Map *p_roomMap = nullptr;
                if (p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getMap cannot fail; continue as before.
                }
                if (p_roomMap != nullptr)
                {
                    p_roomMap->promoteCandidateMapRoom(p_room);
                }
                if (p_room->setRoomVariant(
                        vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setRoomVariant cannot fail; continue as before.
                }
                int roomId8{};
                if (p_room->getId(roomId8) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                if (p_room->setName("semantic::Room#" +
                                    std::to_string(roomId8)) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setName cannot fail; continue as before.
                }

                int roomId9{};
                if (p_room->getId(roomId9) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemMgr] Structural Element #" << roomId9
                          << " classified as a semantic::Room from free-space "
                             "cluster "
                          << clusterId << "." << std::endl;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
