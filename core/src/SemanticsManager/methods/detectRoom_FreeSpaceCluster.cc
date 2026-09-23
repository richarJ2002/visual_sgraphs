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

    /* For every plane, extract walls */
    for (vs_graphs::core::geometric::Plane *plane : allPlanes)
    {
        /* Skip bad planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Append valid wall planes to list */
        if (evaluateWallAdmissionEvidence(plane,
                                          p_sysParams,
                                          groundNormalForEvidence_World)
                .admissible)
        {
            allWalls.push_back(plane);
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
        const Eigen::Vector3d clusterCentroid =
            utils::utils::Utils::computeCentroidFromPoints(cluster);

        /* Initialize a list of planes to track the closest walls */
        std::vector<vs_graphs::core::geometric::Plane *> closestWalls;
        closestWalls.reserve(allWalls.size());

        /* For each wall */
        for (vs_graphs::core::geometric::Plane *wall : allWalls)
        {
            /* Skip wall if it is bad */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Extract the point cloud for the wall */
            const geometric::Plane::GeometrySnapshot wallGeometry =
                wall->getGeometrySnapshot();
            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr wallCloud =
                wallGeometry.supportCloud;

            /* Skip wall if the point cloud is invalid */
            if (wallCloud == nullptr || wallCloud->empty())
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
            Eigen::Vector4d wallEquation = wall->getGlobalEquation().coeffs();

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
            for (const pcl::PointXYZRGBA &point : wallCloud->points)
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
                  [](vs_graphs::core::geometric::Plane *first,
                     vs_graphs::core::geometric::Plane *second)
                  { return first->getId() < second->getId(); });

        /* Remove duplicate walls using IDs rather than pointers */
        closestWalls.erase(
            std::unique(closestWalls.begin(),
                        closestWalls.end(),
                        [](vs_graphs::core::geometric::Plane *first,
                           vs_graphs::core::geometric::Plane *second)
                        { return first->getId() == second->getId(); }),
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

            vs_graphs::core::semantic::Room *p_prospective =
                p_passage->getProspectiveRoom();

            if (p_prospective == nullptr || p_prospective->isBad() ||
                p_prospective->getRoomVariant() !=
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
                matchedRoomIds.count(p_prospective->getId()) > 0U)
            {
                continue;
            }

            Eigen::Vector4d passageEquation_World =
                p_passage->getGlobalEquation().coeffs();
            const double passageNormalNorm =
                passageEquation_World.head<3>().norm();

            if (!passageEquation_World.allFinite() || passageNormalNorm < 1e-8)
            {
                continue;
            }

            passageEquation_World /= passageNormalNorm;

            const Eigen::Vector3d prospectiveCentroid_World_m =
                p_prospective->getCentroid();
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

            const std::vector<vs_graphs::core::geometric::Plane *>
                       prospectiveWalls   = p_prospective->getWalls();
            const bool sharesWallEvidence = std::any_of(
                prospectiveWalls.begin(),
                prospectiveWalls.end(),
                [&closestWalls](
                    vs_graphs::core::geometric::Plane *p_prospectiveWall)
                {
                    return p_prospectiveWall != nullptr &&
                           !p_prospectiveWall->isBad() &&
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
        vs_graphs::core::semantic::Room *room =
            p_clusterProspective != nullptr ? p_clusterProspective
                                            : associateRooms(clusterCentroid,
                                                             closestWalls,
                                                             cluster,
                                                             matchedRoomIds);

        /* Track matched room to prevent double-matching in this cycle */
        if (room != nullptr)
        {
            matchedRoomIds.insert(room->getId());
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
        if (room == nullptr)
        {
            vs_graphs::core::semantic::Room *p_wallOwnerRoom = nullptr;

            const std::vector<vs_graphs::core::semantic::Room *>
                existingRooms_World = p_atlas->getAllRooms();

            for (vs_graphs::core::geometric::Plane *p_candidateWall :
                 closestWalls)
            {
                if (p_candidateWall == nullptr || p_candidateWall->isBad())
                {
                    continue;
                }

                for (vs_graphs::core::semantic::Room *p_existingRoom :
                     existingRooms_World)
                {
                    if (p_existingRoom == nullptr || p_existingRoom->isBad() ||
                        matchedRoomIds.count(p_existingRoom->getId()) > 0)
                    {
                        continue;
                    }

                    const std::vector<vs_graphs::core::geometric::Plane *>
                        roomWallsList = p_existingRoom->getWalls();

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
                room = p_wallOwnerRoom;

                std::cout << "[SemMgr] Reusing existing semantic::Room#"
                          << room->getId() << " for cluster " << clusterId
                          << " (cluster walls already owned)." << std::endl;
            }
        }

        if (room == nullptr)
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
            const bool anyConfirmedRoomExistsInCurrentMap =
                std::any_of(currentMapRooms.begin(),
                            currentMapRooms.end(),
                            [](vs_graphs::core::semantic::Room *p_existingRoom)
                            {
                                return p_existingRoom != nullptr &&
                                       !p_existingRoom->isBad() &&
                                       p_existingRoom->getRoomVariant() ==
                                           vs_graphs::core::semantic::Room::
                                               RoomVariant::ROOM;
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
        const bool roomIsPassageBoundProspective =
            std::any_of(activePassages.begin(),
                        activePassages.end(),
                        [room](vs_graphs::core::semantic::Passage *p_passage)
                        {
                            return p_passage != nullptr &&
                                   p_passage->getProspectiveRoom() == room &&
                                   room->getRoomVariant() ==
                                       vs_graphs::core::semantic::Room::
                                           RoomVariant::UNDEFINED;
                        });

        /* A prospective must first pass the external cluster validation below;
         * generic consolidation must not classify or replace it early. */
        if (!roomIsPassageBoundProspective)
        {
            consolidateRoomsInFreeSpaceCluster(room, cluster, allWalls);
        }

        /* Find the walls of the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
            room->getWalls();

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
            if (wall == nullptr || wall->isBad())
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
                if (p_groundPlane != nullptr && !p_groundPlane->isBad())
                {
                    const Eigen::Vector4d groundEq =
                        p_groundPlane->getGlobalEquation().coeffs();
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
                    if (!segmentCrossesPassageOpening(
                            room->getCentroid(),
                            wall->getCentroid().cast<double>(),
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
                        if (p_other == nullptr || p_other->isBad() ||
                            p_other == room ||
                            p_other->getRoomVariant() ==
                                vs_graphs::core::semantic::Room::RoomVariant::
                                    UNDEFINED)
                        {
                            continue;
                        }
                        const std::vector<geometric::Plane *> otherWalls =
                            p_other->getWalls();
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

                    vs_graphs::core::semantic::Room *pProspective =
                        p_passage->getProspectiveRoom();

                    if (pProspective == nullptr || pProspective->isBad())
                    {
                        /* The far-side room does not exist yet (it may be
                         * created later in this cycle by
                         * associatePassagesToRooms). Keep the wall off the
                         * near room so it can bind onto the prospective. */
                        room->removeWall(wall);
                        if (p_atlas->getRoomWallPlaneById(wall->getId()) ==
                            nullptr)
                        {
                            p_atlas->addRoomWallPlane(wall);
                        }
                        std::cout
                            << "[SemMgr] Far-side Wall#" << wall->getId()
                            << " at semantic::Passage#" << p_passage->getId()
                            << " has no prospective yet; held unbound for the "
                               "far-side room."
                            << std::endl;
                        farSideBound = true;
                        break;
                    }

                    room->removeWall(wall);
                    if (p_atlas->getRoomWallPlaneById(wall->getId()) == nullptr)
                    {
                        p_atlas->addRoomWallPlane(wall);
                    }
                    admitWallToRoom(pProspective, wall);
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
                    for (OpenPassageEvidence &evidence : openPassageEvidence_)
                    {
                        if (!segmentCrossesOpenPassageEvidence(
                                room->getCentroid(),
                                wall->getCentroid().cast<double>(),
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

                        room->removeWall(wall);
                        if (p_atlas->getRoomWallPlaneById(wall->getId()) ==
                            nullptr)
                        {
                            p_atlas->addRoomWallPlane(wall);
                        }
                        std::cout << "[SemMgr] Far-side Wall#" << wall->getId()
                                  << " crosses an unconfirmed passage opening "
                                     "(evidence at wall "
                                  << (evidence.p_supportingWall != nullptr
                                          ? evidence.p_supportingWall->getId()
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
                [wall](vs_graphs::core::geometric::Plane *existingWall) {
                    return existingWall != nullptr &&
                           existingWall->getId() == wall->getId();
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
                [room, wall](vs_graphs::core::semantic::Room *p_otherRoom)
                {
                    if (p_otherRoom == nullptr || p_otherRoom == room ||
                        p_otherRoom->isBad())
                    {
                        return false;
                    }

                    const std::vector<vs_graphs::core::geometric::Plane *>
                        otherRoomWalls = p_otherRoom->getWalls();

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
                Eigen::Vector4d wallEquation_World =
                    wall->getGlobalEquation().coeffs();
                const double wallNormalNorm =
                    wallEquation_World.head<3>().norm();

                if (wallEquation_World.allFinite() && wallNormalNorm > 1e-8)
                {
                    wallEquation_World /= wallNormalNorm;

                    constexpr double maximumProvisionalPlaneDistance_m = 0.20;
                    const double     ownerPlaneDistance_m =
                        std::abs(wallEquation_World.head<3>().dot(
                                     p_existingWallOwner->getCentroid()) +
                                 wallEquation_World(3));

                    existingOwnerIsTransferableProvisional =
                        p_existingWallOwner->getRoomVariant() ==
                            semantic::Room::RoomVariant::UNDEFINED &&
                        p_existingWallOwner->getWalls().size() == 1U &&
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

                for (const auto &[p_keyFrame, observation] :
                     wall->getObservations())
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

                if (!existingOwnerIsTransferableProvisional &&
                    validObservationCount > 0U && p_groundPlane != nullptr &&
                    !p_groundPlane->isBad())
                {
                    meanObservationPosition_World_m /=
                        static_cast<double>(validObservationCount);

                    Eigen::Vector4d groundEquation_World =
                        p_groundPlane->getGlobalEquation().coeffs();
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
                            if (!segmentCrossesPassageOpening(
                                    p_existingWallOwner->getCentroid(),
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

                            Eigen::Vector4d passageEquation_World =
                                p_passage->getGlobalEquation().coeffs();
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
            if (!admitWallToRoom(room, wall))
            {
                continue;
            }

            if (p_existingWallOwner != nullptr &&
                p_existingWallOwner->removeWall(wall))
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

                    p_existingWallOwner->setBad();

                    std::cout << "[SemMgr] Transferred orphan Wall#"
                              << wall->getId() << " from provisional SE#"
                              << p_existingWallOwner->getId()
                              << " to semantic::Room#" << room->getId() << "."
                              << std::endl;
                }
                else
                {
                    std::cout
                        << "[SemMgr] Transferred Wall#" << wall->getId()
                        << " from semantic::Room#"
                        << p_existingWallOwner->getId() << " to semantic::Room#"
                        << room->getId() << " through semantic::Passage#"
                        << p_transferPassage->getId()
                        << " using wall-observation evidence." << std::endl;
                }
            }

            roomWalls = room->getWalls();

            /* Register the uniquely owned room-wall surface. */
            if (p_atlas->getRoomWallPlaneById(wall->getId()) == nullptr)
            {
                p_atlas->addRoomWallPlane(wall);
            }
        }

        /* Find all the walls in a room */
        roomWalls = room->getWalls();

        /*!
         * Consolidate provisional single-wall structural elements whose wall
         * has now been absorbed into the cluster-backed room.
         *
         * @note        This is deliberately more restrictive than the old
         *              centroid-only reAssociateRooms() implementation.
         */
        utils::utils::Utils::consolidateProvisionalRooms(room, p_atlas);

        /* Remove invalid relationships from the room's persistent graph. */
        room->removeInvalidWalls();
        roomWalls = room->getWalls();

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
                if (p_roomWall == nullptr || p_roomWall->isBad())
                {
                    continue;
                }

                const Eigen::Vector3d wallCentroid_World_m =
                    p_roomWall->getCentroid().cast<double>();

                if (!wallCentroid_World_m.allFinite())
                {
                    continue;
                }

                const std::optional<Eigen::Vector3d> inwardNormal_World =
                    room->getWallNormalTowardRoom_World(p_roomWall);

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
                const Eigen::Vector3d previousCentroid_World_m =
                    room->getCentroid();
                constexpr double      centroidDampingWeight = 0.25;
                const Eigen::Vector3d dampedCentroid_World_m =
                    previousCentroid_World_m.allFinite()
                        ? (centroidDampingWeight * correctedCentroid_World_m +
                           (1.0 - centroidDampingWeight) *
                               previousCentroid_World_m)
                        : correctedCentroid_World_m;

                room->setCentroid(dampedCentroid_World_m);
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
            p_clusterProspective == room && p_clusterPassage != nullptr &&
            std::any_of(prospectiveWallsBeforeCluster.begin(),
                        prospectiveWallsBeforeCluster.end(),
                        [&closestWalls,
                         &roomWalls](vs_graphs::core::geometric::Plane *p_wall)
                        {
                            return p_wall != nullptr && !p_wall->isBad() &&
                                   std::find(closestWalls.begin(),
                                             closestWalls.end(),
                                             p_wall) != closestWalls.end() &&
                                   std::find(roomWalls.begin(),
                                             roomWalls.end(),
                                             p_wall) != roomWalls.end();
                        });

        /* Confirm the cluster-backed structural element as a room. */
        if (validFreeSpaceCluster && hasBoundaryEvidence &&
            room->getRoomVariant() ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            if (prospectiveWallEvidenceStillMatches)
            {
                Map *p_roomMap = room->getMap();
                if (p_roomMap != nullptr)
                {
                    p_roomMap->promoteCandidateMapRoom(room);
                }

                room->setRoomVariant(
                    vs_graphs::core::semantic::Room::RoomVariant::ROOM);
                room->setName("semantic::Room#" +
                              std::to_string(room->getId()));
                prospectiveRoomCycles_.erase(room->getId());

                room->setDoorways(p_clusterPassage);
                p_clusterPassage->setProspectiveRoom(room);

                std::cout << "[SemMgr] Promoted prospective semantic::Room#"
                          << room->getId()
                          << " to ROOM from far-side cluster evidence."
                          << std::endl;
            }
            else if (!roomIsPassageBoundProspective)
            {
                Map *p_roomMap = room->getMap();
                if (p_roomMap != nullptr)
                {
                    p_roomMap->promoteCandidateMapRoom(room);
                }
                room->setRoomVariant(
                    vs_graphs::core::semantic::Room::RoomVariant::ROOM);
                room->setName("semantic::Room#" +
                              std::to_string(room->getId()));

                std::cout << "[SemMgr] Structural Element #" << room->getId()
                          << " classified as a semantic::Room from free-space "
                             "cluster "
                          << clusterId << "." << std::endl;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
