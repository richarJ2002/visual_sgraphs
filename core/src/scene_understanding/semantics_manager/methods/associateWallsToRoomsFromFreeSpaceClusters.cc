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
 * @file            associateWallsToRoomsFromFreeSpaceClusters.cc
 *
 * @brief           Implements
 * SemanticsManager::associateWallsToRoomsFromFreeSpaceClusters(), declared in
 * SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::associateWallsToRoomsFromFreeSpaceClusters(void)
{
    PROFILE_FUNCTION();
    /* Extract latest skeleton cluster */
    std::vector<std::vector<Eigen::Vector3d>> latestSkeletonCluster{};
    if (getLatestSkeletonCluster(latestSkeletonCluster) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLatestSkeletonCluster returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<std::vector<Eigen::Vector3d>> clusters{};
    if (partitionFreeSpaceAtPassages(latestSkeletonCluster, clusters) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: partitionFreeSpaceAtPassages returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* If cluster is empty then return */
    if (clusters.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract all planes from the map */
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes{};
    if (p_atlas->getAllPlanes(allPlanes) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Create a list of all the walls there are in the SGraph */
    std::vector<vs_graphs::core::geometric::Plane *> allWalls;
    allWalls.reserve(allPlanes.size());

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition for why this must be
     * ground-anchored rather than an arbitrary in-plane axis). */
    geometric::Plane *p_groundPlaneForEvidence = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlaneForEvidence) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d groundNormalForEvidence_world = Eigen::Vector3d::Zero();
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
            groundNormalForEvidence_world = groundEq.head<3>() / groundNorm;
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
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        /* Append valid wall planes to list */
        WallAdmissionEvidence admissionEvidence{};
        if (evaluateWallAdmissionEvidence(p_plane,
                                          p_sysParams,
                                          groundNormalForEvidence_world,
                                          admissionEvidence) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateWallAdmissionEvidence returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (admissionEvidence.isAdmissible)
        {
            allWalls.push_back(p_plane);
        }
    }

    /* If there are no walls, then return */
    if (allWalls.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
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
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeCentroidFromPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getGeometrySnapshot returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
                wallGeometry.supportCloud;

            /* Skip wall if the point cloud is invalid */
            if (p_wallCloud == nullptr || p_wallCloud->empty())
            {
                continue;
            }

            /* Extract the centroid of the wall */
            const Eigen::Vector3d wallCentroid =
                wallGeometry.planeCentroid_world_m;

            /* Find the distance from the wall centroid and cluster centroid */
            const double centroidDistance =
                (wallCentroid - clusterCentroid).norm();

            /*!
             * Use centroid distance only as a coarse rejection condition.
             *
             * @note            A long wall may have a centroid far from the
             *                  room centre while still forming a valid boundary
             *                  of the room.
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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

            /* Find an axis U tangent to the wall plane. */
            const Eigen::Vector3d axisU =
                wallNormUnitVector.unitOrthogonal().normalized();

            /* Find the orthogonal axis completing the wall-plane basis. */
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

                const Eigen::Vector3d wallPoint_world_m(
                    static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));

                const Eigen::Vector3d wallPointRelToCentroid_world_m =
                    wallPoint_world_m - wallCentroid;

                const double wallPointU_m =
                    wallPointRelToCentroid_world_m.dot(axisU);
                const double wallPointV_m =
                    wallPointRelToCentroid_world_m.dot(axisV);

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

                /* Keep the smallest observed plane distance. */
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
                 * Count nearby points projected inside the finite wall patch.
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
        std::sort(
            closestWalls.begin(),
            closestWalls.end(),
            [](vs_graphs::core::geometric::Plane *p_first,
               vs_graphs::core::geometric::Plane *p_second)
            {
                int firstGetId{};
                if (p_first->getId(firstGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int secondGetId{};
                if (p_second->getId(secondGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return firstGetId < secondGetId;
            });

        /* Remove duplicate walls using IDs rather than pointers */
        closestWalls.erase(
            std::unique(
                closestWalls.begin(),
                closestWalls.end(),
                [](vs_graphs::core::geometric::Plane *p_first,
                   vs_graphs::core::geometric::Plane *p_second)
                {
                    int firstGetId{};
                    if (p_first->getId(firstGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int secondGetId{};
                    if (p_second->getId(secondGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    return firstGetId == secondGetId;
                }),
            closestWalls.end());

        /* Admit nearer walls first (see admitWallToRoom()'s see-through
         * gate); the id order above only serves the duplicate removal. */
        if (sortWallsByCameraDistance(closestWalls) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sortWallsByCameraDistance returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }

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

        std::vector<vs_graphs::core::semantic::Passage *> atlasAllPassages{};
        if (p_atlas->getAllPassages(atlasAllPassages) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (vs_graphs::core::semantic::Passage *p_passage : atlasAllPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            vs_graphs::core::semantic::Room *p_prospective = nullptr;
            if (p_passage->getProspectiveRoom(p_prospective) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            bool prospectiveIsBad{};
            if (!(p_prospective == nullptr) &&
                p_prospective->isBad(prospectiveIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant prospectiveRoomVariant{};
            if (!(p_prospective == nullptr || prospectiveIsBad) &&
                p_prospective->getRoomVariant(prospectiveRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int prospectiveId{};
            if (!(p_prospective == nullptr || prospectiveIsBad ||
                  prospectiveRoomVariant != vs_graphs::core::semantic::Room::
                                                RoomVariant::UNDEFINED) &&
                p_prospective->getId(prospectiveId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d passageEquation_world =
                passageGlobalEquation.coeffs();
            const double passageNormalNorm =
                passageEquation_world.head<3>().norm();

            if (!passageEquation_world.allFinite() || passageNormalNorm < 1e-8)
            {
                continue;
            }

            passageEquation_world /= passageNormalNorm;

            Eigen::Vector3d prospectiveCentroid_world_m{};
            if (p_prospective->getCentroid(prospectiveCentroid_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const double prospectiveSide_m =
                passageEquation_world.head<3>().dot(
                    prospectiveCentroid_world_m) +
                passageEquation_world(3);
            const double clusterSide_m =
                passageEquation_world.head<3>().dot(clusterCentroid) +
                passageEquation_world(3);
            const double centroidDistance_m =
                (prospectiveCentroid_world_m - clusterCentroid).norm();

            /*!
             * The far side is the one opposite the passage's persisted
             * known-side direction, the same direction the prospective room
             * was created from. The seed's own position cannot decide it: it
             * is placed only 0.15 m beyond the passage, inside the 0.20 m
             * band that does not resolve a side. Only a passage without a
             * known-side direction falls back to the prospective centroid.
             */
            semantic::Passage::KnownSideProvenance knownSide{};
            if (p_passage->getKnownSideProvenance(knownSide) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getKnownSideProvenance returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            bool hasKnownSideDirection{};
            if (knownSide.hasDirection(hasKnownSideDirection) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasDirection returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const double knownSideAlongNormal =
                hasKnownSideDirection ? passageEquation_world.head<3>().dot(
                                            knownSide.knownSideDirection_world)
                                      : 0.0;
            double farSideSign = 0.0;
            if (std::abs(knownSideAlongNormal) > 1e-3)
            {
                farSideSign = knownSideAlongNormal < 0.0 ? 1.0 : -1.0;
            }
            else if (std::abs(prospectiveSide_m) > 0.20)
            {
                farSideSign = prospectiveSide_m > 0.0 ? 1.0 : -1.0;
            }

            if (!prospectiveCentroid_world_m.allFinite() ||
                farSideSign == 0.0 || farSideSign * clusterSide_m <= 0.20 ||
                centroidDistance_m > kProspectiveDedupDistance_m)
            {
                continue;
            }

            std::vector<vs_graphs::core::geometric::Plane *> prospectiveWalls{};
            if (p_prospective->getWalls(prospectiveWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
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
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    return p_prospectiveWall != nullptr &&
                           !prospectiveWallIsBad &&
                           std::find(closestWalls.begin(),
                                     closestWalls.end(),
                                     p_prospectiveWall) != closestWalls.end();
                });

            /* A wall-less prospective has no wall to share; the cluster's
             * far-side position above and the absence of a finite wall
             * between seed and cluster (the associateRooms veto) are its
             * whole evidence. */
            const bool hasProspectiveWall = std::any_of(
                prospectiveWalls.begin(),
                prospectiveWalls.end(),
                [](vs_graphs::core::geometric::Plane *p_prospectiveWall)
                {
                    bool prospectiveWallIsBad{};
                    if ((p_prospectiveWall != nullptr) &&
                        p_prospectiveWall->isBad(prospectiveWallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    return p_prospectiveWall != nullptr &&
                           !prospectiveWallIsBad;
                });
            bool isSeedSeparatedFromCluster{};
            if (!hasProspectiveWall &&
                hasSeparatingFiniteWall(
                    allPlanes,
                    prospectiveCentroid_world_m,
                    clusterCentroid,
                    static_cast<double>(
                        p_sysParams->roomSeg.finiteWallBoundsMargin_m),
                    false,
                    isSeedSeparatedFromCluster) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: hasSeparatingFiniteWall returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if ((hasProspectiveWall && !sharesWallEvidence) ||
                isSeedSeparatedFromCluster ||
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
        vs_graphs::core::semantic::Room *p_reusedClusterRoom = nullptr;
        if (!(p_clusterProspective != nullptr) &&
            associateRooms(clusterCentroid,
                           closestWalls,
                           cluster,
                           matchedRoomIds,
                           p_reusedClusterRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: associateRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::semantic::Room *p_room =
            p_clusterProspective != nullptr ? p_clusterProspective
                                            : p_reusedClusterRoom;

        /* Track matched room to prevent double-matching in this cycle */
        if (p_room != nullptr)
        {
            int roomId{};
            if (p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            matchedRoomIds.insert(roomId);
        }

        /*!
         * If no existing room describes this free-space cluster, try to reuse
         * a wall owner; otherwise defer creation to the bootstrap or a
         * passage-linked prospective room.
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

            std::vector<vs_graphs::core::semantic::Room *>
                existingRooms_world{};
            if (p_atlas->getAllRooms(existingRooms_world) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            for (vs_graphs::core::geometric::Plane *p_candidateWall :
                 closestWalls)
            {
                bool candidateWallIsBad{};
                if (!(p_candidateWall == nullptr) &&
                    p_candidateWall->isBad(candidateWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_candidateWall == nullptr || candidateWallIsBad)
                {
                    continue;
                }

                /* The owner of a face bounds the face's observed side; a
                 * cluster on the other side is not that room. */
                bool isClusterOnForeignFaceSide{};
                if (isPointOnForeignSideOfWallFace(
                        p_candidateWall,
                        clusterCentroid,
                        isClusterOnForeignFaceSide) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isPointOnForeignSideOfWallFace returned "
                                 "a failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (isClusterOnForeignFaceSide)
                {
                    continue;
                }

                for (vs_graphs::core::semantic::Room *p_existingRoom :
                     existingRooms_world)
                {
                    bool existingRoomIsBad{};
                    if (!(p_existingRoom == nullptr) &&
                        p_existingRoom->isBad(existingRoomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int existingRoomId{};
                    if (!(p_existingRoom == nullptr || existingRoomIsBad) &&
                        p_existingRoom->getId(existingRoomId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
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
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWalls returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
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

                int reusedClusterRoomId{};
                if (p_room->getId(reusedClusterRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Reusing existing Room#"
                          << reusedClusterRoomId << " for cluster " << clusterId
                          << " (cluster walls already owned)." << std::endl;
            }
        }

        if (p_room == nullptr)
        {
            /*!
             * @brief           Axiom: every room after the first must be
             *                  discovered through a passage (a prospective-room
             *                  handle, checked above via
             *                  p_clusterProspective/matched wall ownership),
             *                  not conjured directly from free-space geometry
             *                  alone -- "if a wall is observed it must be
             *                  linked to a room; if that room is new, it must
             *                  be observed through a passage" (user rule). The
             *                  exception is the first room -- but per-MAP, not
             *                  per-mission: every tracking-loss reset starts an
             *                  entirely new Map with no passages yet either, so
             *                  it needs its own bootstrap room the same way the
             *                  mission's very first map did.
             *                  Atlas::GetAllRooms() spans every map (confirmed
             *                  by reading it), so scoping this to the CURRENT
             *                  map only is required -- otherwise a confirmed
             *                  room surviving in an old, now-inactive map would
             *                  permanently block every future map from ever
             *                  bootstrapping its own first room.
             */
            map::Map *p_currentMapForBootstrapCheck = nullptr;
            if (p_atlas->getCurrentMap(p_currentMapForBootstrapCheck) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCurrentMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<semantic::Room *> currentMapForBootstrapCheckAllRooms{};
            if ((p_currentMapForBootstrapCheck != nullptr) &&
                p_currentMapForBootstrapCheck->getAllRooms(
                    currentMapForBootstrapCheckAllRooms) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const std::vector<semantic::Room *> currentMapRooms =
                p_currentMapForBootstrapCheck != nullptr
                    ? currentMapForBootstrapCheckAllRooms
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
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    semantic::Room::RoomVariant existingRoomRoomVariant{};
                    if ((p_existingRoom != nullptr && !existingRoomIsBad) &&
                        p_existingRoom->getRoomVariant(
                            existingRoomRoomVariant) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRoomVariant returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
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
             * creation from that hierarchy; free-space must not conjure a
             * fresh SE# from stale cross-map walls/clusters in this cycle. */
            int atlasGetCurrentSemanticRoomIdentity{};
            if ((p_atlas != nullptr) &&
                p_atlas->getCurrentSemanticRoomIdentity(
                    atlasGetCurrentSemanticRoomIdentity) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getCurrentSemanticRoomIdentity returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            const int pendingRecoveryRoomId =
                p_atlas != nullptr ? atlasGetCurrentSemanticRoomIdentity : -1;
            std::optional<semantic::RoomContextSnapshot> atlasRoomContext{};
            if ((pendingRecoveryRoomId >= 0) &&
                p_atlas->copyLatestRoomContext(pendingRecoveryRoomId,
                                               atlasRoomContext) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyLatestRoomContext returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (pendingRecoveryRoomId >= 0 && atlasRoomContext.has_value())
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " deferred: recovery Room#"
                          << pendingRecoveryRoomId
                          << " owns first-room creation on this map."
                          << std::endl;
                continue;
            }

            /*!
             * @brief           First-room creation lives in the bootstrap
             *                  hierarchy, which runs before free-space
             *                  detection every cycle and promotes its room
             *                  immediately: a cluster that matches no room,
             *                  wall owner, or passage-linked prospective at
             *                  this point describes no known space, so it is
             *                  deferred rather than conjured into a duplicate
             *                  first room.
             */
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

        std::vector<vs_graphs::core::semantic::Passage *> activePassages{};
        if (p_atlas->getAllPassages(activePassages) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Room::RoomVariant roomVariant{};
                if ((p_passage != nullptr &&
                     p_passageProspectiveRoom == p_room) &&
                    p_room->getRoomVariant(roomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
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
            if (consolidateRoomsInFreeSpaceCluster(p_room, cluster, allWalls) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: consolidateRoomsInFreeSpaceCluster returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /* Find the walls of the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * @brief           Perspective guard: a wall observed through an
         *                  opening is on the far side of the passage's
         *                  supporting wall and therefore cannot bound this
         *                  room. enforcePassageApertureBackstop() decides and
         *                  applies the routing, the same check wall admission
         *                  uses, so cluster-driven and orphan-driven admission
         *                  route a far-side wall identically.
         */
        for (vs_graphs::core::geometric::Plane *wall : closestWalls)
        {
            /* Skip invalid walls */
            bool clusterWallIsBad{};
            if (!(wall == nullptr) &&
                wall->isBad(clusterWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (wall == nullptr || clusterWallIsBad)
            {
                continue;
            }

            /*!
             * @brief           Route far-side walls away from this room.
             *                  The ground normal is needed to project the
             *                  aperture crossing.
             */
            bool farSideBound = false;
            {
                geometric::Plane *p_groundPlane = nullptr;
                if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBiggestGroundPlane returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d groundNormal_world = Eigen::Vector3d::Zero();
                bool            groundPlaneIsBad{};
                if ((p_groundPlane != nullptr) &&
                    p_groundPlane->isBad(groundPlaneIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_groundPlane != nullptr && !groundPlaneIsBad)
                {
                    g2o::Plane3D groundPlaneGetGlobalEquation{};
                    if (p_groundPlane->getGlobalEquation(
                            groundPlaneGetGlobalEquation) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector4d groundEq =
                        groundPlaneGetGlobalEquation.coeffs();
                    const double groundNorm = groundEq.head<3>().norm();
                    if (groundEq.allFinite() && groundNorm > 1e-8)
                    {
                        groundNormal_world = groundEq.head<3>() / groundNorm;
                    }
                }

                std::vector<vs_graphs::core::semantic::Passage *>
                    wallOwnerPassages{};
                if (p_atlas->getAllPassages(wallOwnerPassages) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getAllPassages returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                std::sort(wallOwnerPassages.begin(),
                          wallOwnerPassages.end(),
                          semantic::isEntityIdLess<semantic::Passage>);

                PassageSideEnforcementOutcome outcome{};
                if (enforcePassageApertureBackstop(p_room,
                                                   wall,
                                                   wallOwnerPassages,
                                                   groundNormal_world,
                                                   outcome) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: enforcePassageApertureBackstop returned "
                                 "a failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                farSideBound =
                    outcome != PassageSideEnforcementOutcome::NO_VIOLATION;
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
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int wallGetId{};
                    if ((p_existingWall != nullptr) &&
                        wall->getId(wallGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    return p_existingWall != nullptr &&
                           existingWallGetId == wallGetId;
                });

            /* Skip walls already owned by a room. */
            if (alreadyInRoom)
            {
                continue;
            }

            /*
             * A mapped wall surface has one room owner. A room detected on
             * the opposite side must be bounded by its independently observed
             * wall surface rather than sharing this plane object.
             */
            std::vector<vs_graphs::core::semantic::Room *> mappedRooms{};
            if (p_atlas->getAllRooms(mappedRooms) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllRooms returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            const std::vector<semantic::Room *>::iterator
                existingOwnerIterator = std::find_if(
                    mappedRooms.begin(),
                    mappedRooms.end(),
                    [p_room, wall](vs_graphs::core::semantic::Room *p_otherRoom)
                    {
                        bool otherRoomIsBad{};
                        if (!(p_otherRoom == nullptr ||
                              p_otherRoom == p_room) &&
                            p_otherRoom->isBad(otherRoomIsBad) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isBad returned a failure status "
                                         "although it "
                                         "cannot fail; continuing as before.",
                                         __func__);
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
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getWalls returned a failure status "
                                "although "
                                "it cannot fail; continuing as before.",
                                __func__);
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
                g2o::Plane3D candidateWallGlobalEquation{};
                if (wall->getGlobalEquation(candidateWallGlobalEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getGlobalEquation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector4d wallEquation_world =
                    candidateWallGlobalEquation.coeffs();
                const double wallNormalNorm =
                    wallEquation_world.head<3>().norm();

                if (wallEquation_world.allFinite() && wallNormalNorm > 1e-8)
                {
                    wallEquation_world /= wallNormalNorm;

                    constexpr double maximumProvisionalPlaneDistance_m = 0.20;
                    Eigen::Vector3d  existingWallOwnerCentroid{};
                    if (p_existingWallOwner->getCentroid(
                            existingWallOwnerCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const double ownerPlaneDistance_m =
                        std::abs(wallEquation_world.head<3>().dot(
                                     existingWallOwnerCentroid) +
                                 wallEquation_world(3));

                    semantic::Room::RoomVariant existingWallOwnerRoomVariant{};
                    if (p_existingWallOwner->getRoomVariant(
                            existingWallOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRoomVariant returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    std::vector<geometric::Plane *> existingWallOwnerWalls{};
                    if ((existingWallOwnerRoomVariant ==
                         semantic::Room::RoomVariant::UNDEFINED) &&
                        p_existingWallOwner->getWalls(existingWallOwnerWalls) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWalls returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
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
                geometric::Plane *p_groundPlane = nullptr;
                if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBiggestGroundPlane returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d meanObservationPosition_world_m =
                    Eigen::Vector3d::Zero();
                std::size_t validObservationCount = 0U;

                std::map<keyframe::KeyFrame *, geometric::Plane::Observation>
                    wallGetObservations{};
                if (wall->getObservations(wallGetObservations) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getObservations returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                for (const auto &[p_keyFrame, observation] :
                     wallGetObservations)
                {
                    static_cast<void>(observation);

                    bool keyFrameIsBad{};
                    if (!(p_keyFrame == nullptr) &&
                        p_keyFrame->isBad(keyFrameIsBad) !=
                            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame == nullptr || keyFrameIsBad)
                    {
                        continue;
                    }

                    Eigen::Vector3f keyFrameCameraCenter{};
                    if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector3d cameraCentre_world_m =
                        keyFrameCameraCenter.cast<double>();

                    if (cameraCentre_world_m.allFinite())
                    {
                        meanObservationPosition_world_m += cameraCentre_world_m;
                        validObservationCount++;
                    }
                }

                bool ownerGroundPlaneIsBad{};
                if ((!existingOwnerIsTransferableProvisional &&
                     validObservationCount > 0U && p_groundPlane != nullptr) &&
                    p_groundPlane->isBad(ownerGroundPlaneIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!existingOwnerIsTransferableProvisional &&
                    validObservationCount > 0U && p_groundPlane != nullptr &&
                    !ownerGroundPlaneIsBad)
                {
                    meanObservationPosition_world_m /=
                        static_cast<double>(validObservationCount);

                    g2o::Plane3D ownerGroundPlaneGlobalEquation{};
                    if (p_groundPlane->getGlobalEquation(
                            ownerGroundPlaneGlobalEquation) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector4d groundEquation_world =
                        ownerGroundPlaneGlobalEquation.coeffs();
                    const double groundNormalNorm =
                        groundEquation_world.head<3>().norm();

                    if (groundEquation_world.allFinite() &&
                        groundNormalNorm > 1e-8)
                    {
                        const Eigen::Vector3d groundNormal_world =
                            groundEquation_world.head<3>() / groundNormalNorm;

                        std::vector<vs_graphs::core::semantic::Passage *>
                            ownerBoundaryPassages{};
                        if (p_atlas->getAllPassages(ownerBoundaryPassages) !=
                            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getAllPassages returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        for (semantic::Passage *p_passage :
                             ownerBoundaryPassages)
                        {
                            Eigen::Vector3d passageOwnerCentroid{};
                            if (p_existingWallOwner->getCentroid(
                                    passageOwnerCentroid) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getCentroid returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            bool crossesOwnerPassageOpening{};
                            if (segmentCrossesPassageOpening(
                                    passageOwnerCentroid,
                                    clusterCentroid,
                                    p_passage,
                                    groundNormal_world,
                                    p_sysParams->roomSeg.passagePartition
                                        .openingMargin_m,
                                    p_sysParams->roomSeg.passagePartition
                                        .minimumSideDistance_m,
                                    crossesOwnerPassageOpening) !=
                                SemanticsManagerStatus::
                                    SEMANTICS_MANAGER_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: segmentCrossesPassageOpening returned "
                                    "a failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if (!crossesOwnerPassageOpening)
                            {
                                continue;
                            }

                            g2o::Plane3D ownerPassageGlobalEquation{};
                            if (p_passage->getGlobalEquation(
                                    ownerPassageGlobalEquation) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                /*!
                                 * getGlobalEquation cannot fail; continue as
                                 * before.
                                 */
                            }
                            Eigen::Vector4d passageEquation_world =
                                ownerPassageGlobalEquation.coeffs();
                            const double passageNormalNorm =
                                passageEquation_world.head<3>().norm();

                            if (!passageEquation_world.allFinite() ||
                                passageNormalNorm < 1e-8)
                            {
                                continue;
                            }

                            passageEquation_world /= passageNormalNorm;
                            const double candidateSide_m =
                                passageEquation_world.head<3>().dot(
                                    clusterCentroid) +
                                passageEquation_world(3);
                            const double observationSide_m =
                                passageEquation_world.head<3>().dot(
                                    meanObservationPosition_world_m) +
                                passageEquation_world(3);

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
            bool wasWallAdmitted{};
            if (admitWallToRoom(p_room, wall, wasWallAdmitted) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: admitWallToRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!wasWallAdmitted)
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
                existingWallOwnerWasWallRemoved = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
            }
            if (p_existingWallOwner != nullptr &&
                existingWallOwnerWasWallRemoved)
            {
                if (existingOwnerIsTransferableProvisional)
                {
                    map::Map *p_currentMap = nullptr;
                    if (p_atlas->getCurrentMap(p_currentMap) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCurrentMap returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (p_currentMap != nullptr)
                    {
                        if (p_currentMap->eraseDetectedMapRoom(
                                p_existingWallOwner) !=
                            map::MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: eraseDetectedMapRoom returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (p_currentMap->eraseMarkerBasedMapRoom(
                                p_existingWallOwner) !=
                            map::MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: eraseMarkerBasedMapRoom returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                    }

                    if (p_existingWallOwner->setBad() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }

                    int existingWallOwnerId{};
                    if (p_existingWallOwner->getId(existingWallOwnerId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int orphanReceivingRoomId{};
                    if (p_room->getId(orphanReceivingRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int orphanWallId{};
                    if (wall->getId(orphanWallId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Transferred orphan Wall#"
                              << orphanWallId << " from provisional SE#"
                              << existingWallOwnerId << " to Room#"
                              << orphanReceivingRoomId << "." << std::endl;
                }
                else
                {
                    int previousRoomId{};
                    if (p_existingWallOwner->getId(previousRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int transferReceivingRoomId{};
                    if (p_room->getId(transferReceivingRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int transferPassageId{};
                    if (p_transferPassage->getId(transferPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int transferWallId{};
                    if (wall->getId(transferWallId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Transferred Wall#" << transferWallId
                              << " from Room#" << previousRoomId << " to Room#"
                              << transferReceivingRoomId << " through Passage#"
                              << transferPassageId
                              << " using wall-observation evidence."
                              << std::endl;
                }
            }

            std::vector<geometric::Plane *> wallsAfterAdmission{};
            if (p_room->getWalls(wallsAfterAdmission) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            roomWalls = wallsAfterAdmission;

            /* Register the uniquely owned room-wall surface. */
            int registeredWallId{};
            if (wall->getId(registeredWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::geometric::Plane *p_registeredRoomWall = nullptr;
            if (p_atlas->getRoomWallPlaneById(registeredWallId,
                                              p_registeredRoomWall) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getRoomWallPlaneById returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_registeredRoomWall == nullptr)
            {
                if (p_atlas->addRoomWallPlane(wall) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addRoomWallPlane returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        /* Find all the walls in a room */
        std::vector<geometric::Plane *> wallsBeforeConsolidation{};
        if (p_room->getWalls(wallsBeforeConsolidation) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        roomWalls = wallsBeforeConsolidation;

        /*!
         * Consolidate provisional single-wall structural elements whose wall
         * has now been absorbed into the cluster-backed room.
         *
         * @note            This is deliberately more restrictive than the old
         *                  centroid-only reAssociateRooms() implementation.
         */
        if (utils::utils::Utils::consolidateProvisionalRooms(p_room, p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: consolidateProvisionalRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Remove invalid relationships from the room's persistent graph. */
        std::size_t roomRemovedWallCount{};
        if (p_room->removeInvalidWalls(roomRemovedWallCount) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: removeInvalidWalls returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> wallsAfterInvalidRemoval{};
        if (p_room->getWalls(wallsAfterInvalidRemoval) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        roomWalls = wallsAfterInvalidRemoval;

        /*!
         * The semantic room centre comes from computeRoomCentreFromWalls():
         * the centre of the floor box spanned by the walls' extents, or the
         * inward-pushed wall-centroid mean for walls that span no box. Rooms
         * without walls keep their creation centroid until walls arrive:
         * free-space evidence places a room once, at creation, and never
         * maintains it.
         */
        std::optional<Eigen::Vector3d> wallCentre_world_m{};
        if (computeRoomCentreFromWalls(p_room, roomWalls, wallCentre_world_m) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeRoomCentreFromWalls returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (wallCentre_world_m.has_value())
        {
            /*!
             * Damped update, not a snap. This centroid feeds two decisions
             * that REMOVE walls -- isWallFaceForeignToRoom's foreign-face
             * check (enforcePassageSideInvariant) and the passage far-side
             * router -- which change roomWalls, which changes the centre
             * computed above. An undamped snap closes an unstable feedback
             * loop with no damping: remove a wall -> centroid shifts ->
             * another wall's side test flips -> remove that too -> centroid
             * shifts further. Live-observed 2026-09-04 crossing Office 6's
             * passage: the same handful of walls near the doorway repeatedly
             * removed and re-admitted, tens of times in a row, a previously
             * COMPLETE room's boundary never settling. Blending in a minority
             * weight of the fresh centre still lets the centroid track
             * genuinely new evidence over several cycles, but one cycle's
             * wall churn can no longer swing it far enough to flip another
             * wall's side test.
             */
            Eigen::Vector3d previousCentroid_world_m{};
            if (p_room->getCentroid(previousCentroid_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            constexpr double      centroidDampingWeight = 0.25;
            const Eigen::Vector3d dampedCentroid_world_m =
                previousCentroid_world_m.allFinite()
                    ? (centroidDampingWeight * (*wallCentre_world_m) +
                       (1.0 - centroidDampingWeight) * previousCentroid_world_m)
                    : *wallCentre_world_m;

            if (p_room->setCentroid(dampedCentroid_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /*!
         * Confirm the room from its free-space cluster.
         *
         * @note            A room is defined by connected free space, not by
         *                  having a particular arrangement or number of walls.
         *                  The associated walls describe the room boundary but
         *                  do not define whether the free-space region is a
         *                  room.
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
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
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
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (validFreeSpaceCluster && hasBoundaryEvidence &&
            roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            if (prospectiveWallEvidenceStillMatches)
            {
                map::Map *p_roomMap = nullptr;
                if (p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_roomMap != nullptr)
                {
                    if (p_roomMap->promoteCandidateMapRoom(p_room) !=
                        map::MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: promoteCandidateMapRoom returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                }

                if (p_room->setRoomVariant(
                        vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                int confirmedRoomId{};
                if (p_room->getId(confirmedRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room->setName("Room#" +
                                    std::to_string(confirmedRoomId)) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setName returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int confirmedCycleRoomId{};
                if (p_room->getId(confirmedCycleRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                prospectiveRoomCycles.erase(confirmedCycleRoomId);

                if (p_room->setDoorways(p_clusterPassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_clusterPassage->setProspectiveRoom(p_room) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                int promotedRoomId{};
                if (p_room->getId(promotedRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Promoted prospective Room#"
                          << promotedRoomId
                          << " to ROOM from far-side cluster evidence."
                          << std::endl;
            }
            else if (!roomIsPassageBoundProspective)
            {
                map::Map *p_roomMap = nullptr;
                if (p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_roomMap != nullptr)
                {
                    if (p_roomMap->promoteCandidateMapRoom(p_room) !=
                        map::MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: promoteCandidateMapRoom returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                }
                if (p_room->setRoomVariant(
                        vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                int prospectiveRoomId{};
                if (p_room->getId(prospectiveRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room->setName("Room#" +
                                    std::to_string(prospectiveRoomId)) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setName returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                int structuralElementId{};
                if (p_room->getId(structuralElementId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Structural Element #"
                          << structuralElementId
                          << " classified as a Room from free-space "
                             "cluster "
                          << clusterId << "." << std::endl;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
