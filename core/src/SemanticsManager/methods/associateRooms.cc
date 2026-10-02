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
 * @file            associateRooms.cc
 *
 * @brief           Implements SemanticsManager::associateRooms(), declared in
 *                  SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"

#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::associateRooms(
    const Eigen::Vector3d clusterCentroid_world_in,
    const std::vector<vs_graphs::core::geometric::Plane *> &wallList_world_in,
    const std::vector<Eigen::Vector3d> &freeSpaceCluster_world_m_in,
    const std::unordered_set<int>      &excludedRoomIds_in,
    vs_graphs::core::semantic::Room   *&p_room_out)
{
    /* Extract parameter on centre distance threshold of room */
    const double centerDistanceThreshold =
        static_cast<double>(p_sysParams->roomSeg.centerDistanceThresh);

    constexpr double sideEpsilon = 0.20;

    /* Init list variables of nearest room and best shared room */
    vs_graphs::core::semantic::Room *p_bestSharedRoom = nullptr;
    vs_graphs::core::semantic::Room *p_nearestRoom    = nullptr;

    /* Init a counter to track the number of best same side matches */
    std::size_t bestSameSideMatches = 0;

    /* Init variables to find the best shared distance and nearest distance */
    double bestSharedDistance = std::numeric_limits<double>::max();
    double nearestDistance    = std::numeric_limits<double>::max();

    /* Get a list of all rooms within map */
    std::vector<vs_graphs::core::semantic::Room *> allRooms_world{};
    if (p_atlas->getAllRooms(allRooms_world) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Evaluate every room once against the complete cluster wall set. */
    for (vs_graphs::core::semantic::Room *p_mappedRoom : allRooms_world)
    {
        /* Skip room if invalid */
        bool isRoomBad{};
        if (!(p_mappedRoom == nullptr) &&
            p_mappedRoom->isBad(isRoomBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mappedRoom == nullptr || isRoomBad)
        {
            continue;
        }

        /* Skip rooms already matched to another cluster in this cycle */
        int roomId{};
        if (p_mappedRoom->getId(roomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (excludedRoomIds_in.count(roomId) > 0)
        {
            continue;
        }

        /* Extract room centroid */
        Eigen::Vector3d roomCenter_world{};
        if (p_mappedRoom->getCentroid(roomCenter_world) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Find the distance from the cluster center to the room center */
        const double roomCenterRelClusterCenterDistance =
            (roomCenter_world - clusterCentroid_world_in).norm();

        std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
        if (p_atlas->getAllPlanes(atlasAllPlanes) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        bool roomSeparatedFromCluster{};
        if (hasSeparatingFiniteWall(
                atlasAllPlanes,
                roomCenter_world,
                clusterCentroid_world_in,
                static_cast<double>(
                    p_sysParams->roomSeg.finiteWallBoundsMargin_m),
                roomSeparatedFromCluster) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: hasSeparatingFiniteWall returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Extract the walls from the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWallsList{};
        if (p_mappedRoom->getWalls(roomWallsList) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Init a list to track the room wall ids */
        std::unordered_set<int> roomWallIds;

        /* Reserve space in list to track room ids */
        roomWallIds.reserve(roomWallsList.size());

        /* Iterate through room walls to extract ids of room */
        for (vs_graphs::core::geometric::Plane *p_roomWall : roomWallsList)
        {
            /* Skip invalid walls */
            bool roomWallIsBad{};
            if ((p_roomWall != nullptr) &&
                p_roomWall->isBad(roomWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_roomWall != nullptr && !roomWallIsBad)
            {
                int roomWallGetId{};
                if (p_roomWall->getId(roomWallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                roomWallIds.insert(roomWallGetId);
            }
        }

        /* Init counters for walls on same and oposite sides of room */
        std::size_t sameSideMatches     = 0;
        std::size_t oppositeSideMatches = 0;

        /* Init a flag to see if the rooms share any walls */
        bool sharesAnyWall = false;

        /* Iterate through walls in room and see if they share walls */
        for (vs_graphs::core::geometric::Plane *p_candidateWall :
             wallList_world_in)
        {
            /* Skip invalid walls */
            bool candidateWallIsBad{};
            if (!(p_candidateWall == nullptr) &&
                p_candidateWall->isBad(candidateWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateWall == nullptr || candidateWallIsBad)
            {
                continue;
            }

            /* If wall is not linked to room, skip */
            int candidateWallGetId{};
            if (p_candidateWall->getId(candidateWallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (roomWallIds.count(candidateWallGetId) == 0)
            {
                continue;
            }

            /* If wall is not skipped, then shares a wall */
            sharesAnyWall = true;

            /* Extract plane equation */
            g2o::Plane3D candidateWallGetGlobalEquation{};
            if (p_candidateWall->getGlobalEquation(
                    candidateWallGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d candidateWallEquation =
                candidateWallGetGlobalEquation.coeffs();

            /* Find plane normal magnitude */
            const double normalNorm = candidateWallEquation.head<3>().norm();

            /* If magnitude is invalud, skip wall */
            if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
            {
                continue;
            }

            /* Find unit vector of plane norm */
            candidateWallEquation /= normalNorm;

            /* Caldaulte the side of the cluster */
            const double clusterSide =
                candidateWallEquation.head<3>().dot(clusterCentroid_world_in) +
                candidateWallEquation(3);

            /* Caldaulte the side of the room */
            const double roomSide =
                candidateWallEquation.head<3>().dot(roomCenter_world) +
                candidateWallEquation(3);

            /*!
             * A provisional room may initially have its centroid
             * directly on its first wall. Treat this as compatible.
             */
            const bool centroidNearWall =
                std::abs(clusterSide) <= sideEpsilon ||
                std::abs(roomSide) <= sideEpsilon;

            const bool sameSide = clusterSide * roomSide > 0.0;

            if (centroidNearWall || sameSide)
            {
                sameSideMatches++;
            }
            else
            {
                oppositeSideMatches++;
            }
        }

        /*!
         * Shared-wall matching is preferred, but only when the cluster and
         * room are on the same side.
         */
        if (sameSideMatches > 0 && !roomSeparatedFromCluster)
        {
            if (sameSideMatches > bestSameSideMatches ||
                (sameSideMatches == bestSameSideMatches &&
                 roomCenterRelClusterCenterDistance < bestSharedDistance))
            {
                bestSameSideMatches = sameSideMatches;

                bestSharedDistance = roomCenterRelClusterCenterDistance;

                p_bestSharedRoom = p_mappedRoom;
            }
        }

        /*!
         * A shared wall on the opposite side is evidence of an adjacent
         * room. Do not use centroid fallback in that case.
         */
        if (sharesAnyWall && sameSideMatches == 0 && oppositeSideMatches > 0)
        {
            continue;
        }

        double nearestFreeSpacePointDistance_m =
            std::numeric_limits<double>::infinity();

        for (const Eigen::Vector3d &freeSpacePoint_world_m :
             freeSpaceCluster_world_m_in)
        {
            if (freeSpacePoint_world_m.allFinite())
            {
                nearestFreeSpacePointDistance_m = std::min(
                    nearestFreeSpacePointDistance_m,
                    (roomCenter_world - freeSpacePoint_world_m).norm());
            }
        }

        const bool roomSupportedByConnectedFreeSpace =
            nearestFreeSpacePointDistance_m <= centerDistanceThreshold;

        /*!
         * Connected free space may update confirmed rooms even when a
         * changing wall subset provides no exact shared plane. A finite
         * wall between the two centroids always vetoes this fallback.
         */
        if (!sharesAnyWall && roomSupportedByConnectedFreeSpace &&
            !roomSeparatedFromCluster &&
            roomCenterRelClusterCenterDistance < nearestDistance)
        {
            nearestDistance = roomCenterRelClusterCenterDistance;

            p_nearestRoom = p_mappedRoom;
        }
    }

    vs_graphs::core::semantic::Room *p_selectedRoom =
        p_bestSharedRoom != nullptr ? p_bestSharedRoom : p_nearestRoom;

    p_room_out = p_selectedRoom;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
