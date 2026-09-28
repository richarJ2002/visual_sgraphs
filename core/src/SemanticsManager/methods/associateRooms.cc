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

#include <cmath>
#include <limits>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

vs_graphs::core::semantic::Room *SemanticsManager::associateRooms(
    const Eigen::Vector3d clusterCentroid_World_in,
    const std::vector<vs_graphs::core::geometric::Plane *> &wallList_World_in,
    const std::vector<Eigen::Vector3d> &freeSpaceCluster_World_m_in,
    const std::unordered_set<int>      &excludedRoomIds_in)
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
    const std::vector<vs_graphs::core::semantic::Room *> allRooms_World =
        p_atlas->getAllRooms();

    /* Evaluate every room once against the complete cluster wall set. */
    for (vs_graphs::core::semantic::Room *p_room_World : allRooms_World)
    {
        /* Skip room if invalid */
        bool room_WorldIsBad{};
        if (!(p_room_World == nullptr) &&
            p_room_World->isBad(room_WorldIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room_World == nullptr || room_WorldIsBad)
        {
            continue;
        }

        /* Skip rooms already matched to another cluster in this cycle */
        int room_WorldId{};
        if (p_room_World->getId(room_WorldId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        if (excludedRoomIds_in.count(room_WorldId) > 0)
        {
            continue;
        }

        /* Extract room centroid */
        Eigen::Vector3d roomCenter_World{};
        if (p_room_World->getCentroid(roomCenter_World) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }

        /* Find the distance from the cluster center to the room center */
        const double roomCenterRelClusterCenterDistance =
            (roomCenter_World - clusterCentroid_World_in).norm();

        const bool roomSeparatedFromCluster = hasSeparatingFiniteWall(
            p_atlas->getAllPlanes(),
            roomCenter_World,
            clusterCentroid_World_in,
            static_cast<double>(p_sysParams->roomSeg.finiteWallBoundsMargin_m));

        /* Extract the walls from the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWallsList{};
        if (p_room_World->getWalls(roomWallsList) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

        /* Init a list to track the room wall ids */
        std::unordered_set<int> roomWallIds;

        /* Reserve space in list to track room ids */
        roomWallIds.reserve(roomWallsList.size());

        /* Iterate through room walls to extract ids of room */
        for (vs_graphs::core::geometric::Plane *p_roomWall : roomWallsList)
        {
            /* Skip invalid walls */
            if (p_roomWall != nullptr && !p_roomWall->isBad())
            {
                roomWallIds.insert(p_roomWall->getId());
            }
        }

        /* Init counters for walls on same and oposite sides of room */
        std::size_t sameSideMatches     = 0;
        std::size_t oppositeSideMatches = 0;

        /* Init a flag to see if the rooms share any walls */
        bool sharesAnyWall = false;

        /* Iterate through walls in room and see if they share walls */
        for (vs_graphs::core::geometric::Plane *p_candidateWall :
             wallList_World_in)
        {
            /* Skip invalid walls */
            if (p_candidateWall == nullptr || p_candidateWall->isBad())
            {
                continue;
            }

            /* If wall is not linked to room, skip */
            if (roomWallIds.count(p_candidateWall->getId()) == 0)
            {
                continue;
            }

            /* If wall is not skipped, then shares a wall */
            sharesAnyWall = true;

            /* Extract plane equation */
            Eigen::Vector4d candidateWallEquation =
                p_candidateWall->getGlobalEquation().coeffs();

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
                candidateWallEquation.head<3>().dot(clusterCentroid_World_in) +
                candidateWallEquation(3);

            /* Caldaulte the side of the room */
            const double roomSide =
                candidateWallEquation.head<3>().dot(roomCenter_World) +
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

                p_bestSharedRoom = p_room_World;
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

        for (const Eigen::Vector3d &freeSpacePoint_World_m :
             freeSpaceCluster_World_m_in)
        {
            if (freeSpacePoint_World_m.allFinite())
            {
                nearestFreeSpacePointDistance_m = std::min(
                    nearestFreeSpacePointDistance_m,
                    (roomCenter_World - freeSpacePoint_World_m).norm());
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

            p_nearestRoom = p_room_World;
        }
    }

    vs_graphs::core::semantic::Room *p_selectedRoom =
        p_bestSharedRoom != nullptr ? p_bestSharedRoom : p_nearestRoom;

    return p_selectedRoom;
}

} // namespace core
} // namespace vs_graphs
