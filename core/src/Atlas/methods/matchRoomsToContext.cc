/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Atlas.h"

#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
extern "C" void vsGraphsAtlasLockOrderBeforeMapSnapshot() __attribute__((weak));
#endif

namespace vs_graphs
{
namespace core
{

void Atlas::matchRoomsToContext(Map *p_newMap_in)
{
    if (!p_newMap_in)
        return;

    /* Candidate generation is intentionally separate from P4 verification.
     * This legacy method used cross-map centroids/normals and transferred live
     * walls before verification; it is retained as a disabled compatibility
     * entry point until the verified merge seam exists. */
    return;

    /* Snapshot Atlas membership before taking the room-context lock. Map
     * creation takes these locks in the opposite sequence by necessity. */
#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
    if (vsGraphsAtlasLockOrderBeforeMapSnapshot != nullptr)
    {
        vsGraphsAtlasLockOrderBeforeMapSnapshot();
    }
#endif
    const std::vector<Map *> allMaps = getAllMaps();

    std::unique_lock<std::mutex> lock(roomContextMutex);

    if (roomContextHistory.empty())
        return;

    /* Match BOTH detected rooms AND candidate/prospective rooms.
     * Candidate rooms need identity tags for cross-restart continuity. */
    std::vector<semantic::Room *> newRooms =
        p_newMap_in->getAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms =
        p_newMap_in->getAllCandidateMapRooms();
    newRooms.insert(newRooms.end(),
                    candidateRooms.begin(),
                    candidateRooms.end());

    if (newRooms.empty())
        return;

    std::vector<semantic::RoomContextSnapshot> allContext;
    for (const auto &entry : roomContextHistory)
        for (const auto &snap : entry.second)
            allContext.push_back(snap);

    if (allContext.empty())
        return;

    for (semantic::Room *p_room : newRooms)
    {
        bool roomIsBad{};
        if (!(!p_room) && p_room->isBad(roomIsBad) !=
                              semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (!p_room || roomIsBad)
            continue;

        bool roomHasRoomTag{};
        if (p_room->hasRoomTag(roomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // hasRoomTag cannot fail; continue as before.
        }
        if (roomHasRoomTag)
            continue;

        Eigen::Vector3d roomCentroid{};
        if (p_room->getCentroid(roomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        double bestDistance = std::numeric_limits<double>::max();
        const semantic::RoomContextSnapshot *p_bestMatch = nullptr;

        for (const semantic::RoomContextSnapshot &snap : allContext)
        {
            /* PREFER tag-based matching if snapshot has persistent tag.
             * This provides deterministic identity across restarts. */
            bool roomHasRoomTag2{};
            if ((!snap.roomTag.empty()) &&
                p_room->hasRoomTag(roomHasRoomTag2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // hasRoomTag cannot fail; continue as before.
            }
            if (!snap.roomTag.empty() && roomHasRoomTag2)
            {
                std::string roomTag2{};
                if (p_room->getRoomTag(roomTag2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getRoomTag cannot fail; continue as before.
                }
                if (roomTag2 == snap.roomTag)
                {
                    p_bestMatch  = &snap;
                    bestDistance = 0.0;
                    break;
                }
            }

            double distance = (snap.centroid - roomCentroid).norm();

            if (distance > kRoomContextMatchThreshold_m)
                continue;

            /* Verify wall-normal agreement: compare the first available
             * wall normal of the new room against each snapshot wall
             * normal. Accept when |cosθ| > kWallNormalAlignmentCosTheta. */
            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }
            if (roomWalls.empty() || snap.wallNormals.empty())
            {
                /* Fallback: accept on centroid distance alone when no wall
                 * normals are available for normal validation. */
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    p_bestMatch  = &snap;
                }
                continue;
            }

            std::optional<Eigen::Vector3d> newRoomNormal{};
            if (p_room->getWallNormalTowardRoom_World(roomWalls[0],
                                                      newRoomNormal) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWallNormalTowardRoom_World cannot fail; continue as
                // before.
            }
            if (!newRoomNormal)
                continue;

            bool normalOk = false;
            for (const Eigen::Vector3d &snapNormal : snap.wallNormals)
            {
                double dot = newRoomNormal->dot(snapNormal);
                if (std::abs(dot) > kWallNormalAlignmentCosTheta)
                {
                    normalOk = true;
                    break;
                }
            }

            if (!normalOk)
                continue;

            if (distance < bestDistance)
            {
                bestDistance = distance;
                p_bestMatch  = &snap;
            }
        }

        if (p_bestMatch)
        {
            if (p_room->setRoomTag("room_" +
                                   std::to_string(p_bestMatch->roomId)) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setRoomTag cannot fail; continue as before.
            }

            /* Locate the snapshot pointer in the stored history so the
             * room can hold a non-owning reference for WP3. */
            for (auto &entry : roomContextHistory)
            {
                for (auto &storedSnap : entry.second)
                {
                    if (storedSnap.roomId == p_bestMatch->roomId &&
                        storedSnap.centroid.isApprox(p_bestMatch->centroid))
                    {
                        if (p_room->setMatchedContext(&storedSnap) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            // setMatchedContext cannot fail; continue as
                            // before.
                        }
                        break;
                    }
                }
            }

            /* ----------------------------------------------------------- *
             * CONTINUITY: Transfer walls/passages from prior room instance.
             * The matched prior room (in stored map) already has accumulated
             * boundary walls. Re-associate them to this new room instance
             * so boundary validation continues from where it left off.
             * ----------------------------------------------------------- */

            /* Find the stored map that contains the prior room. */
            Map            *p_priorMap  = nullptr;
            semantic::Room *p_priorRoom = nullptr;
            for (Map *p_map : allMaps)
            {
                if (!p_map || p_map->isBad() || p_map == p_newMap_in)
                    continue;
                for (semantic::Room *r : p_map->getAllDetectedMapRooms())
                {
                    bool rIsBad{};
                    if ((r) && r->isBad(rIsBad) !=
                                   semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    int rId{};
                    if ((r && !rIsBad) &&
                        r->getId(rId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    if (r && !rIsBad && rId == p_bestMatch->roomId)
                    {
                        p_priorRoom = r;
                        p_priorMap  = p_map;
                        break;
                    }
                }
                if (p_priorRoom)
                    break;
            }

            if (p_priorRoom && p_priorMap)
            {
                int priorRoomId{};
                if (p_priorRoom->getId(priorRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::vector<geometric::Plane *> priorRoomWalls{};
                if (p_priorRoom->getWalls(priorRoomWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
                std::cout << "[Atlas] Prior semantic::Room#" << priorRoomId
                          << " has " << priorRoomWalls.size() << " walls"
                          << std::endl;

                /* Transfer walls from prior room to current room */
                std::vector<geometric::Plane *> priorRoomWalls2{};
                if (p_priorRoom->getWalls(priorRoomWalls2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
                for (geometric::Plane *p_wall : priorRoomWalls2)
                {
                    bool wallIsBad{};
                    if (!(!p_wall) &&
                        p_wall->isBad(wallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    if (!p_wall || wallIsBad)
                        continue;

                    /* Re-associate wall to new room */
                    bool priorRoomWasWallRemoved{};
                    if (p_priorRoom->removeWall(p_wall,
                                                priorRoomWasWallRemoved) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        priorRoomWasWallRemoved =
                            false; // rejected input reads as before
                    }
                    if (p_room->setWalls(p_wall) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // setWalls cannot fail; continue as before.
                    }

                    int priorRoomId2{};
                    if (p_priorRoom->getId(priorRoomId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int roomId2{};
                    if (p_room->getId(roomId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int wallGetId{};
                    if (p_wall->getId(wallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    std::cout << "[Atlas] Transferred Wall#" << wallGetId
                              << " from prior semantic::Room#" << priorRoomId2
                              << " to matched semantic::Room#" << roomId2
                              << std::endl;
                }

                /* Passages will be re-associated by associatePassagesToRooms()
                 */

                int roomId3{};
                if (p_room->getId(roomId3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::vector<geometric::Plane *> roomWalls2{};
                if (p_room->getWalls(roomWalls2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
                std::cout << "[Atlas] semantic::Room#" << roomId3 << " now has "
                          << roomWalls2.size()
                          << " walls (continuing from prior semantic::Room#"
                          << p_bestMatch->roomId << ")" << std::endl;
            }
            else
            {
                std::cout
                    << "[Atlas] NO prior room found for match (p_priorRoom="
                    << p_priorRoom << ", p_priorMap=" << p_priorMap << ")"
                    << std::endl;
            }

            int roomId4{};
            if (p_room->getId(roomId4) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            std::string roomTag3{};
            if (p_room->getRoomTag(roomTag3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomTag cannot fail; continue as before.
            }
            std::cout << "[Atlas] Matched room " << roomId4
                      << " in new map, tagged with identity \"" << roomTag3
                      << "\" (prior room " << p_bestMatch->roomId
                      << ", dist=" << bestDistance << " m)" << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
